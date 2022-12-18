#!/usr/bin/env python
import roslib; roslib.load_manifest("velma_task_cs_ros_interface")
import rospy
import PyKDL
import math
import subprocess
import tf_conversions.posemath as ourPosemath
import geometry_msgs.msg 
import numpy as np

from velma_common import *
from rcprg_ros_utils import exitError
from rcprg_planner.rcprg_planner import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma
from moveit_msgs.msg import Constraints, JointConstraint, AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive


class Velma:
    planner = None
    side = "right"


    def __init__(self, velma):
        self.velma = velma

    def homing(self):
        print("HomingHP")
        self.velma.enableMotors()
        self.velma.startHomingHP()
        self.velma.startHomingHT()

    def getTable1Position(self):
        print("Table1 position")
        return self.velma.getTf("Wo", "table1")

    def getTable2Position(self):
        print("Table2 position")
        return self.velma.getTf("Wo", "table2")

    def getObject1Position(self):
        print("Object1 position")
        return self.velma.getTf("Wo", "object1")

    def initializePlanner(self):
        ourPlanner = Planner(self.velma.maxJointTrajLen())
        if not ourPlanner.waitForInit():
            print("ERROR: not init")
        self.velma.waitForInit(timeout_s=5.0)
        oml = OctomapListener("/octomap_binary")
        if not oml:
            print("ERROR: not initializing octomap")
        print("Octomap initialization complete")
        rospy.sleep(1.0)
        octomap = oml.getOctomap(timeout_s=5.0)
        if not octomap:
            print("ERROR: not get octomap")
        if not ourPlanner.processWorld(octomap):
            print("ERROR: not process world octomap") 
        self.planner = ourPlanner

    def jimpCimpSwitch(self, modeSwitch):

        if modeSwitch == 1:   #joint imp
            print("Jnt_imp mode switch")
            self.velma.moveJointImpToCurrentPos(start_time=0.5)
            if self.velma.waitForJoint():
                print("ERROR: jimp wait for joint")

            rospy.sleep(0.5)
            diag = self.velma.getCoreCsDiag()
            if not diag.inStateJntImp():
                print("ERROR: core_cs not in state jnt imp")
            print("Finished jnt_imp mode")

        elif modeSwitch == 0:  #cart imp
            print ("Cart_imp mode switch")
            self.velma.moveCartImpRightCurrentPos(start_time=0.5)
            self.velma.waitForEffectorRight()
            rospy.sleep(0.5)

            diag = self.velma.getCoreCsDiag()
            if not diag.inStateCartImp():
                print ("ERROR: core_cs not in state cart imp")
            print("Finished cart_imp mode")

    def GetTWEForObj(self, z_off=0, x_off=0,y_off=0):
        print("Get gripper position for object")
        T_W_O = self.getObject1Position()
        ourRotation = PyKDL.Rotation.EulerZYX(0,0,0)
        T_W_O=PyKDL.Frame(ourRotation,T_W_O.p)
        T_O_G = PyKDL.Frame(PyKDL.Rotation.RPY(math.radians(-180), math.radians(0), math.radians(0)), PyKDL.Vector(x_off, y_off, z_off))
        
        T_G_P=self.velma.getTf("Gr", "Pr")
        T_P_E=self.velma.getTf("Pr", "Er")

        T_W_E = T_W_O*T_O_G*T_G_P*T_P_E
        return T_W_E

    def GetTWEForTab(self, z_off=0, x_off=0,y_off=0):
        print("Get gripper position not for object")
        T_W_O = self.getTable1Position()
        ourRotation = PyKDL.Rotation.EulerZYX(0,0,0)
        T_W_O=PyKDL.Frame(ourRotation,T_W_O.p)
        T_O_G = PyKDL.Frame(PyKDL.Rotation.RPY(math.radians(-180), math.radians(0), math.radians(0)), PyKDL.Vector(x_off, y_off, z_off))
        
        T_G_P=self.velma.getTf("Gr", "Pr")
        T_P_E=self.velma.getTf("Pr", "Er")
        T_W_E = T_W_O*T_O_G*T_G_P*T_P_E
        return T_W_E
        
    def GetIK(self, T_W_E):
        solver = KinematicsSolverVelma()
        if not solver:
            print("Failed to initialize IK solver")
        print("IK solver initialization completed")
        ik=[]
        f1 = True
        f2 = True
        f3 = True
        for q_torso in np.linspace(-math.pi/2, math.pi/2, 20):
            for elbow_ang in np.linspace(-math.pi, math.pi, 20):
                arm_q = solver.calculateIkArm(self.side, T_W_E, q_torso, elbow_ang, f1, f2, f3)
                if not arm_q[0] is None:
                    q_dict = {}
                    q_dict["torso_0_joint"] = q_torso

                    for i in range(len(arm_q)):
                            q_dict['right_arm_{}_joint'.format(i)] = arm_q[i]
                    ik.append(q_dict)
                if rospy.is_shutdown():
                    break
            if rospy.is_shutdown():
                break
        print("Inverse kinematic has been solved")
        print("Possible results: ", len(ik))
        return ik

    def getPlanAndMove(self, ik):
        rospy.loginfo("Planning trajectory...")
        self.velma.moveJointImpToCurrentPos(start_time=0.2)

        gc = [qMapToConstraints(i, 0.3, group=self.velma.getJointGroup("{}_arm_torso".format(self.side)))for i in ik]
        for i in range(15):
            js = self.velma.getLastJointState()[1]
            traj = self.planner.plan(js, gc, "impedance_joints",max_velocity_scaling_factor=0.15,planner_id="RRTConnect", num_planning_attempts = 20)
            if traj == None:

                print("Executing trajectory1...")
                continue
                

            # Wykonanie planu ruchu
            rospy.loginfo("Executing trajectory2...")
            if not self.velma.moveJointTraj(traj, start_time=0.5,
                                        position_tol = 10.0/180.0 * math.pi,
                                        velocity_tol = 10.0/180.0*math.pi ):
                exitError(5)
            if self.velma.waitForJoint() == 0:
                break
            else:
                rospy.loginfo("Failed to execute trajectory")



if __name__ == "__main__":

    rospy.init_node('pp', anonymous=False)
    rospy.sleep(0.5)

    print("Running python interface for Velma...")
    velma = Velma(VelmaInterface())
    velma.homing()
    velma.initializePlanner()
    velma.jimpCimpSwitch(1)
    my_tf = velma.getObject1Position()
    my_twe = velma.GetTWEForObj()
    my_ik = velma.GetIK(my_twe)
    velma.getPlanAndMove(my_ik)
    print("OK")
    exitError(0)
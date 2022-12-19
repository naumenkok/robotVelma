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

    def GetTWEForObj(self):
        print("Get gripper position for object")
        T_W_O = self.getObject1Position()
        ourRotation = PyKDL.Rotation.EulerZYX(0,0,0)
        T_W_O=PyKDL.Frame(ourRotation,T_W_O.p)
        T_O_G = PyKDL.Frame(PyKDL.Rotation.RPY(math.radians(-180), math.radians(0), math.radians(0)), PyKDL.Vector(0, 0, 0.1))
        
        T_G_P=self.velma.getTf("Gr", "Pr")
        T_P_E=self.velma.getTf("Pr", "Er")

        T_W_E = T_W_O*T_O_G*T_G_P*T_P_E
        return T_W_E

    def GetTWEForTab(self, side = "right"):
        print("Get gripper position not for object")
        if side == "right":
            T_W_O = self.getTable1Position()
            T_G_P=self.velma.getTf("Gr", "Pr")
            T_P_E=self.velma.getTf("Pr", "Er")
        elif side == "left":
            T_W_O = self.getTable2Position()
            T_G_P=self.velma.getTf("Gl", "Pl")
            T_P_E=self.velma.getTf("Pl", "El")
        ourRotation = PyKDL.Rotation.EulerZYX(0,0,0)
        T_W_O=PyKDL.Frame(ourRotation,T_W_O.p)
        T_O_G = PyKDL.Frame(PyKDL.Rotation.RPY(math.radians(-180), math.radians(0), math.radians(0)), PyKDL.Vector(0, 0, 0.2))
        T_W_E = T_W_O*T_O_G*T_G_P*T_P_E
        return T_W_E
        
    def GetIK(self, T_W_E, side = "right"):
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
                arm_q = solver.calculateIkArm(side, T_W_E, q_torso, elbow_ang, f1, f2, f3)
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

    def getPlanAndMove(self, ik, side = "right"):
        rospy.loginfo("Planning trajectory...")
        self.velma.moveJointImpToCurrentPos(start_time=0.2)

        gc = [qMapToConstraints(i, 0.16, group=self.velma.getJointGroup("{}_arm_torso".format(side)))for i in ik]
        for i in range(15):
            js = self.velma.getLastJointState()[1]
            traj = self.planner.plan(js, gc, "impedance_joints",max_velocity_scaling_factor=0.15,planner_id="RRTConnect", num_planning_attempts = 20)
            if traj == None:
                continue
            if not self.velma.moveJointTraj(traj, start_time=0.5,
                                        position_tol = 10.0/180.0*math.pi,
                                        velocity_tol = 10.0/180.0*math.pi ):
                exitError(5)
            if self.velma.waitForJoint() == 0:
                break
            else:
                rospy.loginfo("Failed to execute trajectory")

        self.jimpCimpSwitch(0)

        T_B_Wr = self.velma.getTf("B", "Wr")
        T_B_Wl = self.velma.getTf("B", "Wl")
        self.velma.moveCartImpRight([T_B_Wr], [0.5], [PyKDL.Frame()], [0.5], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
        self.velma.moveCartImpLeft([T_B_Wl], [0.5], [PyKDL.Frame()], [0.5], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
        
        goal=self.GetTWEForObj()
        if side == "right":
            self.velma.moveCartImpRight([goal], [2.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
            self.velma.waitForEffectorRight()
        elif side == "left":
            self.velma.moveCartImpLeft([goal], [2.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
            self.velma.waitForEffectorLeft()
        rospy.sleep(0.5)

    def closeGripper(self, side = "right"):
        q = [math.radians(180), math.radians(180), math.radians(180), math.radians(0)]
        if side == "right":
            self.velma.moveHandRight(q,[10,10,10,10],[0,0,0,0],5,True)
        elif side == "left":
            self.velma.moveHandLeft(q,[10,10,10,10],[10,10,10,10],5,True)
        print("Gripper closed")

    def openGripper(self, side = "right"):
        q = [math.radians(0), math.radians(0), math.radians(0), math.radians(0)]
        if side == "right":
            self.velma.moveHandRight(q,[10,10,10,10],[10,10,10,10],5,False)
        elif side == "left":
            self.velma.moveHandLeft(q,[10,10,10,10],[0,0,0,0],5,False)
        print("Gripper opened")

def getSide(my_tf):
        objPose = pm.toMsg(my_tf)
        objPosition = objPose.position.y
        if objPosition<0:
            side = "right"
        elif objPosition>0:
            side = "left"
        return side


if __name__ == "__main__":

    rospy.init_node('pp', anonymous=False)
    rospy.sleep(0.5)

    print("Running python interface for Velma...")
    velma = Velma(VelmaInterface())
    velma.homing()
    my_tf = velma.getObject1Position()
    side = getSide(my_tf)
    velma.openGripper(side)
    velma.initializePlanner()
    velma.jimpCimpSwitch(1)
    my_twe = velma.GetTWEForObj()
    my_ik = velma.GetIK(my_twe, side)
    velma.getPlanAndMove(my_ik, side)
    velma.closeGripper(side)
   # start = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
   #    'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
   #    'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
   #    'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
   # velma.getPlanAndMove(start, "left")
    print("OK")
    exitError(0)
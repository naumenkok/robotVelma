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

    def getTWEForObj(self, offsetZ = 0.3, side = "right"):
        print("Get gripper position for object")
        T_W_O = self.getObject1Position()
        ourRotation = PyKDL.Rotation.EulerZYX(0,0,0)
        T_W_O=PyKDL.Frame(ourRotation,T_W_O.p)
        T_O_G = PyKDL.Frame(PyKDL.Rotation.RPY(math.radians(0), math.radians(0), math.radians(0)), PyKDL.Vector(0, 0, offsetZ))
        if side == "right":
            T_G_P=self.velma.getTf("Gr", "Pr")
            T_P_E=self.velma.getTf("Pr", "Er")
        elif side == "left":
            T_G_P=self.velma.getTf("Gl", "Pl")
            T_P_E=self.velma.getTf("Pl", "El")
        T_W_E = T_W_O*T_O_G*T_G_P*T_P_E
        return T_W_E

    def getTWEForTab(self, offsetX = -0.4, offsetY = -0.4, offsetZ = 0.9, side = "right"):
        print("Get gripper position not for object")
        if side == "right":
            T_W_O = self.getTable1Position()
            T_G_P=self.velma.getTf("Gr", "Pr")
            T_P_E=self.velma.getTf("Pr", "Er")
            offsetY = 0.4
        elif side == "left":
            T_W_O = self.getTable2Position()
            T_G_P=self.velma.getTf("Gl", "Pl")
            T_P_E=self.velma.getTf("Pl", "El")
        ourRotation = PyKDL.Rotation.EulerZYX(0,0,0)
        T_W_O=PyKDL.Frame(ourRotation,T_W_O.p)
        T_O_G = PyKDL.Frame(PyKDL.Rotation.RPY(math.radians(-180), math.radians(0), math.radians(0)), PyKDL.Vector(offsetX, offsetY, offsetZ))
        T_W_E = T_W_O*T_O_G*T_G_P*T_P_E
        return T_W_E
        
    def getIK(self, T_W_E, side = "right"):
        solver = KinematicsSolverVelma()
        (solver.getArmLimits())
        if not solver:
            print("Failed to initialize IK solver")
        print("IK solver initialization completed")
        ik=[]
        f1 = True
        f2 = True
        f3 = True
        for q_torso in np.linspace(-math.pi/2, math.pi/2, 24):
            for elbow_ang in np.linspace(-math.pi, math.pi, 30):
                arm_q = solver.calculateIkArm(side, T_W_E, q_torso, elbow_ang, f1, f2, f3)
                if not arm_q[0] is None:
                    q_dict = {}
                    q_dict["torso_0_joint"] = q_torso

                    for i in range(len(arm_q)):
                            q_dict['{}_arm_{}_joint'.format(side, i)] = arm_q[i]
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

        gc = [qMapToConstraints(i, 0.25, group=self.velma.getJointGroup("{}_arm_torso".format(side)))for i in ik]
        for i in range(15):
            js = self.velma.getLastJointState()[-1]
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

    def moveCimp(self, offsetZ = 0.3, side = "right", table=False):
        self.jimpCimpSwitch(0)

        T_B_Wr = self.velma.getTf("B", "Wr")
        T_B_Wl = self.velma.getTf("B", "Wl")
        self.velma.moveCartImpRight([T_B_Wr], [0.5], [PyKDL.Frame()], [0.5], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
        self.velma.moveCartImpLeft([T_B_Wl], [0.5], [PyKDL.Frame()], [0.5], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
        
        if table:
            if side == "right":
                goal=self.getTWEForTab(side = "left")
            elif side == "left":
                goal=self.getTWEForTab(side = "right")
        else:
            goal=self.getTWEForObj(offsetZ, side)
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

    def moveUp(self, side = "right"):
        self.moveCimp(0.4, side)

    def startPos(self):
        start = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
       'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
       'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
       'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
        self.velma.moveJoint(start, 8.0, start_time=0.1, position_tol=15.0/180.0*math.pi)

    def moveOneJoint(self):
        js = self.velma.getLastJointState()[1]
        gc=Constraints()
        gc.joint_constraints=[JointConstraint('right_arm_6_joint', 0.0, 0.01, 0.01, 0.01)]
        traj = self.planner.plan(js, [gc], "impedance_joints",max_velocity_scaling_factor=0.15,planner_id="RRTConnect")
        if not self.velma.moveJointTrajAndWait(traj):
            print('error')

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
    velma.initializePlanner()
    #velma.moveOneJoint()
    my_tf = velma.getObject1Position()
    print(pm.toMsg(my_tf))
    handSide = getSide(my_tf)
    velma.openGripper(handSide)
    print(handSide)
    velma.jimpCimpSwitch(1)
    my_twe = velma.getTWEForObj(side = handSide)
    my_ik = velma.getIK(my_twe, handSide)
    velma.getPlanAndMove(my_ik, handSide)
    velma.moveCimp(side = handSide)
    velma.closeGripper(handSide)
    velma.moveUp(handSide)
    velma.jimpCimpSwitch(1)
    if handSide == "right":
        my_twe_tabl = velma.getTWEForTab(side = "left")
    elif handSide == "left":
        my_twe_tabl = velma.getTWEForTab(side = "right")
    
    my_ik_tabl = velma.getIK(my_twe_tabl, handSide)
    velma.getPlanAndMove(my_ik_tabl, handSide)
    velma.moveCimp(side = handSide, table=True)
    
    velma.openGripper(handSide)
    velma.startPos()
    print("OK")
    exitError(0)
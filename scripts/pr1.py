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

    def get_start_joints(self):
        return self.velma.getLastJointState()[-1]

    def homing(self):
        print("Homing and wait for interface")
        self.velma.waitForInit()
        self.velma.enableMotors()
        self.velma.startHomingHP()
        self.velma.startHomingHT()

    def getTable1Position(self):
        print("Table1 tf transform")
        return self.velma.getTf("Wo", "table1")

    def getTable2Position(self):
        print("Table2 tf transform")
        return self.velma.getTf("Wo", "table2")

    def getObject1Position(self):
        print("Object1 tf transform")
        return self.velma.getTf("Wo", "object1")

    def getPlanner(self):
        ourPlanner = Planner(self.velma.maxJointTrajLen())
        print("Planner: wait until planner interface is not initialized")
        if not ourPlanner.waitForInit():
            print("Planner: no planner interface initialized")
            exitError(2)
        print("Planner: planner interface initialized")
        print('Octomap: wait until octomap interface is not initialized')
        oml = OctomapListener("/octomap_binary")
        rospy.sleep(1.0)
        if not oml:
            print("Octomap: no octomap initialized")
            exitError(3)
        print("Octomap: initialization complete")
        rospy.sleep(1.0)
        octomap = oml.getOctomap(timeout_s=5.0)
        if octomap == 0:
            print("Octomap: no octomap")
            exitError(4)
        if ourPlanner.processWorld(octomap) == 0:
            print("Octomap: no world processed")
            exitError(5)
        else:
            print("Octomap: done-------------------") 
        self.planner = ourPlanner

    def jimpCimpSwitch(self, modeSwitch, side='right'):

        if modeSwitch == 1:
            print("Jnt_imp: switch core_cs to jnt_imp mode")
            self.velma.moveJointImpToCurrentPos()
            if self.velma.waitForJoint():
                print("ERROR: no joint space movement")
                exitError(6)
            rospy.sleep(1.0)
            if not self.velma.getCoreCsDiag().inStateJntImp():
                print("ERROR: core_cs not in state jnt imp")
                exitError(7)
            else:
                print("Jnt_imp: finished jnt_imp mode")

        elif modeSwitch == 0:
            print ("Cart_imp: switch core_cs to cart_imp mode")
            rospy.sleep(0.5)
            self.velma.moveCartImpCurrentPos(side)
            rospy.sleep(0.5)
            print('Cart_imp: wait for completion of end-effector motion in cartesian impedance mode')
            if self.velma.waitForEffector(side):
                exitError(8)
            rospy.sleep(0.5)
            if not self.velma.getCoreCsDiag().inStateCartImp():
                print ("ERROR: core_cs not in state cart imp")
            else:
                print("Cart_imp: finished cart_imp mode")

    def getTWEForObj(self, offsetZ = 0.2, side = 'right'):
        T_W_O = self.getObject1Position()
        sidde=side[0]
        T_G_P=self.velma.getTf("G"+sidde, "P"+sidde)
        T_P_E=self.velma.getTf("P"+sidde, "E"+sidde)
        ourRotation = PyKDL.Rotation.EulerZYX(0,0,0)
        T_W_O=PyKDL.Frame(ourRotation,T_W_O.p)
        T_O_G = PyKDL.Frame(PyKDL.Rotation.RPY(math.radians(-180), math.radians(0), math.radians(0)), PyKDL.Vector(0, 0, offsetZ))
        T_W_E = T_W_O*T_O_G*T_G_P*T_P_E
        print('Gripper: found end position')
        return T_W_E

    def getTWEForTab(self, side = 'right'):
        offsetZ = 1.4
        T_W_O = self.getTable1Position() if side == 'right' else self.getTable2Position()
        offsetX = -0.3 if  side == 'right' else -0.4
        offsetY = 0.3 if  side == 'right' else -0.4
        sidde=side[0]
        T_G_P = self.velma.getTf("G"+sidde, "P"+sidde)
        T_P_E = self.velma.getTf("P"+sidde, "E"+sidde)
        ourRotation = PyKDL.Rotation.EulerZYX(0,0,0)
        T_W_O = PyKDL.Frame(ourRotation,T_W_O.p)
        T_O_G = PyKDL.Frame(PyKDL.Rotation.RPY(math.radians(-180), math.radians(-180), math.radians(0)), PyKDL.Vector(offsetX, offsetY, offsetZ))
        T_W_E = T_W_O*T_O_G*T_G_P*T_P_E
        return T_W_E
        
    def getIK(self, T_W_E, side = 'right'):
        solver = KinematicsSolverVelma()
        solver.getArmLimits()
        if not solver:
            print("Inverse kinematic: failed initialization")
        else:
            print("Inverse kinematic: initialized")
        ik=[]
        f1 = True
        f2 = True
        f3 = True
        for q_torso in np.linspace(-math.pi/2, math.pi/2, 24):
            for elbow_ang in np.linspace(-math.pi, math.pi, 24):
                arm_q = solver.calculateIkArm(side, T_W_E, q_torso, elbow_ang, f1, f2, f3)
                q_dict = {}
                if arm_q[0]:
                    for i in range(len(arm_q)):
                        joint_name=side+'_arm_'+str(i)+'_joint'
                        q_dict[joint_name] = arm_q[i]
                    q_dict["torso_0_joint"] = q_torso
                    ik.append(q_dict)
        print("Inverse kinematic: done-------------------------")
        return ik

    def getTrajectory(self, ik, side = 'right'):
        print("Plan: start planning")
        self.velma.moveJointImpToCurrentPos(start_time=0.2)
        gc=[]
        for j in ik:
            gc.append(qMapToConstraints(j, 0.2, group=self.velma.getJointGroup(side+"_arm_torso")))
        for i in range(5):
            traj = self.planner.plan(self.velma.getLastJointState()[1], gc, side+"_arm_torso",max_velocity_scaling_factor=0.15,planner_id="RRTConnect", num_planning_attempts = 20)
            if self.velma.moveJointTraj(traj, start_time=1.0, position_tol = 10.0/180.0*math.pi, velocity_tol = 10.0/180.0*math.pi)==0:
                exitError(10)
            if traj == None:
                continue
            if self.velma.waitForJoint()==0:
                print("Done")
                break
            else:
                continue

    def moveCimp(self, offsetZ = 0.1, side = 'right', table=False):
        self.jimpCimpSwitch(0, side)
        print('Cimp: move in cimp')
        N=None
        T_B_W = self.velma.getTf("Wo", "W"+side[0])
        pdl=PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5))
        self.velma.moveCartImp(side, [T_B_W], [2.0], [PyKDL.Frame()], [0.5], N, N, pdl, start_time=0.5)
        if table:
            if side == 'right':
                goal=self.getTWEForTab('left')
            elif side == 'left':
                goal=self.getTWEForTab('right')
        else:
            goal=self.getTWEForObj(offsetZ, side)
        self.velma.moveCartImp(side, [goal], [2.0], N, N, N, N, pdl, start_time=0.5)
        if self.velma.waitForEffector(side) != 0:
            exitError(11)
        rospy.sleep(0.5)

    def closeGripper(self, side = 'right'):
        print('Gripper: close gripper')
        q = [math.radians(180), math.radians(180), math.radians(180), math.radians(0)]
        self.velma.moveHand(side,q,[1, 1, 1, 1],[0.1, 0.1, 0.1, 0.1],0.1,True)
        if self.velma.waitForHand(side) != 0:
            exitError(12)
        else:
            print("Gripper: completion of hand movement")

    def openGripper(self, side = 'right'):
        print('Gripper: open gripper')
        q = [math.radians(0), math.radians(0), math.radians(0), math.radians(0)]
        self.velma.moveHand(side,q,[1, 1, 1, 1],[0.1, 0.1, 0.1, 0.1],0.1)
        if self.velma.waitForHand(side) != 0:
            exitError(13)
        else:
            print("Gripper: completion of hand movement")

    def moveUp(self, side = 'right'):
        print('Cimp: move up')
        self.moveCimp(0.4, side)

    def startPos(self, joints , side = 'right'):
        start = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
       'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
       'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
       'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
        self.getTrajectory(joints, side)

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

    rospy.init_node('pr1', anonymous=True)

    print("Start")
    velma = Velma(VelmaInterface())
    velma.homing()
    velma.getPlanner()
    #velma.moveOneJoint()
    my_tf = velma.getObject1Position()
    start_joint_pos = velma.get_start_joints()
    print(pm.toMsg(my_tf))
    handSide = getSide(my_tf)
    velma.openGripper(handSide)
    print("Object is on "+handSide)
    velma.jimpCimpSwitch(1, handSide)
    my_twe = velma.getTWEForObj(side = handSide)
    my_ik = velma.getIK(my_twe, handSide)
    velma.getTrajectory(my_ik, handSide)
    velma.moveCimp(side = handSide)
    velma.closeGripper(handSide)
    velma.moveUp(handSide)
    velma.jimpCimpSwitch(1, handSide)
    if handSide == "right":
        my_twe_tabl = velma.getTWEForTab(side = "left")
    elif handSide == "left":
        my_twe_tabl = velma.getTWEForTab(side = "right")
    
    my_ik_tabl = velma.getIK(my_twe_tabl, handSide)
    velma.getTrajectory(my_ik_tabl, handSide)
    velma.moveCimp(side = handSide, table=True)
    velma.openGripper(handSide)
    velma.moveUp(handSide)
    velma.jimpCimpSwitch(1, handSide)
    velma.startPos([start_joint_pos], handSide)

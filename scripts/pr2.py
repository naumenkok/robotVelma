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
        self.saveRot=None

    def get_start_joints(self):
        return self.velma.getLastJointState()[-1]

    def homing(self):
        print("Homing and wait for interface")
        self.velma.waitForInit()
        self.velma.enableMotors()
        self.velma.startHomingHP()
        self.velma.startHomingHT()

    def getcabinetPosition(self):
        return self.velma.getTf("Wo", "cabinet")

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

    def getTWEForObj(self, saveRotation, offsetX = -0.2, offsetY = 0, offsetZ = 0.1):
        T_W_O = self.getcabinetPosition()
        sidde='r'
        T_G_P=self.velma.getTf("G"+sidde, "P"+sidde)
        T_P_E=self.velma.getTf("P"+sidde, "E"+sidde)
        if saveRotation==True:
            a=T_W_O.M
            a.DoRotY(3.14) 
            a.DoRotX(3.14)
            self.saveRot=a
            print(self.saveRot)
        T_W_O = PyKDL.Frame(self.saveRot,T_W_O.p)
        T_O_G = PyKDL.Frame(PyKDL.Rotation.RPY(math.radians(0), math.radians(90), math.radians(0)), PyKDL.Vector(offsetX, offsetY, offsetZ))
        T_W_E = T_W_O*T_O_G*T_G_P*T_P_E
        print('Gripper: found end position')
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
            gc.append(qMapToConstraints(j, 0.1, group=self.velma.getJointGroup(side+"_arm_torso")))
        for i in range(5):
            traj = self.planner.plan(self.velma.getLastJointState()[1], gc, side+"_arm_torso",max_velocity_scaling_factor=0.15,planner_id="RRTConnect", num_planning_attempts = 20)
            if self.velma.moveJointTraj(traj, start_time=1.0, position_tol = 20.0/180.0*math.pi, velocity_tol = 10.0/180.0*math.pi)==0:
                exitError(10)
            if traj == None:
                continue
            if self.velma.waitForJoint()==0:
                print("Done")
                break
            else:
                continue

    def moveCimp(self, offsetX, offsetY, offsetZ):
        print('Cimp: move in cimp')
        N=None
        T_B_W = self.velma.getTf("Wo", "Wr")
        imp=PyKDL.Wrench(PyKDL.Vector(700, 700, 700), PyKDL.Vector(150, 150, 150))
        imp1=PyKDL.Wrench(PyKDL.Vector(300, 300, 300), PyKDL.Vector(150, 150, 150))
        imp2=PyKDL.Wrench(PyKDL.Vector(200, 200, 200), PyKDL.Vector(150, 150, 150))
        imp_time=[0.5, 1, 1.5]
        pdl=PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5))
        self.velma.moveCartImp('right', [T_B_W], [2.0], [PyKDL.Frame()], [0.5], N, N, pdl, start_time=0.5)
        goal=self.getTWEForObj(False, offsetX,offsetY,offsetZ)
        self.velma.moveCartImp('right', [goal], [2.0], N, N, [imp, imp1, imp2], imp_time, pdl, start_time=0.5)
        if self.velma.waitForEffector('right') != 0:
            exitError(11)
        rospy.sleep(0.5)

    def closeGripper(self, two_fingers, one_finger, side = 'right'):
        print('Gripper: close gripper')
        q = [math.radians(0), math.radians(two_fingers), math.radians(one_finger), math.radians(0)]
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
        self.moveCimp(0,0,0)

    def startPos(self, joints , side = 'right'):
        self.getTrajectory(joints, side)

    def moveOneJoint(self):
        js = self.velma.getLastJointState()[1]
        gc=Constraints()
        gc.joint_constraints=[JointConstraint('right_arm_6_joint', 0.0, 0.01, 0.01, 0.01)]
        traj = self.planner.plan(js, [gc], "impedance_joints",max_velocity_scaling_factor=0.15,planner_id="RRTConnect")
        if not self.velma.moveJointTrajAndWait(traj):
            print('error')


if __name__ == "__main__":
    rospy.init_node('pr2', anonymous=True)
    print("Start")
    velma = Velma(VelmaInterface())
    velma.homing()
    velma.getPlanner()
    #Koniec inicjalizacji
    #Zapamietaj poczatkowa poz
    start_joint_pos = velma.get_start_joints()
    #Przeksztalcenia
    velma.openGripper()
    velma.jimpCimpSwitch(1)
    my_tf = velma.getcabinetPosition()
    my_twe = velma.getTWEForObj(True)
    my_ik = velma.getIK(my_twe)
    velma.getTrajectory(my_ik)
    velma.jimpCimpSwitch(0)
    velma.moveCimp(0.0, 0.0, 0.1)
    velma.closeGripper(0, 100)
    velma.moveCimp(-0.13, 0.0, 0.1)
    velma.closeGripper(0, 0)
    velma.moveCimp(0.0, 0.0, 0.1)
    velma.closeGripper(80, 0)
    velma.moveCimp(-0.3,-0.4, 0.1)
    velma.openGripper()
    velma.startPos([start_joint_pos])

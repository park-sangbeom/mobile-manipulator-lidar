#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import tf
import numpy
import copy
import time
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


rospy.init_node('pickandplace', anonymous=True)  
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("arm")
scene = moveit_commander.PlanningSceneInterface()

goal=[-0.3004884065435629,-1.89270333133803,1.9277298476124773,-1.6044670780114405,-1.5697175550123657,2.8396582340451655]

arm_group.go(goal, wait=True)
arm_group.stop()

print("Ready to move")

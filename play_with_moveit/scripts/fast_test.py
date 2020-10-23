#!/usr/bin/env python
import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import RobotTrajectory
from play_with_moveit.KukaRobot import *
from play_with_moveit.Ur10eRobot import *

rospy.init_node("test")
# kuka = KukaRobot()
# kuka.go_to_pose_goal([0.0, 0.5, 1.4, 0, 0, 0])
ur10e = Ur10eRobot()
ur10e.go_to_joint_goal([0, 0, 0, 0, 0, 0])
# ur10e.go_to_pose_goal([0.65, -0.3, 1.4,0.7071068, 0.7071068, 0, 0], ur10e.GRIPPER_GROUP)
# group = MoveGroupCommander("kuka_group")
# group.set_joint_value_target([1.57, 1.57, 0])
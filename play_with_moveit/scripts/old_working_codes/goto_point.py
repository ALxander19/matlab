#!/usr/bin/env python
import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface


rospy.init_node("goto_point")
robot = RobotCommander()
gripper_group = MoveGroupCommander("gripper_group")
scene = PlanningSceneInterface()
rospy.sleep(2)

xyz_goal = [0, 0.0, 1.5]
gripper_group.set_position_target(xyz_goal, gripper_group.get_end_effector_link())
gripper_group.go()

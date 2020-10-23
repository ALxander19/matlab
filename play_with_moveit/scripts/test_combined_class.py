#!/usr/bin/env python
from play_with_moveit.CombinedRobots import *
from play_with_moveit.KukaRobot import *
from play_with_moveit.Ur10eRobot import *

import rospy
import tf.transformations
import numpy as np
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped


def h():
    global scene
    scene.remove_attached_object(ur10e.GRIPPER_EF_LINK)
    scene.remove_attached_object(kuka.VACUUM_EF_LINK)
    rospy.sleep(1)
    scene.remove_world_object()
    combined_robots.go_to_joint_goal([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])


rospy.init_node("test_combined_calss")
rospy.on_shutdown(h)
scene = PlanningSceneInterface()
combined_robots = CombinedRobots()
kuka = KukaRobot()
ur10e = Ur10eRobot()
# this amount of sleep is a MUST 
rospy.sleep(2)

# add object and table to the planning scene
kuka_table_size = [1.0, 0.4, 1.0]
kuka_table_pose_list = [0.0, 0.4, kuka_table_size[2]/2.0]
kuka_table_name = "kuka_table"
kuka_table_pose = PoseStamped()
kuka_table_pose.header.frame_id = "world"
kuka_table_pose.header.stamp = rospy.Time.now()
kuka_table_pose.pose.position.x = kuka_table_pose_list[0]
kuka_table_pose.pose.position.y = kuka_table_pose_list[1]
kuka_table_pose.pose.position.z = kuka_table_pose_list[2]
scene.add_box(name=kuka_table_name, pose=kuka_table_pose, size=kuka_table_size)
rospy.sleep(1)

kuka_object_size = [0.2, 0.2, 0.2]
kuka_object_pose_list = [kuka_table_pose_list[0], kuka_table_pose_list[1], kuka_table_size[2] + kuka_object_size[2]/2]
kuka_object_name = "kuka_object"
kuka_object_pose = PoseStamped()
kuka_object_pose.header.frame_id = "world"
kuka_object_pose.header.stamp = rospy.Time.now()
kuka_object_pose.pose.position.x = kuka_object_pose_list[0]
kuka_object_pose.pose.position.y = kuka_object_pose_list[1]
kuka_object_pose.pose.position.z = kuka_object_pose_list[2]
scene.add_box(name=kuka_object_name, pose=kuka_object_pose, size=kuka_object_size)
rospy.sleep(1)

# add object and table to the planning scene
ur10e_table_size = [1.0, 0.4, 1.0]
ur10e_table_pose_list = [0.4, -0.4, ur10e_table_size[2]/2.0]
ur10e_table_name = "ur10e_table"
ur10e_table_pose = PoseStamped()
ur10e_table_pose.header.frame_id = "world"
ur10e_table_pose.header.stamp = rospy.Time.now()
ur10e_table_pose.pose.position.x = ur10e_table_pose_list[0]
ur10e_table_pose.pose.position.y = ur10e_table_pose_list[1]
ur10e_table_pose.pose.position.z = ur10e_table_pose_list[2]
scene.add_box(name=ur10e_table_name, pose=ur10e_table_pose, size=ur10e_table_size)
rospy.sleep(1)

ur10e_object_size = [0.01, 0.01, 0.3]
ur10e_object_pose_list = [ur10e_table_pose_list[0], ur10e_table_pose_list[1], ur10e_table_size[2] + ur10e_object_size[2]/2]
ur10e_object_name = "ur10e_object"
ur10e_object_pose = PoseStamped()
ur10e_object_pose.header.frame_id = "world"
ur10e_object_pose.header.stamp = rospy.Time.now()
ur10e_object_pose.pose.position.x = ur10e_object_pose_list[0]
ur10e_object_pose.pose.position.y = ur10e_object_pose_list[1]
ur10e_object_pose.pose.position.z = ur10e_object_pose_list[2]
scene.add_box(name=ur10e_object_name, pose=ur10e_object_pose, size=ur10e_object_size)
rospy.sleep(1)

"""
go to picking pose
"""
ur10e_pick_approach_dist = [0, 0.03, 0]
ur10e_pick_retreat_dist = [0, -0.1, 0.1]
contact_margin = 0.05
ur10e_gripper_pose = [ur10e_object_pose_list[0], ur10e_object_pose_list[1] - contact_margin, ur10e_object_pose_list[2], 0, 0, 0]

kuka_pick_approach_dist = [0.0, 0, -0.05] 
kuka_pick_retreat_dist = [0.0, 0, 0.1]
contact_margin = 0.02
kuka_vacuum_height = kuka_object_pose_list[2] + kuka_object_size[2]/2.0 + contact_margin - kuka_pick_approach_dist[2]
kuka_vacuum_pose = [kuka_object_pose_list[0], kuka_object_pose_list[1], kuka_vacuum_height, 0, 0, 0]

pick_combined_goals = []
# kuka must be before ur10e
pick_combined_goals.append(kuka_vacuum_pose)
pick_combined_goals.append(ur10e_gripper_pose)
combined_robots.go_to_pose_goal(pick_combined_goals)

"""
pick the object
"""
# kuka approach, pick the object, then retreat
kuka.pick_object(kuka_object_name, kuka_table_name, kuka_pick_approach_dist, kuka_pick_retreat_dist)
# ur10e approach, pick the object, then retreat
ur10e.pick_object(ur10e_object_name, ur10e_table_name, ur10e_pick_approach_dist, ur10e_pick_retreat_dist, ur10e_object_size[0])

"""
go to the place location
"""
# approach, place the object and then retreat
ur10e_place_approach_dist = [0, 0.1, -0.1]
ur10e_place_retreat_dist = [0, -0.1, 0.1]
ur10e_gripper_pose[0] = ur10e_gripper_pose[0] + 0.2
ur10e_gripper_pose[2] = ur10e_gripper_pose[2] - ur10e_place_approach_dist[2] + contact_margin

kuka_place_approach_dist = [0.0, 0, -0.1]
kuka_place_retreat_dist = [0.0, 0, 0.2]
kuka_vacuum_pose[0] = kuka_vacuum_pose[0] + 0.2
kuka_vacuum_pose[2] = kuka_vacuum_pose[2] - ur10e_place_approach_dist[2] + contact_margin

place_combined_goals = []
# kuka must be before ur10e
place_combined_goals.append(kuka_vacuum_pose)
place_combined_goals.append(ur10e_gripper_pose)
combined_robots.go_to_pose_goal(place_combined_goals)

"""
place the object
"""
kuka.place_object(kuka_object_name, kuka_table_name, kuka_place_approach_dist, kuka_place_retreat_dist)
ur10e.place_object(ur10e_object_name, ur10e_table_name, ur10e_place_approach_dist, ur10e_place_retreat_dist)

while not rospy.is_shutdown():
    pass
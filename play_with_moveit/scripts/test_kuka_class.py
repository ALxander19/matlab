#!/usr/bin/env python
from play_with_moveit.KukaRobot import *
import rospy
import tf.transformations
import numpy as np
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

rospy.init_node("test_kuka_calss")
scene = PlanningSceneInterface()
kuka = KukaRobot()
# this amount of sleep is a MUST 
rospy.sleep(3)

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

# go to the picking pose
kuka_pick_approach_dist = [0.0, 0, -0.05] 
kuka_pick_retreat_dist = [0.0, 0, 0.1]
contact_margin = 0.02
kuka_vacuum_height = kuka_object_pose_list[2] + kuka_object_size[2]/2.0 + contact_margin - kuka_pick_approach_dist[2]
kuka_vacuum_pose = [kuka_object_pose_list[0], kuka_object_pose_list[1], kuka_vacuum_height, 0, 0, 0]
kuka.go_to_pose_goal(kuka_vacuum_pose, kuka.VACUUM_GROUP)

# approach, pick the object, then retreat
kuka.pick_object(kuka_object_name, kuka_table_name, kuka_pick_approach_dist, kuka_pick_retreat_dist)

# go to the place location
kuka.move_tool_in_straight_line([0.2, 0.0, 0.0], avoid_collisions=True)

# approach, place the object and then retreat
kuka_place_approach_dist = [0.0, 0, -0.1]
kuka_place_retreat_dist = [0.0, 0, 0.2]
kuka.place_object(kuka_object_name, kuka_table_name, kuka_place_approach_dist, kuka_place_retreat_dist)

# remove objects from the planning scene
def h():
    global scene
    scene.remove_attached_object(kuka.VACUUM_EF_LINK)
    rospy.sleep(1)
    scene.remove_world_object()
    kuka.go_to_joint_goal([0, 0, 0, 0, 0, 0, 0])

rospy.on_shutdown(h)
while not rospy.is_shutdown():
    pass

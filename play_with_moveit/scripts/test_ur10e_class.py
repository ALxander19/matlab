#!/usr/bin/env python
from play_with_moveit.Ur10eRobot import *
import rospy
import tf.transformations
import numpy as np
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped


rospy.init_node("test_ur10e_calss")
scene = PlanningSceneInterface()
# this amount of sleep is a MUST 
rospy.sleep(2)

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

ur10e = Ur10eRobot()
# go to the picking pose
ur10e_pick_approach_dist = [0, 0.03, 0]
ur10e_pick_retreat_dist = [0, -0.1, 0.1]
contact_margin = 0.05

ur10e_gripper_pose = [ur10e_object_pose_list[0], ur10e_object_pose_list[1] - contact_margin, ur10e_object_pose_list[2], 0, 0, 0]
ur10e.go_to_pose_goal(ur10e_gripper_pose, ur10e.GRIPPER_GROUP)

# approach, pick the object, then retreat
ur10e.pick_object(ur10e_object_name, ur10e_table_name, ur10e_pick_approach_dist, ur10e_pick_retreat_dist, ur10e_object_size[0])

# go to the place location
# with respect to the tool
ur10e.move_tool_in_straight_line([0.2, 0, 0.0], avoid_collisions=True)

# approach, place the object and then retreat
ur10e_place_approach_dist = [0, 0.1, -0.1]
ur10e_place_retreat_dist = [0, -0.1, 0.1]
ur10e.place_object(ur10e_object_name, ur10e_table_name, ur10e_place_approach_dist, ur10e_place_retreat_dist)

def h():
    global scene
    scene.remove_attached_object(ur10e.GRIPPER_EF_LINK)
    rospy.sleep(1)
    scene.remove_world_object()
    ur10e.go_to_joint_goal([0, 0, 0, 0, 0, 0])

rospy.on_shutdown(h)
while not rospy.is_shutdown():
    pass


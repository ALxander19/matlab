#!/usr/bin/env python
import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, String

rospy.init_node("pick_and_place_scene")
scene = PlanningSceneInterface()

# publish the scene info.
kuka_table_size_pub = rospy.Publisher("kuka_table_size", Float32MultiArray, queue_size=1, latch=True)
kuka_table_pose_pub = rospy.Publisher("kuka_table_pose", Float32MultiArray, queue_size=1, latch=True)
kuka_table_name_pub = rospy.Publisher("kuka_table_name", String, queue_size=1, latch=True)
kuka_object_size_pub = rospy.Publisher("kuka_object_size", Float32MultiArray, queue_size=1, latch=True)
kuka_object_pose_pub = rospy.Publisher("kuka_object_pose", Float32MultiArray, queue_size=1, latch=True)
kuka_object_name_pub = rospy.Publisher("kuka_object_name", String, queue_size=1, latch=True)

ur10e_table_size_pub = rospy.Publisher("ur10e_table_size", Float32MultiArray, queue_size=1, latch=True)
ur10e_table_pose_pub = rospy.Publisher("ur10e_table_pose", Float32MultiArray, queue_size=1, latch=True)
ur10e_table_name_pub = rospy.Publisher("ur10e_table_name", String, queue_size=1, latch=True)
ur10e_object_size_pub = rospy.Publisher("ur10e_object_size", Float32MultiArray, queue_size=1, latch=True)
ur10e_object_pose_pub = rospy.Publisher("ur10e_object_pose", Float32MultiArray, queue_size=1, latch=True)
ur10e_object_name_pub = rospy.Publisher("ur10e_object_name", String, queue_size=1, latch=True)
# this amount of sleep is a MUST 
rospy.sleep(10)

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

kuka_table_size_pub.publish(Float32MultiArray(data=kuka_table_size))
kuka_table_pose_pub.publish(Float32MultiArray(data=kuka_table_pose_list))
kuka_table_name_pub.publish(String(data=kuka_table_name))
kuka_object_size_pub.publish(Float32MultiArray(data=kuka_object_size))
kuka_object_pose_pub.publish(Float32MultiArray(data=kuka_object_pose_list))
kuka_object_name_pub.publish(String(data=kuka_object_name))

ur10e_table_size_pub.publish(Float32MultiArray(data=ur10e_table_size))
ur10e_table_pose_pub.publish(Float32MultiArray(data=ur10e_table_pose_list))
ur10e_table_name_pub.publish(String(data=ur10e_table_name))
ur10e_object_size_pub.publish(Float32MultiArray(data=ur10e_object_size))
ur10e_object_pose_pub.publish(Float32MultiArray(data=ur10e_object_pose_list))
ur10e_object_name_pub.publish(String(data=ur10e_object_name))
rospy.spin()
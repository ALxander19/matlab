#!/usr/bin/env python

import rospy
import tf
import numpy as np

from moveit_commander import MoveGroupCommander, RobotCommander

from moveit_msgs.msg import MoveItErrorCodes, Grasp, PlaceLocation, RobotTrajectory
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, quaternion_matrix, translation_from_matrix, translation_matrix
from geometry_msgs.msg import WrenchStamped, PoseStamped, PointStamped, Pose
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class CombinedRobots():

    def __init__(self):
        
        self.COMBINED_CAMERAS_GROUP = "combined_cameras"
        self.COMBINED_GRIPPERS_GROUP = "combined_grippers"
        
        self.COMBINED_CAMERAD_EF_LINKS = ["kuka_camera_ef", "ur10e_camera_ef"]
        self.COMBINED_GRIPPERS_EF_LINKS = ["kuka_vacuum_ef", "ur10e_gripper_ef"]
        
        self.__PLANNING_JOINT_NAMES = ["kuka_joint_a1","kuka_joint_a2","kuka_joint_a3","kuka_joint_a4","kuka_joint_a5",
                            "kuka_joint_a6","kuka_joint_a7","ur10e_shoulder_pan_joint", "ur10e_shoulder_lift_joint", "ur10e_elbow_joint",
                                        "ur10e_wrist_1_joint", "ur10e_wrist_2_joint", "ur10e_wrist_3_joint"]
              
        # create move group commander
        self.__combined_cameras_commander = MoveGroupCommander(self.COMBINED_CAMERAS_GROUP)
        self.__combined_grippers_commander = MoveGroupCommander(self.COMBINED_GRIPPERS_GROUP)

    """
    Go to pose goal. Plan and execute and wait for the controller respons
    :param pose_goal list in this form [x, y, z, r, p, y]
     or [x, y, z, qx, qy, qz, qw]
     or [x, y, z]
    :type list of int
    :param ef_link
    :type: string
    """
    def go_to_pose_goal(self, pose_goal, group_name=None):
        
        if (group_name == self.COMBINED_CAMERAS_GROUP):
            group = self.__combined_cameras_commander
            ef_links = self.COMBINED_CAMERAS_EF_LINKS
            
        else:
            group = self.__combined_grippers_commander
            ef_links = self.COMBINED_GRIPPERS_EF_LINKS

        for i in range(len(pose_goal)):
            if (len(pose_goal[i]) == 6 or len(pose_goal[i]) == 7):
                group.set_pose_target(pose_goal[i], ef_links[i])                                           

            elif (len(pose_goal[i]) == 3):
                group.set_position_target(pose_goal[i], ef_links[i])  
                
            else:
                print("Invalid inputs for goal ", i)
        group.go()
        
        
    """
    Go to joint goal
    """
    def go_to_joint_goal(self, joint_goal):
        
        if(len(joint_goal) == len(self.__PLANNING_JOINT_NAMES)):
            joint_goal_msg = JointState()
            joint_goal_msg.name = self.__PLANNING_JOINT_NAMES
            joint_goal_msg.position = joint_goal
            self.__combined_cameras_commander.set_joint_value_target(joint_goal_msg)
            self.__combined_cameras_commander.go()
        else:
            print("Invalid inputs")
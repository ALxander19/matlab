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

class Ur10eRobot():

    def __init__(self):
        
        self.GRIPPER_GROUP = "ur10e_electric_gripper"
        self.CAMERA_GROUP = "ur10e_camera"
        self.CHANGER_GROUP = "ur10e_tool_changer"
        self.GRIPPER_EF_GROUP = "ur10e_gripper_ef"
        
        self.GRIPPER_EF_LINK = "ur10e_gripper_ef"
        self.CAMERA_EF_LINK = "ur10e_camera_ef"
        self.CHANGER_EF_LINK = "ur10e_changer_ef"
        
        self.___PLANNING_JOINT_NAMES = ["ur10e_shoulder_pan_joint", "ur10e_shoulder_lift_joint", "ur10e_elbow_joint",
                                        "ur10e_wrist_1_joint", "ur10e_wrist_2_joint", "ur10e_wrist_3_joint"]

        self.__GRIPPER_EF_JOINT_NAMES = ["ur10e_upper_finger_base", "ur10e_lower_finger_base"]
        
        self.__FORCE_SENSOR_TOPIC = "/ur10e_force_topic"
        
        # create move group commander
        self.__gripper_group_commander = MoveGroupCommander(self.GRIPPER_GROUP)
        self.__camera_group_commander = MoveGroupCommander(self.CAMERA_GROUP)
        self.__changer_group_commander = MoveGroupCommander(self.CHANGER_GROUP)
        self.__gripper_ef_commander = MoveGroupCommander(self.GRIPPER_EF_GROUP)
        

    """
    Update the current value from force sensor
    """
    def update_force_sensor_value(self, force_sensor_value):
        self.__force_sensor_value = force_sensor_value
        
    
    """
    Get the current value from force sensor
    """
    def get_force_sensor_value(self):
        return self.__force_sensor_value
    
    """
    Open electric gripper
    """
    def open_electric_gripper(self):
        self.__gripper_ef_commander.set_joint_value_target([0, 0])
        self.__gripper_ef_commander.go()
        
    def close_electric_gripper(self, object_width):
        max_width = 0.02
        if (object_width > max_width):
            print ("cannot close gripper, object width exceeds the max width")
        else:
            gripper_dist = (max_width - object_width) / 2
            self.__gripper_ef_commander.set_joint_value_target([-gripper_dist, -gripper_dist])
            self.__gripper_ef_commander.go()
     
    def move_tool_in_straight_line(self, pose_goal, avoid_collisions=True, group_name=None, n_way_points=2):
        
        if (group_name == self.CAMERA_GROUP):
            group = self.__camera_group_commander
        
        elif (group_name == self.CHANGER_GROUP):
            group = self.__changer_group_commander
            
        else:
            group = self.__gripper_group_commander
            
        way_points_list = []
        way_point = Pose()
        x_way_points = np.linspace(0, pose_goal[0], n_way_points)
        y_way_points = np.linspace(0, pose_goal[1], n_way_points)
        z_way_points = np.linspace(0, pose_goal[2], n_way_points)
        qx_way_points = np.linspace(0, 0, n_way_points)
        qy_way_points = np.linspace(0, 0, n_way_points)
        qz_way_points = np.linspace(0, 0, n_way_points)
        qw_way_points = np.linspace(0, 0, n_way_points)
        for i in range(n_way_points):
            way_point.position.x = x_way_points[i]
            way_point.position.y = y_way_points[i]
            way_point.position.z = z_way_points[i]
            way_point.orientation.x = qx_way_points[i]
            way_point.orientation.y = qy_way_points[i]
            way_point.orientation.z = qz_way_points[i]
            way_point.orientation.w = qw_way_points[i]
            way_points_list.append(way_point)
                    
        group.set_pose_reference_frame(group.get_end_effector_link())
        # group.set_goal_tolerance(0.001)

        plan, fraction = group.compute_cartesian_path(way_points_list, 0.005, 0.0)
        print (fraction)
        # print plan

        # post processing for the trajectory to ensure its validity 
        last_time_step = plan.joint_trajectory.points[0].time_from_start.to_sec
        new_plan = RobotTrajectory()
        new_plan.joint_trajectory.header = plan.joint_trajectory.header
        new_plan.joint_trajectory.joint_names = plan.joint_trajectory.joint_names
        new_plan.joint_trajectory.points.append(plan.joint_trajectory.points[0])

        for i in range(1, len(plan.joint_trajectory.points)):
            point = plan.joint_trajectory.points[i]
            if (point.time_from_start.to_sec > last_time_step):
                new_plan.joint_trajectory.points.append(point)
            last_time_step = point.time_from_start.to_sec

        # execute the trajectory
        group.execute(new_plan)
                
    
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
        
        if (group_name == self.CAMERA_GROUP):
            group = self.__camera_group_commander

        elif (group_name == self.CHANGER_GROUP):
            group = self.__changer_group_commander
            
        else:
            group = self.__gripper_group_commander

        if (len(pose_goal) == 6 or len(pose_goal) == 7):
            group.set_pose_target(pose_goal)  
            group.go()                                           

        elif (len(pose_goal) == 3):
            group.set_position_target(pose_goal)  
            group.go() 
            
        else:
            print("Invalid inputs")
        
        
    """
    Go to joint goal
    """
    def go_to_joint_goal(self, joint_goal):
        
        if(len(joint_goal) == 6):
            joint_goal_msg = JointState()
            joint_goal_msg.name = self.___PLANNING_JOINT_NAMES
            joint_goal_msg.position = joint_goal
            self.__camera_group_commander.set_joint_value_target(joint_goal_msg)
            self.__camera_group_commander.go()
        else:
            print("Invalid inputs")
                 
    """
    pick an object using electric gripper
    This function assumes that the electric gripper is already in a place near to the object. With an appropriate orientation.
    So what is left in order to pick. is to approach the object. turn electric gripper on and then retreat.
    """
    def pick_object(self, object_name, table_name, approach_dist, retreat_dist, object_width):
        # disable collision between object and table
        self.__gripper_group_commander.set_support_surface_name(table_name)

        # approach the object
        self.move_tool_in_straight_line(approach_dist, avoid_collisions=False)
        
        # attach object and open gripper
        self.__gripper_group_commander.attach_object(object_name)
        
        # close electric gripper
        self.close_electric_gripper(object_width)
        
        rospy.sleep(2)
        
        # retreat 
        self.move_tool_in_straight_line(retreat_dist, avoid_collisions=True)

    
    """
    Place an object using electric gripper
    """
    def place_object(self, object_name, table_name, approach_dist, retreat_dist):
        # disable collision between object and table
        self.__gripper_group_commander.set_support_surface_name(table_name)

        # approach the table
        self.move_tool_in_straight_line(approach_dist, avoid_collisions=False)
        
        # detach object and turn off gripper
        self.__gripper_group_commander.detach_object(object_name)
        
        # open electric gripper
        self.open_electric_gripper()
        
        rospy.sleep(2)
        
        # retreat 
        self.move_tool_in_straight_line(retreat_dist, avoid_collisions=True)
  
    
    
#!/usr/bin/env python

import rospy
import numpy as np

from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import WrenchStamped, Pose
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState


class KukaRobot:

    def __init__(self):

        self.VACUUM_GROUP = "kuka_vacuum_gripper"
        self.CAMERA_GROUP = "kuka_camera"
        self.IMPACT_GROUP = "kuka_impact_wrench"

        self.VACUUM_EF_LINK = "kuka_vacuum_ef"
        self.CAMERA_EF_LINK = "kuka_camera_ef"
        self.IMPACT_EF_LINK = "kuka_impact_ef"

        self.___PLANNING_JOINT_NAMES = ["kuka_joint_a1", "kuka_joint_a2", "kuka_joint_a3", "kuka_joint_a4",
                                        "kuka_joint_a5",
                                        "kuka_joint_a6", "kuka_joint_a7"]

        self.__FORCE_SENSOR_TOPIC = "/kuka_force_topic"
        self.__VACUUM_STATUS_TOPIC = "/kuka_vacuum_topic"

        # create move group commander
        self.__vacuum_group_commander = MoveGroupCommander(self.VACUUM_GROUP)
        self.__camera_group_commander = MoveGroupCommander(self.CAMERA_GROUP)
        self.__impact_group_commander = MoveGroupCommander(self.IMPACT_GROUP)

        # initialize vacuum services
        self.__vacuum_on_service = rospy.ServiceProxy("/on", Empty)
        self.__vacuum_off_service = rospy.ServiceProxy("/off", Empty)

        # initialize feedback variables
        self.__vacuum_status = Bool()
        self.__force_sensor_value = WrenchStamped()

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
    Update Vacuum status
    """
    def update_vacuum_status(self, vacuum_status):
        self.__vacuum_status = vacuum_status

    """
    Get Vacuum status
    """
    def get_vacuum_status(self):
        return self.__vacuum_status

    """ 
    Turn on Vacuum gripper
    """
    def vacuum_gripper_on(self):
        self.__vacuum_on_service.wait_for_service(0.2)
        request = EmptyRequest()
        self.__vacuum_on_service(request)

    """ 
    Turn off Vacuum gripper
    """
    def vacuum_gripper_off(self):
        self.__vacuum_off_service.wait_for_service(0.2)
        request = EmptyRequest()
        self.__vacuum_off_service(request)

    """
    move in a straight line with respect to the tool reference frame
    """
    def move_tool_in_straight_line(self, pose_goal, avoid_collisions=True, group_name=None, n_way_points=2):

        if group_name == self.CAMERA_GROUP:
            group = self.__camera_group_commander

        elif group_name == self.IMPACT_GROUP:
            group = self.__impact_group_commander

        else:
            group = self.__vacuum_group_commander

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

        plan, fraction = group.compute_cartesian_path(way_points_list, 0.005, 0.0, avoid_collisions)
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
            if point.time_from_start.to_sec > last_time_step:
                new_plan.joint_trajectory.points.append(point)
            last_time_step = point.time_from_start.to_sec

        # execute the trajectory
        group.execute(new_plan)

    """
    Go to pose goal. Plan and execute and wait for the controller response
    :param pose_goal list in this form [x, y, z, r, p, y]
     or [x, y, z, qx, qy, qz, qw]
     or [x, y, z]
    :type list of int
    :param ef_link
    :type: string
    """
    def go_to_pose_goal(self, pose_goal, group_name=None):

        if group_name == self.CAMERA_GROUP:
            group = self.__camera_group_commander

        elif group_name == self.IMPACT_GROUP:
            group = self.__impact_group_commander

        else:
            group = self.__vacuum_group_commander

        if len(pose_goal) == 6 or len(pose_goal) == 7:
            group.set_pose_target(pose_goal)
            group.go()

        elif len(pose_goal) == 3:
            group.set_position_target(pose_goal)
            group.go()

        else:
            print("Invalid inputs")

    """
    Go to joint goal
    """
    def go_to_joint_goal(self, joint_goal):

        if len(joint_goal) == 7:
            joint_goal_msg = JointState()
            joint_goal_msg.name = self.___PLANNING_JOINT_NAMES
            joint_goal_msg.position = joint_goal
            self.__camera_group_commander.set_joint_value_target(joint_goal_msg)
            self.__camera_group_commander.go()
            
        else:
            print("Invalid inputs")

    """
    pick an object using vacuum gripper
    This function assumes that the vacuum gripper is already in a place near to the object.
     With an appropriate orientation.
    So what is left in order to pick. is to approach the object. turn vacuum gripper on and then retreat.
    """
    def pick_object(self, object_name, table_name, approach_dist, retreat_dist):
        # disable collision between object and table
        self.__vacuum_group_commander.set_support_surface_name(table_name)

        # approach the object
        self.move_tool_in_straight_line(approach_dist, avoid_collisions=True)

        # attach object and open gripper
        self.__vacuum_group_commander.attach_object(object_name)

        # turn on vacuum gripper
        self.vacuum_gripper_on()

        rospy.sleep(2)

        # retreat 
        self.move_tool_in_straight_line(retreat_dist, avoid_collisions=True)

    """
    Place an object using vacuum gripper
    """
    def place_object(self, object_name, table_name, approach_dist, retreat_dist):
        # disable collision between object and table
        self.__vacuum_group_commander.set_support_surface_name(table_name)

        # approach the table
        self.move_tool_in_straight_line(approach_dist, avoid_collisions=True)

        # detach object and turn off gripper
        self.__vacuum_group_commander.detach_object(object_name)

        # turn off vacuum gripper
        self.vacuum_gripper_off()

        rospy.sleep(2)

        # retreat 
        self.move_tool_in_straight_line(retreat_dist, avoid_collisions=True)

#!/usr/bin/env python
import rospy
from moveit_commander import PlanningSceneInterface, RobotCommander, MoveGroupCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import MoveItErrorCodes

if __name__ == '__main__':

    try:
        rospy.init_node("ik_node")
        robot = RobotCommander()
        group_name = "manipulator"
        group = MoveGroupCommander(group_name)
        connection = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        rospy.loginfo("waiting for ik server")
        connection.wait_for_service()
        rospy.loginfo("server found")

        request = GetPositionIKRequest()
        request.ik_request.group_name = group_name
        request.ik_request.ik_link_name = group.get_end_effector_link()
        request.ik_request.ik_link_names = robot.get_link_names(group_name)
        request.ik_request.robot_state = robot.get_current_state()

        # get cartesian state
        cartesian= raw_input("enter cartesian state x y z r p y \n")
        [x, y, z, roll, pitch, yaw] = [float(idx) for idx in cartesian.split(' ')]
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z

        quat = quaternion_from_euler(roll, pitch, yaw)
        request.ik_request.pose_stamped.pose.orientation.x = quat[0]
        request.ik_request.pose_stamped.pose.orientation.y = quat[1]
        request.ik_request.pose_stamped.pose.orientation.z = quat[2]
        request.ik_request.pose_stamped.pose.orientation.w = quat[3]

        response = connection(request)
        if response.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("solution found")
            rospy.loginfo(response.solution.joint_state)

        else:
            rospy.loginfo("NO solution found")

    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
import rospy
from moveit_commander import PlanningSceneInterface, RobotCommander, MoveGroupCommander
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import MoveItErrorCodes

if __name__ == '__main__':

    try:
        rospy.init_node("fk_node")
        robot = RobotCommander()
        group_name = "manipulator"
        group = MoveGroupCommander(group_name)
        connection = rospy.ServiceProxy("/compute_fk", GetPositionFK)
        rospy.loginfo("waiting for fk server")
        connection.wait_for_service()
        rospy.loginfo("server found")

        request = GetPositionFKRequest()
        request.fk_link_names = robot.get_link_names(group_name)
        request.header.frame_id = robot.get_root_link()
        request.robot_state.joint_state.name = group.get_joints()[0:-1]
        

        # get joint state
        joint_states= raw_input("enter joint state j1 j2 j3 j4 j5 j6 j7 \n")
        joints= [float(idx) for idx in joint_states.split(' ')]
        request.robot_state.joint_state.position = joints

        response = connection(request)
        if response.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("solution found")
            rospy.loginfo(response.pose_stamped[-1])

        else:
            rospy.loginfo("NO solution found")

    except rospy.ROSInterruptException:
        pass

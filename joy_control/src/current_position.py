import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


if __name__ == '__main__':

  rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
  robot = moveit_commander.RobotCommander()

  group_name = "ur10e_electric_gripper"
  group = moveit_commander.MoveGroupCommander(group_name)

  print "============ Printing robot current pose ============"
  print group.get_current_pose()
  print ""

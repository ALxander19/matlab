#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
import numpy

if __name__ == "__main__":
    try:
        rospy.init_node("ft_sensor_read")
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message("force_topic", WrenchStamped)
            print("force x = ", msg.wrench.force.x)
            print("force y = ", msg.wrench.force.y)
            print("force z = ", msg.wrench.force.z)
            print("torque x = ", msg.wrench.torque.x)
            print("torque y = ", msg.wrench.torque.y)
            print("torque z = ", msg.wrench.torque.z)
            print ("----------------------------------")
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        pass

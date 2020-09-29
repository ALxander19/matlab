#!/usr/bin/env python
import rospy
from pruebas.msg import Num

def callback(msg):

  print msg.arrray

if __name__ == '__main__':

  rospy.init_node('subscriber_matlab')
  rospy.Subscriber('array', Num, callback)

  print "Subscriber Started!"

  rospy.spin()

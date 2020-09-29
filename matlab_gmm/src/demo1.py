#!/usr/bin/env python
import rospy
import matlab as mat
import matlab.engine as meng
from matlab_gmm.msg import Num

if __name__ == '__main__':

  rospy.init_node('publisher')
  pub = rospy.Publisher('arrays', Num, queue_size=10)
  rate = rospy.Rate(1)

  eng = meng.start_matlab()
  
  msg = Num()
  out1_list = []
  out2_list = []

  try:
    out1,out2 = eng.demo1(nargout=2)

    for i in out1[0]:
      out1_list.append(i)

    for i in out2[0]:
      out2_list.append(i)

    msg.arrray = out1_list
    pub.publish(msg)
    msg.arrray = out2_list
    pub.publish(msg)

  except:
    print 'Error in python script'
    eng.quit()


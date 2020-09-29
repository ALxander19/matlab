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
  out11_list = []
  out12_list = []
  out21_list = []
  out22_list = []

  try:
    out11,out12,out21,out22 = eng.demo2(nargout=4)

    for i in out11[0]:
      out11_list.append(i)

    for i in out12[0]:
      out12_list.append(i)

    for i in out21[0]:
      out21_list.append(i)

    for i in out22[0]:
      out22_list.append(i)

    msg.arrray = out11_list
    pub.publish(msg)
    msg.arrray = out12_list
    pub.publish(msg)
    msg.arrray = out21_list
    pub.publish(msg)
    msg.arrray = out22_list
    pub.publish(msg)

  except:
    print 'Error in python script'
    eng.quit()


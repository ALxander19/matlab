#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import numpy as np
from std_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import *


def dh(d, theta, a, alpha):
  """Calcular la matriz de transformacion homogenea asociada con los parametros de Denavit-Hartenberg"""
  
  sth = np.sin(theta)
  cth = np.cos(theta)
  sa = np.sin(alpha)
  ca = np.cos(alpha)
  T = np.array([[cth, -ca*sth, sa*sth, a*cth],[sth, ca*cth, -sa*cth, a*sth],[0.0, sa, ca, d],[0.0, 0.0, 0.0, 1.0]])
  
  return T


def fkine(q):
  """Calcular la cinematica directa del robot UR5 dados sus valores articulares"""
  
  # Longitudes (en metros)
  l0 = 0.0892
  l1 = 0.425
  l2 = 0.392
  l3 = 0.1093
  l4 = 0.09475
  l5 = 0.0825
  # Matrices DH (completar)
  T1 = dh(l0, q[0], 0, np.pi/2)
  T2 = dh(0 , q[1], -l1, 0 )
  T3 = dh(0 , q[2], -l2, 0 )
  T4 = dh(l3, q[3], 0, np.pi/2)
  T5 = dh(l4, q[4], 0, -np.pi/2)
  T6 = dh(l5, q[5], 0, 0 )
  # Efector final con respecto a la base
  T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)
  
  return T


def jacobian_position(q, delta=0.0001):
  """Jacobiano analitico para la posicion"""

  # Alocacion de memoria
  J = np.zeros((3,6))
  # Transformacion homogenea inicial (usando q)
  T = fkine(q)

  # Iteracion para la derivada de cada columna
  for i in xrange(6):
  
    # Copiar la configuracion articular inicial
    dq = q
    # Incrementar la articulacion i-esima usando un delta
    dq[i] = dq[i] + delta
    # Transformacion homogenea luego del incremento (q+dq)
    Td = fkine(dq)
    # Aproximacion del Jacobiano de posicion usando diferencias finitas
    J[0,i] = (1/delta)*(Td[0,3]-T[0,3])
    J[1,i] = (1/delta)*(Td[1,3]-T[1,3])
    J[2,i] = (1/delta)*(Td[2,3]-T[2,3])

  return J


def jacobian_pose(q, delta=0.0001):

  """Jacobiano analitico para la posicion y orientacion (usando un cuaternion)"""

  # Implementar este Jacobiano aqui
  J = np.zeros((7,6))
  # Cuaternion inicial (usando q)
  T = fkine(q)
  quat0 = TF2xyzquat(T)
  
  for i in xrange(6):
  
    # Copiar la configuracion articular inicial
    dq = q
    # Incrementar la articulacion i-esima usando un delta
    dq[i] = dq[i] + delta
    # Cuaternion luego del incremento (q+dq)
    Td = fkine(dq)
    quatd = TF2xyzquat(Td)
    # Aproximacion del Jacobiano de posicion usando diferencias finitas
    J[0,i] = (1/delta)*(quatd[0] - quat0[0])
    J[1,i] = (1/delta)*(quatd[1] - quat0[1])
    J[2,i] = (1/delta)*(quatd[2] - quat0[2])
    J[3,i] = (1/delta)*(quatd[3] - quat0[3])
    J[4,i] = (1/delta)*(quatd[4] - quat0[4])
    J[5,i] = (1/delta)*(quatd[5] - quat0[5])
    J[6,i] = (1/delta)*(quatd[6] - quat0[6])

  return J


def TF2xyzquat(T):

  """Convert a homogeneous transformation matrix into the a vector containing the pose of the robot"""
  quat = rot2quat(T[0:3,0:3])
  res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
  return np.array(res)


global press_key
press_key = "0"


def callback(msg):
  
  global press_key
  press_key = msg.data
  print "Press key recived: "+str(press_key)


if __name__ == '__main__':
    
  rospy.init_node("test_gazebo")
  rospy.Subscriber("/keys", String, callback)
  robot_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

  print "Waiting for server..."
  robot_client.wait_for_server()
  print "Connected to server"

  joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
  q0 = [0.0, -1.0, 1.7, -2.2, -1.6, 0.0]

  g = FollowJointTrajectoryGoal()
  g.trajectory = JointTrajectory()
  g.trajectory.joint_names = joint_names

  # Initial position
  g.trajectory.points = [ JointTrajectoryPoint(positions=q0, velocities=[0]*6,time_from_start=rospy.Duration(2.0))]
  robot_client.send_goal(g)
  robot_client.wait_for_result()
  rospy.sleep(1)

  q = q0
  T = fkine(q)
  x = T[0:3,3]
  freq = 100
  dt = 1.0/freq
  rate = rospy.Rate(freq)

  k = 0.5; epsilon = 1.5e-2

  while not rospy.is_shutdown():

    robot_client.cancel_goal()

    # Modification of the motion

    # Moving up
    if press_key== 'w':
      
      xf=np.array([x[0], x[1], x[2] + 0.5])
      J= jacobian_position(q)
      T= fkine(q)
      xi= T[0:3,3]
      e= xi-xf    # Error
      de = -k*e    # Control Law
      dq = np.linalg.pinv(J).dot(de)    # Variacion de congiguracion articular
      q = q+dt*dq

    # Moving down
    elif press_key== 's':

      xf=np.array([x[0], x[1], x[2] - 0.5])
      J= jacobian_position(q)
      T= fkine(q)
      xi= T[0:3,3]
      e= xi-xf    # Error
      de = -k*e    # Control Law
      dq = np.linalg.pinv(J).dot(de)    # Variacion de congiguracion articular
      q = q+dt*dq

    # Moving to the right
    elif press_key== 'd':

      xf=np.array([x[0], x[1] - 0.5, x[2]])
      J= jacobian_position(q)
      T= fkine(q)
      xi= T[0:3,3]
      e= xi-xf    # Error
      de = -k*e    # Control Law
      dq = np.linalg.pinv(J).dot(de)    # Variacion de congiguracion articular
      q = q+dt*dq

    # Moving to the left
    elif press_key== 'a':

      xf=np.array([x[0], x[1]+0.5, x[2]])
      J= jacobian_position(q)
      T= fkine(q)
      xi= T[0:3,3]
      e= xi-xf    # Error
      de = -k*e    # Control Law
      dq = np.linalg.pinv(J).dot(de)    # Variacion de congiguracion articular
      q = q+dt*dq

    # Moving to the front
    elif press_key== 'q':

      xf=np.array([x[0]+0.7, x[1], x[2]])
      J= jacobian_position(q)
      T= fkine(q)
      xi= T[0:3,3]
      e= xi-xf    # Error
      de = -k*e    # Control Law
      dq = np.linalg.pinv(J).dot(de)    # Variacion de congiguracion articular
      q = q+dt*dq

    # Moving to the back
    elif press_key== 'e':
    
      xf=np.array([x[0]-0.7, x[1], x[2]])
      J= jacobian_position(q)
      T= fkine(q)
      xi= T[0:3,3]
      e= xi-xf    # Error
      de = -k*e    # Control Law
      dq = np.linalg.pinv(J).dot(de)    # Variacion de congiguracion articular
      q = q+dt*dq

    # Limite articular
    for i in range(6):
      if(q[i] > 3.14):
        q[i] = 3.14
      elif(q[i] < - 3.14):
        q[i] > - 3.14

    g.trajectory.points = [ JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(0.008))]
    robot_client.send_goal(g)
    robot_client.wait_for_result()

    rate.sleep()

  robot_client.cancel_goal()

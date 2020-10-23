import numpy as np


def eulertoquatertion(roll,pitch,yaw):
  
  R = np.zeros((3, 3))
  R[0,0] = np.cos(yaw)*np.cos(pitch)
  R[1,0] = np.sin(yaw)*np.cos(pitch)
  R[2,0] = -np.sin(pitch)
  R[0,1] = np.cos(yaw)*np.sin(pitch)*np.sin(roll) - np.sin(yaw)*np.cos(roll)
  R[1,1] = np.sin(yaw)*np.sin(pitch)*np.sin(roll) + np.cos(yaw)*np.cos(roll)
  R[2,1] = np.cos(pitch)*np.sin(roll)
  R[0,2] = np.cos(yaw)*np.sin(pitch)*np.cos(roll) + np.sin(yaw)*np.sin(roll)
  R[1,2] = np.sin(yaw)*np.sin(pitch)*np.cos(roll) - np.cos(yaw)*np.sin(roll)
  R[2,2] = np.cos(pitch)*np.cos(roll)
  R = np.round(R,decimals=4) 

  dEpsilon = 1e-6
  quat = 4*[0.,]

  quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
  if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
    quat[1] = 0.0
  else:
    quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
  if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
    quat[2] = 0.0
  else:
    quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
  if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
    quat[3] = 0.0
  else:
    quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

  return np.array(quat)


if __name__ == '__main__':

  roll = 0.0; pitch = 1.5708/2 ; yaw = 0.0
  print eulertoquatertion(roll,pitch,yaw)
  print "------------"
  roll = 0.0; pitch = 1.6208 ; yaw = 0.0
  print eulertoquatertion(roll,pitch,yaw)
  print "------------"
  roll = 0.0; pitch = 1.5208 ; yaw = 0.0
  print eulertoquatertion(roll,pitch,yaw)
  print "------------"
   

#!/usr/bin/env python
import matlab as mat
import matlab.engine as meng

if __name__ == '__main__':

  eng = meng.start_matlab()

  try:
    out11,out12,out21,out22 = eng.demo2(nargout=4)
    print out11
    print '---------'
    print out12
    print '---------'
    print out21
    print '---------'
    print out22

  except:
    print 'Error in python script'
    eng.quit()


""" Robotics Toolbox for Python -- Test wrench transform
"""

import _robot
from robot.testparser import *


tests = '''
echo off
##
## Test wtrans function.
##

##
## W is a 6 element list
W = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
type(W)
## rotated 90 deg about x-axis
R = rpy2r(90.0, pitch=0.0, yaw=0.0, deg=True);
T = r2t(R);
Wt = wtrans(T,W)
type(Wt)
Wt.shape

##
## W is a (6,) array
W = array([1.0, 0.0, 0.0, 0.0, 0.0, 1.0])
type(W)
W.shape
## rotated 90 deg about x-axis
R = rpy2r(90.0, pitch=0.0, yaw=0.0, deg=True);
T = r2t(R);
Wt = wtrans(T,W)
type(Wt)
Wt.shape

##
## W is a 1x6 array
W = array([[1.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
type(W)
W.shape
## rotated 90 deg about x-axis
R = rpy2r(90.0, pitch=0.0, yaw=0.0, deg=True);
T = r2t(R);
Wt = wtrans(T,W)
type(Wt)
Wt.shape

##
## W is a 6x1 array
W = array(mat([1.0, 0.0, 0.0, 0.0, 0.0, 1.0]).T)
type(W)
W.shape
## rotated 90 deg about x-axis
R = rpy2r(90.0, pitch=0.0, yaw=0.0, deg=True);
T = r2t(R);
Wt = wtrans(T,W)
type(Wt)
Wt.shape

##
## W is a 6x1 matrix
W = mat([1.0, 0.0, 0.0, 0.0, 0.0, 1.0]).T
type(W)
W.shape
## rotated 90 deg about x-axis
R = rpy2r(90.0, pitch=0.0, yaw=0.0, deg=True);
T = r2t(R);
wt = wtrans(T,W)
type(Wt)
Wt.shape

##
## W translated 1 unit along -y-axis and rotated 90 deg 
## about x-axis.
W = [1, 0, 0, 0, 0, 1]
Tt = transl([0, -1, 0]);
Tr = rpy2tr(90.0, pitch=0.0, yaw=0.0, deg=True);
T = Tt
Wt = wtrans(T,W)
'''

if __name__ == "__main__" :

    testparser(tests)

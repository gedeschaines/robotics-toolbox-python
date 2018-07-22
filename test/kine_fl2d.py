""" Robotics Toolbox for Python -- Kinematics tests for four-link 2D
"""

import _robot
from robot.testparser import *


tests = '''
echo off
from robot.fourlink2d import *
echo on

fl2d

# from zero to nominal pose
qn = arg2array(qn)
tn = fkine(fl2d, qn)  # transform from base to tool (end-effector)
q = ikine(fl2d, tn, q0=qz, m=[1,1,1,1,0,0])  # same as qn?
qn
t = fkine(fl2d, q)  # same as tn?
tn
ikine(fl2d, t, q0=qz, m=[1,1,1,1,0,0], verbose=2)  # same as qn?
qn
'''

if __name__ == "__main__" :

    testparser(tests)
    

""" Robotics Toolbox for Python -- Kinematics tests for four-link 3D
"""

import _robot
from robot.testparser import *


tests = '''
echo off
from robot.fourlink3d import *
echo on

fl3d

# from zero to nominal pose
qn = arg2array(qn)
tn = fkine(fl3d, qn)  # transform from base to tool (end-effector)
q = ikine(fl3d, tn, q0=qz, m=[1,1,1,1,0,0])  # same as qn?
qn
t = fkine(fl3d, q)  # same as tn?
tn
ikine(fl3d, t, q0=qz, m=[1,1,1,1,0,0], verbose=2)  # same as qn?
qn
'''

if __name__ == "__main__" :

    testparser(tests)
    

""" Robotics Toolbox for Python -- Kinematics tests for Puma 560 (mod)
"""

import _robot
from robot.testparser import *


tests = '''
echo off
from robot.puma560akb import *
echo on

## **** Kinematics Test ****

p560m

T0 = fkine(p560m, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q0 = ikine(p560m, T0, alpha=0.4, ilimit=2000)
fkine(p560m, q0)

T1 = fkine(p560m, [pi/2, -pi/4, 3*pi/4, -pi/8, 0.0, -pi/5])
q1 = ikine(p560m, T1, alpha=0.4, ilimit=2000)
fkine(p560m, q1)
'''

if __name__ == "__main__" :

    testparser(tests)
    

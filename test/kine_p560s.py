""" Robotics Toolbox for Python -- Kinematics tests for Puma 560 (std)
"""

import _robot
from robot.testparser import *


tests = '''
echo off
from robot.puma560 import *
echo on

p560

# at nominal pose
arg2array(qn)
t = fkine(p560, qn)
q = ikine560(p560, t)
fkine(p560, q)
ikine(p560, t, q0=[0, 0.7, 3, 0, 0.7, 0], m=[1, 1, 1, 1, 1, 1], verbose=2)
'''

if __name__ == "__main__" :

    testparser(tests)
    

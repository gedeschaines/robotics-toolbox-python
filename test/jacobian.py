""" Robotics Toolbox for Python -- Test Jacobian primitives
"""

import _robot
from robot.testparser import * 


tests = '''
set_printoptions(precision=5, suppress=True);

echo off
from robot.puma560 import *
echo on

p560

jacob0(p560, qz);
jacob0(p560, qr)
jacob0(p560, qn)

jacobn(p560, qz)
jacobn(p560, qr)
jacobn(p560, qn)

t = fkine(p560, qn)
tr2jac(t)
'''

if __name__ == "__main__" :

    testparser(tests)

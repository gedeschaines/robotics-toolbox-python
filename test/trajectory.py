""" Robotics Toolbox for Python -- Test trajectory transformations
"""

import _robot
from robot.testparser import *


tests = '''
echo off
from robot.puma560 import *
echo on

(q,qd,qdd) = jtraj(qz, qr, 20)
q
qd
qdd

(q,qd,qdd) = jtraj(qz, qr, 20, 0.1*mat(ones((1,6))), -0.1*mat(ones((1,6))) )
q
qd
qdd

(q,qd,qdd) = jtraj(qz, qr, arange(0, 10.2, 0.2))
q

t1 = trotx(0.1) * transl(0.2, 0.3, 0.4)
t1
t2 = troty(-0.3) * transl(-0.2, -0.3, 0.6)
t2
ctraj(t1, t2, 5)
ctraj(t1, t2, arange(0, 1.01, 0.1))
'''

if __name__ == "__main__" :

    testparser(tests)


""" Robotics Toolbox for Python -- Test kinematics primitives
"""

import _robot
from robot.testparser import * 


tests = '''
echo off
from robot.puma560 import *
echo on

p560

# at zero pose
t = fkine(p560, qz)
t
q = ikine560(p560, t)
q
fkine(p560, q)
ikine560(p560, t, 'r')
ikine560(p560, t, 'rn')

ikine(p560, t, piter=False, debug=0, verbose=0)

# at nominal pose
arg2array(qn)
t = fkine(p560, qn)
t
q = ikine560(p560, t)
q
fkine(p560, q)
ikine(p560, t, q0=[0, 0.7, 3, 0, 0.7, 0])

# along trajectory
(q,qd,qdd) = jtraj(qz, qr, 20)
fkine(p560, q)

t1 = fkine(p560, qz)
t2 = fkine(p560, qr)
traj = ctraj(t1, t2, 5)
ikine(p560, traj, alpha=0.5, ilimit=1000)
'''

if __name__ == "__main__" :

    testparser(tests)


""" Robotics Toolbox for Python -- Test dynamics primitives
"""

import _robot
from robot.testparser import *


tests = '''
q1 = mat(ones((1,6)))

echo off

## test the RNE primitive for DH using standard Puma560

from robot.puma560 import *
p560
p560.ismdh()

echo on

# -- use qz with three state parameters
qz
rne(p560, qz, qz, qz)
rne(p560, qz, qz, qz, gravity=[1,2,3])
rne(p560, qz, qz, qz, gravity=[0,0,9.81], fext=[1,2,3,0,0,0])
rne(p560, qz, 0.2*q1, qz, gravity=[0,0,9.81], fext=[0,0,0,0,0,0])
rne(p560, qz, 0.2*q1, 0.2*q1)

# -- use qr with three state parameters
qr
rne(p560, qr, qz, qz)
rne(p560, qr, qz, qz, gravity=[1,2,3])

# -- use qr with single state parameter
z = hstack((qr, qz, qz))
rne(p560, z, gravity=[1,2,3])
rne(p560, z, fext=[1,2,3,0,0,0])
rne(p560, vstack((z,z,z)))

echo off

## test the RNE primitive for MDH using modified Puma560

from robot.puma560akb import *
p560m
p560.ismdh()

echo on

# -- use qz with three state parameters
arg2array(qz)
rne(p560m, qz, qz, qz)
rne(p560m, qz, qz, qz, gravity=[1,2,3])
rne(p560m, qz, qz, qz, fext=[1,2,3,0,0,0])
rne(p560m, qz, 0.2*q1, qz)
rne(p560m, qz, 0.2*q1, 0.2*q1)

# -- use qr with three state parameters
arg2array(qr)
rne(p560m, qr, qz, qz)
rne(p560m, qr, qz, qz, gravity=[1,2,3])

# -- use qr with single state parameter
z = hstack((qr, qz, qz))
rne(p560m, z, gravity=[1,2,3])
rne(p560m, z, fext=[1,2,3,0,0,0])
rne(p560m, vstack((z,z,z)))

# at zero pose

gravload(p560, qz)
gravload(p560, qz, gravity=[9,0,0])
gravload(p560, vstack((qz,qz,qz)))

inertia(p560, qz)
inertia(p560, vstack((qz,qr)))

accel(p560, qz, 0.2*q1, 0.5*q1)

# need to pick a non-singular configuration for cinertia()
cinertia(p560, qn)

coriolis(p560, qn, 0.5*q1)

# along trajectory
[q, qd, qdd] = jtraj(qz, qr, 20)
rne(p560, q, qd, qdd)
'''

if __name__ == "__main__" :

    testparser(tests)


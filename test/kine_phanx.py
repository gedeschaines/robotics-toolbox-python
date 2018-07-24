""" Robotics Toolbox for Python -- Kinematics tests for PhantomX

    Refs:

    [1] Peter Corke, "Using the Robotics Toolbox with a real robot", March 2013,
        Peter Corke Robotics Toolbox Tutorials, web available at
        https://petercorke.com/wordpress/interfacing-a-hobby-robot-arm-to-matlab
"""

import _robot
from robot.testparser import *


tests = '''
echo off
from robot.phantomx import *
echo on

## **** Kinematics Test ****

px

T0 = fkine(px, qz)
q0 = ikine(px, T0, q0=qz, m=[1,1,1,0,0,1], alpha=0.9, ilimit=1000)
fkine(px, q0)

T1 = fkine(px, qn)
q1 = ikine(px, T1, q0=qz, m=[1,1,1,0,0,1], alpha=0.9, ilimit=1000)
fkine(px, q1)
'''

if __name__ == "__main__" :

    testparser(tests)
    

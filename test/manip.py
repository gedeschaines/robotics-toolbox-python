""" Robotics Toolbox for Python -- Test manipulability
"""

import _robot
from robot.testparser import *


tests = '''
echo off
from robot.puma560 import *
echo on

p560

q = qn
manipblty(p560, q)
manipblty(p560, q, 'yoshi')
manipblty(p560, q, 'y')
manipblty(p560, q, 'asada')
manipblty(p560, q, 'a')
manipblty(p560, q, 'z')

qq = vstack((q, q, q, q))
qq
manipblty(p560, qq, 'yoshi')
manipblty(p560, qq, 'asada')
'''

if __name__ == "__main__" :

    testparser(tests)


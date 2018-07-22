""" Robotics Toolbox for Python -- Test rblot
"""

import _robot
from robot.testparser import *


tests = '''
set_printoptions(precision=4, suppress=True);

from robot.puma560 import *

l.display()
l.offset = 9
l.m = 10
l.G = 11
l.Jm = 12
l.B = 14

l.r = [1,2,3]
l.r
l.r = mat([1,2,3])
l.r
l.I = [1,2,3]
l.I
l.I = [1,2,3,4,5,6]
l.I
l.I = mat(diag([1,2,3]))
l.I

l.Tc = 1
l.Tc
l.Tc = [-1,2]
l.Tc

l.qlim = array([4,5])

l.display()
l2 = l.nofriction()
l.display()
l2.display()

l.friction(2)
l.friction(-2)

l.tr(0)
l.tr(0.2)

p560
p560.n
p560.mdh
p560.links
p560.base
p560.tool
p560.config()
p560.ismdh()

pbig=p560*p560
pbig
p560

p560.showlinks()

p560nf = p560.nofriction()
p560nf.showlinks()

p560.base = transl(1,2,3);
p560.base

p560.tool = transl(4,5,6);
p560.tool

p560.gravity = [1,2,3]
p560.gravity

'''

if __name__ == "__main__" :

    testparser(tests)


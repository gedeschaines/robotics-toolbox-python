# DESCRIPTION:
# A test for the kinematics and trajectories codes of the robotics toolbox

# import section
import _robot
from numpy import set_printoptions

set_printoptions(precision=5, suppress=True);

# import robot models section
from robot.puma560 import *
from robot.puma560akb import *
from robot.stanford import *

# import kinematics section
from robot.kinematics import *
from robot.jacobian import *

# import trajectories section
from robot.trajectory import *

# import manipulability section
from robot.manipblty import *

print("\nThis is a test for the kinematics and trajectories codes of the Robotics Toolbox.")
print("Codes are tested with puma560, puma560akb and stanford models in order to proof both")
print("standard and modified conventions for DH parameters, prismatic and rotational joints.\n")

print("\n\t\t\tRobot Models:\n")
print("%s\n" % p560)
print("%s\n" % p560m)
print("%s\n" % stanf)

print("\t\t\t***Kinematics test***\n")

print("T0 = fkine(p560,[0,0,0,0,0,0])\n")
T0 = fkine(p560,[0,0,0,0,0,0])
print("%s\n" % T0)
print("q0 = ikine(p560,T0,alpha=0.4,ilimit=2000)\n")
try:
    q0 = ikine(p560,T0,alpha=0.4,ilimit=2000)
    print("%s\n" % q0)
    print("fkine(p560,q0)\n")
    print("%s\n\n" % fkine(p560,q0))
except ValueError:
    print("\n\n")
    
print("T1 = fkine(p560m,[pi/2,-pi/4,3*pi/4,-pi/8,0,-pi/5])\n")
T1 = fkine(p560m,[pi/2,-pi/4,3*pi/4,-pi/8,0,-pi/5])
print("%s\n" % T1)
print("q1 = ikine(p560m,T1,alpha=0.4,ilimit=3000)\n")
try:
    q1 = ikine(p560m,T1,alpha=0.4,ilimit=3000)
    print("%s\n" % q1)
    print("fkine(p560m,q1)\n")
    print("%s\n\n" % fkine(p560m,q1))
except ValueError:
    print("\n\n")
    
print("T3 = fkine(stanf,[1,2,0.25,3,2,1])\n")
T3 = fkine(stanf,[1,2,0.25,3,2,1])
print("%s\n" % T3)
print("q3 = ikine(stanf,T3,alpha=0.4,ilimit=3000)\n")
try:
    q3 = ikine(stanf,T3,alpha=0.4,ilimit=3000)
    print("%s\n" % q3)
    print("fkine(stanf,q3)\n")
    print("%s\n\n" % fkine(stanf,q3))
except ValueError:
    print("\n\n")

print("q4 = ikine(p560m,fkine(p560m,[1,2,3,1,3,2]),alpha=0.4,ilimit=3000)\n")
try:
    q4 = ikine(p560m,fkine(p560m,[1,2,3,1,3,2]),alpha=0.4,ilimit=3000)
    print("%s\n\n" % q4)
    print("fkine(p560m,q4)\n")
    print("%s\n\n" % fkine(p560m,q4))
except ValueError:
    print("\n\n")

print("q5 = ikine(stanf,fkine(stanf,[1,2,1,0,2,3]),alpha=0.4,ilimit=3000)\n")
try:
    q5 = ikine(stanf,fkine(stanf,[1,2,1,0,2,3]),alpha=0.4,ilimit=3000)
    print("%s\n\n" % q5)
    print("fkine(stanf,q5)\n")
    print("%s\n\n" % fkine(stanf,q5))
except ValueError:
    print("\n\n")

print("fkine(stanf,[1,2,1,0,2,3])\n")
print("%s\n\n" % fkine(stanf,[1,2,1,0,2,3]))

print("\t\t\t***Test for Jacobian***\n")

print("J1 = jacobn(stanf,[1,2,1,0,2,3])\n")
J1 = jacobn(stanf,[1,2,1,0,2,3])
print("%s\n\n" % J1)

print("J2 = jacobn(p560m,[1,0,3,-3,0,1])\n")
J2 = jacobn(p560m,[1,0,3,-3,0,1])
print("%s\n\n" % J2)

print("J3 = jacobn(p560,[1,0,3,-3,0,1])\n")
J3 = jacobn(p560,[1,0,3,-3,0,1])
print("%s\n\n" % J3)

print("J01 = jacob0(stanf,[1,2,1,0,2,3])\n")
J01 = jacob0(stanf,[1,2,1,0,2,3])
print("%s\n\n" % J01)

print("J02 = jacob0(p560m,[1,0,3,-3,0,1])\n")
J02 = jacob0(p560m,[1,0,3,-3,0,1])
print("%s\n\n" % J02)

print("J03 = jacob0(p560,[1,0,3,-3,0,1])\n")
J03 = jacob0(p560,[1,0,3,-3,0,1])
print("%s\n\n" % J03)

print("\t\t\t***Trajectory Test***\n")

print("Joint trajectory")
print("qj,qdj,qddj = jtraj([0,0,0,0,0,0],[pi/2,pi/4,-3*pi/5,4*pi/6,0,1], 5)\n")
qj,qdj,qddj = jtraj([0,0,0,0,0,0],[pi/2,pi/4,-3*pi/5,4*pi/6,0,1], 5)
print("qj:\n%s\n\nqdj:\n%s\n\nqddj:\n%s\n\n" % (qj,qdj,qddj))

print("Cartesian trajectory")
print("tt = ctraj(fkine(p560m,[0,0,0,0,0,0]), fkine(p560m,[pi/2,pi/4,-3*pi/4,-pi/8,0,1]), 5)\n")
tt = ctraj(fkine(p560m,[0,0,0,0,0,0]), fkine(p560m,[pi/2,pi/4,-3*pi/4,-pi/8,0,1]), 5)
for i in tt:
    print("%s\n\n" % i)
    
print("\t\t\t***Trajectory Case for Kinematics Test***\n")

print("qj:\n%s\n\n" % qj)
print("Tt1 = fkine(p560m,qj)\n")
Tt1 = fkine(p560m,qj)
for i in Tt1:
    print("%s\n\n" % i)
print("Qt1 = ikine(p560m,Tt1,alpha=0.3,ilimit=3000)\n")
try:
    Qt1 = ikine(p560m,Tt1,alpha=0.3,ilimit=3000)
    for i in Qt1:
        print("%s\n\n" % i)
except ValueError:
    print("\n\n")
        
print("Qt2 = ikine(stanf, fkine(stanf,[[0,0,0,0,0,0],[1,0,1,0,1,0],[1,0,0.8,0,1,0]]), alpha=0.4, ilimit=3000)\n")
try:
    Qt2 = ikine(stanf, fkine(stanf,[[0,0,0,0,0,0],[1,0,1,0,1,0],[1,0,0.8,0,1,0]]), alpha=0.4, ilimit=3000)
    for i in Qt2:
        print("%s\n\n" % i)
except ValueError:
    print("\n\n")
  
print("\t\t\tManipulability test:\n")

print("\nmanip1 = manipblty(p560,[1,2,3,4,5,6],\'y\')\n")
manip1 = manipblty(p560,[1,2,3,4,5,6],'y')
print("%s\n\n" % manip1)
print("\nmanip1 = manipblty(p560,[1,2,3,4,5,6],\'a\')\n")
manip1 = manipblty(p560,[1,2,3,4,5,6],'a')
print("%s\n\n" % manip1)

print("\nmanip2 = manipblty(p560m,[1,2,3,4,5,6],\'y\')\n")
manip2 = manipblty(p560m,[1,2,3,4,5,6],'y')
print("%s\n\n" % manip2)
print("\nmanip2 = manipblty(p560m,[1,2,3,4,5,6],\'a\')\n")
manip2 = manipblty(p560m,[1,2,3,4,5,6],'a')
print("%s\n\n" % manip2)

print("\nmanip3 = manipblty(stanf,[pi/3,pi/6,0.3,-3*pi/4,-2*pi/5,pi/6],\'y\')\n")
manip3 = manipblty(stanf,[pi/3,pi/6,0.3,-3*pi/4,-2*pi/5,pi/6],'y')
print("%s\n\n" % manip3)
print("\nmanip3 = manipblty(stanf,[pi/3,pi/6,0.3,-3*pi/4,-2*pi/5,pi/6],\'a\')\n")
manip3 = manipblty(stanf,[pi/3,pi/6,0.3,-3*pi/4,-2*pi/5,pi/6],'a')
print("%s\n\n" % manip3)

print("\nTrajectory case test for manipulability\n")
print("\nmanip1 = manipblty(p560,[[1,2,3,4,5,6],[0,-5,4,0,2,1],[pi/4,-1,0.5,1,-pi/6,1],\'a\')\n")
manip1 = manipblty(p560,[[1,2,3,4,5,6],[0,-5,4,0,2,1],[pi/4,-1,0.5,1,-pi/6,1]],'a')
for i in manip1.T:
    print("%s\n\n" % i)


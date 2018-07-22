'''
PUMA560AKB Load kinematic and dynamic data for a Puma 560 manipulator

    from robot.puma560akb import *

Defines the object 'p560m' in current workspace which describes the 
kinematic and dynamic characterstics of a Unimation Puma 560 manipulator 
modified DH conventions and using the data and conventions of:

    Armstrong, Khatib and Burdick 1986.
    "The Explicit Dynamic Model and Inertial Parameters of the Puma 560 Arm"

Also define the vector qz which corresponds to the zero joint
angle configuration, qr which is the vertical 'READY' configuration,
and qstretch in which the arm is stretched out in the X direction.

@see: L{Robot}, L{puma560}, L{stanford}, L{phantomx}, L{fourlink}, L{twolink}

Python implementation by: Luis Fernando Lara Tobar and Peter Corke.
Based on original Robotics Toolbox for Matlab code by Peter Corke.
Permission to use and copy is granted provided that acknowledgement
of the authors is made.

@author: Luis Fernando Lara Tobar and Peter Corke
'''

from numpy import *
from robot.Link import *
from robot.Robot import *

print("Creating a Puma 560 (mod) as p560m")

L = []

# link DH parameters
L.append(Link(alpha=0,     A=0,       theta=0, D=0,       sigma=0, convention=2))
L.append(Link(alpha=-pi/2, A=0,       theta=0, D=0.2435,  sigma=0, convention=2))
L.append(Link(alpha=0,     A=0.4318,  theta=0, D=-0.0934, sigma=0, convention=2))
L.append(Link(alpha=pi/2,  A=-0.0203, theta=0, D=.4331,   sigma=0, convention=2))
L.append(Link(alpha=-pi/2, A=0,       theta=0, D=0,       sigma=0, convention=2))
L.append(Link(alpha=pi/2,  A=0,       theta=0, D=0,       sigma=0, convention=2))

# link mass
L[0].m = 0
L[1].m = 17.4
L[2].m = 4.8
L[3].m = 0.82
L[4].m = 0.34
L[5].m = .09

# link COG wrt link coordinate frame
#              rx       ry        rz
L[0].r = mat([ 0.0,     0.0,      0.0  ])
L[1].r = mat([ 0.068,   0.006,   -0.016])
L[2].r = mat([ 0.0,    -0.070,    0.014])
L[3].r = mat([ 0.0,     0.0,     -0.019])
L[4].r = mat([ 0.0,     0.0,      0.0  ])
L[5].r = mat([ 0.0,     0.0,      0.032])

# link inertia matrix about link COG
#              Ixx       Iyy       Izz     Ixy  Iyz  Ixz
L[0].I = mat([ 0.0,      0.0,      0.35,     0,   0,   0])
L[1].I = mat([ 0.13,     0.524,    0.539,    0,   0,   0])
L[2].I = mat([ 0.066,    0.0125,   0.066,    0,   0,   0])
L[3].I = mat([ 1.8e-3,   1.8e-3,   1.3e-3,   0,   0,   0])
L[4].I = mat([ 0.3e-3,   0.3e-3,   0.4e-3,   0,   0,   0])
L[5].I = mat([ 0.15e-3,  0.15e-3,  0.04e-3,  0,   0,   0])

# actuator motor inertia (motor referred)
L[0].Jm =  291e-6
L[1].Jm =  409e-6
L[2].Jm =  299e-6
L[3].Jm =  35e-6
L[4].Jm =  35e-6
L[5].Jm =  35e-6

# actuator gear ratio
L[0].G =  -62.6111
L[1].G =  107.815
L[2].G =  -53.7063
L[3].G =   76.0364
L[4].G =   71.923
L[5].G =   76.686

# viscous friction (motor referenced)
# unknown

# Coulomb friction (motor referenced)
# unknown

#
# some useful poses
#
qz =       [0.0,  0.0,  0.0,  0.0, 0.0, 0.0]  # zero angles, L shaped pose
qr =       [0.0, -pi/2, pi/2, 0.0, 0.0, 0.0]  # ready pose, arm up
qstretch = [0.0,  0,    pi/2, 0.0, 0.0, 0.0]  # horizontal along x-axis
# prevent name collision with puma560
qz_m = qz
qr_m = qr
qs_m = qstretch

p560m = Robot(L, name='Puma560-AKB', manuf='Unimation', comment='AK&B')
p560m.set_handle('p3D', 1)
p560m.set_handle('mag', 0.25)

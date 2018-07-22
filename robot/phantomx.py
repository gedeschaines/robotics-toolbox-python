"""
PHANTOMX Load kinematic and dynamic data for a PhantomX  manipulator

    from robot.phantomx import *

Defines the object 'px' in the current workspace which describes the
kinematic characteristics of a PhantomX Pincher Robot, a 4 joint hobby
class manipulator by Trossen Robotics.

Also defines the vector qz = [0, 0, 0, 0] which corresponds to the zero
joint angle configuration.

Notes::
 - the x-axis is forward, and the z-axis is upwards.
 - uses standard DH conventions.
 - Tool centrepoint is middle of the fingertips.
 - all translational units in m.

Reference::

 - http://www.trossenrobotics.com/productdocs/assemblyguides/phantomx-basic-robot-arm.html

@see: L{Robot}, L{puma560}, L{puma560akb}, L{stanford}, L{fourlink}, L{twolink}.

Python implementation by: Gary Deschaines.
Based on original Robotics Toolbox for Matlab code by Peter Corke.
Permission to use and copy is granted provided that acknowledgement
of the author is made.

@author: Peter I. Corke
"""

from numpy import *
from robot.Robot import *
from robot.Link import *

print("Creating a PhantomX robot as px")

L = []

# link DH parameters
L.append( Link(alpha=-pi/2,  A= 0.0,    D=0.04, sigma=0))
L.append( Link(alpha=pi,     A=-0.105,  D=0.0,  sigma=0))
L.append( Link(alpha=0.0,    A=-0.105,  D=0.0,  sigma=0))
L.append( Link(alpha=0.0,    A=-0.105,  D=0.0,  sigma=0))

L[1].offset = pi/2

# link mass
L[0].m = 1.0
L[1].m = 1.0
L[2].m = 1.0
L[3].m = 1.0

# link COG wrt link coordinate frame
#               rx       ry       rz
L[0].r = mat([  0.0,     0.0,     0.04])
L[1].r = mat([ -0.105,   0.0,     0.0 ])
L[2].r = mat([ -0.105,   0.0,     0.0 ])
L[3].r = mat([ -0.105,   0.0,     0.0 ])

# link inertia matrix about link COG
#              Ixx       Iyy       Izz     Ixy  Iyz  Ixz
L[0].I = mat([ 0.0,      0.0,      0.0,      0,   0,   0])
L[1].I = mat([ 0.0,      0.0,      0.0,      0,   0,   0])
L[2].I = mat([ 0.0,      0.0,      0.0,      0,   0,   0])
L[3].I = mat([ 0.0,      0.0,      0.0,      0,   0,   0])

# actuator motor inertia (motor referred)
L[0].Jm = 0.0
L[1].Jm = 0.0
L[2].Jm = 0.0
L[3].Jm = 0.0

# actuator gear ratio
L[0].G = 1.0
L[1].G = 1.0
L[2].G = 1.0
L[3].G = 1.0

# viscous friction (motor referenced)
# unknown

# Coulomb friction (motor referenced)
# unknown
#

# some useful poses
#
qz = [0.0, 0.0, 0.0, 0.0]  # zero angles, straight along x-axis pose
# prevent name collision with other robots
qz_px = qz

# create robot with serial-link manipulator
px = Robot(L, name='PhantomX', manuf='Trossen Robotics')
px.tool = trotz(-pi/2) * trotx(pi/2)
px.set_handle('p3D', 1)
px.set_handle('mag', 0.05)
px.set_plotopt('xlim', [-0.5, 0.5])
px.set_plotopt('ylim', [-0.5, 0.5])
px.set_plotopt('zlim', [-0.5, 0.5])

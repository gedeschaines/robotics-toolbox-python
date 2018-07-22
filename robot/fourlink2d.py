"""
FOURLINK2D Load kinematic and dynamic data for a simple planar 4-link
mechanism

	from robot.fourlink2d import *

Defines the object 'fl2d' in the current workspace which describes the
kinematic and dynamic characterstics of a simple planar 4-link mechanism.

Example based on IK_Solver 2D 4-link chain

Assume links with all mass (unity) concentrated at the joints.

Also defines the vectors qz, qn and qt which correspond to the zero joint
angle, a nominal joint angle and a targeted joint angle configuration
respectively.

@see: L{Robot}, L{puma560}, L{puma560akb}, L{stanford}, L{phantomx}, {twolink}.

Python implementation by: Gary Deschaines.
Based on original Robotics Toolbox for Matlab code by Peter Corke.
Permission to use and copy is granted provided that acknowledgement
of the author is made.

@author: Gary Deschaines
"""

from numpy import *
from robot.Robot import *
from robot.Link import *

print("Creating a planar four-link arm as fl2d")

L = []

# link DH parameters
L.append( Link(alpha=0.0,   A=2.0,   D=0.0,  sigma=0) )
L.append( Link(alpha=0.0,   A=2.0,   D=0.0,  sigma=0) )
L.append( Link(alpha=0.0,   A=1.0,   D=0.0,  sigma=0) )
L.append( Link(alpha=0.0,   A=1.0,   D=0.0,  sigma=0) )

# link mass
L[0].m = 1.0
L[1].m = 1.0
L[2].m = 1.0
L[3].m = 1.0

# link COG wrt link coordinate frame
#               rx       ry       rz
L[0].r = mat([  2.0,     0.0,     0.0])
L[1].r = mat([  2.0,     0.0,     0.0])
L[2].r = mat([  1.0,     0.0,     0.0])
L[3].r = mat([  1.0,     0.0,     0.0])

# link inertia matrix about link COG
#              Ixx       Iyy       Izz     Ixy  Iyz  Ixz
L[0].I = mat([ 0.0,      0.0,      4.0,      0,   0,   0])
L[1].I = mat([ 0.0,      0.0,      2.0,      0,   0,   0])
L[2].I = mat([ 0.0,      0.0,      0.5,      0,   0,   0])
L[3].I = mat([ 0.0,      0.0,      0.5,      0,   0,   0])

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

#
# some useful poses
#
qz = [0.0, 0.0, 0.0, 0.0]  # zero angles, straight along x-axis pose
qn = [pi/7.2, pi/12.0, pi/18.0, pi/36.0]   # nominal pose
qt = [0.53938, 0.98726, 0.85419, 0.48104]  # targeted pose
# prevent name collision with other robots
qz_fl2d = qz
qn_fl2d = qn
qt_fl2d = qt

# create robot with serial-link manipulator
fl2d = Robot(L, name='Simple planar four link')
fl2d.set_handle('p3D', 0)
fl2d.set_handle('mag', 0.5)
fl2d.set_plotopt('xlim', [-6.0, 6.0])
fl2d.set_plotopt('ylim', [-6.0, 6.0])


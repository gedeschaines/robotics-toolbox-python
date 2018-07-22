"""
TWOLINK Load kinematic and dynamic data for a simple 2-link mechanism

	from robot.twolink import *

Defines the object 'tl' in the current workspace which describes the 
kinematic and dynamic characterstics of a simple planar 2-link mechanism.

Example based on Fig 3-6 (p73) of Spong and Vidyasagar (1st edition).  
It is a planar mechanism operating in the XY (horizontal) plane and is 
therefore not affected by gravity.

Assume unit length links with all mass (unity) concentrated at the joints.

Also defines the vector qz = [0, 0] which corresponds to the zero joint
angle configuration.

@see: L{Robot}, L{puma560}, L{puma560akb}, L{stanford}, L{phantomx}, {fourlink}.

Python implementation by: Luis Fernando Lara Tobar and Peter Corke.
Based on original Robotics Toolbox for Matlab code by Peter Corke.
Permission to use and copy is granted provided that acknowledgement
of the authors is made.

@author: Luis Fernando Lara Tobar and Peter Corke
"""

from numpy import *
from robot.Robot import *
from robot.Link import *

print("Creating a two-link planar arm as tl")

L = []

# link DH parameters
L.append( Link(alpha=0.0,  A=1.0,   D=0.0,  sigma=0) )
L.append( Link(alpha=0.0,  A=1.0,   D=0.0,  sigma=0) )

# link mass
L[0].m = 1.0
L[1].m = 1.0

# link COG wrt link coordinate frame
#               rx       ry       rz
L[0].r = mat([  1.0,     0.0,     0.0])
L[1].r = mat([  1.0,     0.0,     0.0])

# link inertia matrix about link COG
#              Ixx       Iyy       Izz     Ixy  Iyz  Ixz
L[0].I = mat([ 0.0,      0.0,      0.0,      0,   0,   0])
L[1].I = mat([ 0.0,      0.0,      0.0,      0,   0,   0])

# actuator motor inertia (motor referred)
L[0].Jm = 0.0
L[1].Jm = 0.0

# actuator gear ratio
L[0].G = 1.0
L[1].G = 1.0

# viscous friction (motor referenced)
# unknown

# Coulomb friction (motor referenced)
# unknown
#

# some useful poses
#
qz = [0.0, 0.0]  # zero angles, straight along x-axis pose
# prevent name collision with other robots
qz_tl = qz

# create robot with serial-link manipulator
tl = Robot(L, name='Simple two link')
tl.set_handle('p3D', 0)
tl.set_handle('mag', 0.5)
tl.set_plotopt('xlim', [-3.0, 3.0])
tl.set_plotopt('ylim', [-3.0, 3.0])

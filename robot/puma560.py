'''
PUMA560 Load kinematic and dynamic data for a Puma 560 manipulator

    from robot.puma560 import *

Defines the object 'p560' in the current workspace which describes the
kinematic and dynamic % characterstics of a Unimation Puma 560 manipulator
using standard DH conventions.

The model includes armature inertia and gear ratios.

Also define the vector qz which corresponds to the zero joint
angle configuration, qr which is the vertical 'READY' configuration,
and qstretch in which the arm is stretched out in the X direction.

@see: L{Robot}, L{puma560akb}, L{stanford}, L{phantomx}, L{fourlink}, L{twolink}

@notes:
the value of m1 is given as 0 here.  Armstrong found no value for it
and it does not appear in the equation for tau1 after the substituion
is made to inertia about link frame rather than COG frame.
updated:

Python implementation by: Luis Fernando Lara Tobar and Peter Corke.
Based on original Robotics Toolbox for Matlab code by Peter Corke.
Permission to use and copy is granted provided that acknowledgement
of the authors is made.

@author: Luis Fernando Lara Tobar and Peter Corke
'''

from numpy import *
from robot.Link import *
from robot.Robot import *

print("Creating a Puma 560 (std) as p560")

L = []

# link DH parameters
L.append( Link(alpha=pi/2,  A=0,      D=0,       sigma=0) )
L.append( Link(alpha=0,     A=0.4318, D=0,       sigma=0) )
L.append( Link(alpha=-pi/2, A=0.0203, D=0.15005, sigma=0) )
L.append( Link(alpha=pi/2,  A=0,      D=0.4318,  sigma=0) )
L.append( Link(alpha=-pi/2, A=0,      D=0,       sigma=0) )
L.append( Link(alpha=0,     A=0,      D=0,       sigma=0) )

# link mass
L[0].m = 0
L[1].m = 17.4
L[2].m = 4.8
L[3].m = 0.82
L[4].m = 0.34
L[5].m = .09

# link COG wrt link coordinate frame
#               rx       ry       rz
L[0].r = mat([  0.0,     0.0,     0.0   ])
L[1].r = mat([ -0.3638,  0.006,   0.2275])
L[2].r = mat([ -0.0203, -0.0141,  0.070 ])
L[3].r = mat([  0.0,     0.019,   0.0   ])
L[4].r = mat([  0.0,     0.0,     0.0   ])
L[5].r = mat([  0.0,     0.0,     0.032 ])

# link inertia matrix about link COG
#              Ixx       Iyy       Izz     Ixy  Iyz  Ixz
L[0].I = mat([ 0.0,      0.35,     0.0,      0,   0,   0])
L[1].I = mat([ 0.13,     0.524,    0.539,    0,   0,   0])
L[2].I = mat([ 0.066,    0.086,    0.0125,   0,   0,   0])
L[3].I = mat([ 1.8e-3,   1.3e-3,   1.8e-3,   0,   0,   0])
L[4].I = mat([ 0.3e-3,   0.4e-3,   0.3e-3,   0,   0,   0])
L[5].I = mat([ 0.15e-3,  0.15e-3,  0.04e-3,  0,   0,   0])

# actuator motor inertia (motor referred)
L[0].Jm =  200e-6
L[1].Jm =  200e-6
L[2].Jm =  200e-6
L[3].Jm =  33e-6
L[4].Jm =  33e-6
L[5].Jm =  33e-6

# actuator gear ratio
L[0].G =  -62.6111
L[1].G =  107.815
L[2].G =  -53.7063
L[3].G =   76.0364
L[4].G =   71.923
L[5].G =   76.686

# viscous friction (motor referenced)
L[0].B =   1.48e-3
L[1].B =   0.817e-3
L[2].B =   1.38e-3
L[3].B =  71.2e-6
L[4].B =  82.6e-6
L[5].B =  36.7e-6

# Coulomb friction (motor referenced)
L[0].Tc = mat([ .395, -.435])
L[1].Tc = mat([ .126, -.071])
L[2].Tc = mat([ .132, -.105])
L[3].Tc = mat([ 11.2e-3, -16.9e-3])
L[4].Tc = mat([ 9.26e-3, -14.5e-3])
L[5].Tc = mat([ 3.96e-3, -10.5e-3])

# joint variable limits [min max]

def qlim_mat_rad(lims_deg):
  qlim = mat([lims_deg[0], lims_deg[1]])*pi/180
  return qlim

L[0].qlim = qlim_mat_rad([-160, 160])
L[1].qlim = qlim_mat_rad([ -45, 225])
L[2].qlim = qlim_mat_rad([-225,  45])
L[3].qlim = qlim_mat_rad([-110, 170])
L[4].qlim = qlim_mat_rad([-100, 100])
L[5].qlim = qlim_mat_rad([-266, 266])

#
# some useful poses
#
qz = [0.0, 0.0,   0.0,  0.0, 0.0,  0.0]  # zero angles, L shaped pose
qr = [0.0, pi/2, -pi/2, 0.0, 0.0,  0.0]  # ready pose, arm up
qs = [0.0, 0.0,  -pi/2, 0.0, 0.0,  0.0]  # stretch horizontal along x-axis
qn = [0.0, pi/4,  pi,   0.0, pi/4, 0.0]  # nominal non-singular configuration
# prevent name collision with puma560akb
qz_s = qz
qr_s = qr
qs_s = qs
qn_s = qn

# create robot with serial-link manipulator
p560 = Robot(L, name='Puma 560', manuf='Unimation', comment='params of 8/95')
p560.set_handle('p3D', 1)
p560.set_handle('mag', 0.25)

# Copyright (C) 1993-2014, by Peter I. Corke
#
# This file is part of The Robotics Toolbox for MATLAB (RTB).
#
# RTB is free software: you can redistribute it and/or modify it under
# the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# RTB is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with RTB.  If not, see <http://www.gnu.org/licenses/>.
#
# http://www.petercorke.com

import _robot
from robot import parsedemo as p
import sys

if __name__ == '__main__':

    s ='''
echo on
# A serial link manipulator comprises a series of links. Each link is
# described by four Denavit-Hartenberg (DH) parameters.
#
# Let's define a simple 2 link planar manipulator. The first link is
    L1 = Link(D=0, A=1, alpha=0)

# The Link object we created has a number of properties,
    L1.A  # distance along this link's x-axis to next joint
    L1.D  # distance along prior link's z-axis to this joint

# and we determine that it is a revolute joint.
    L1.isrevolute()
pause % press any key to continue

# For a given joint angle, say q=0.8 rad (~46 deg), we can determine
# the link transform matrix.
    L1.tr(0.8)
pause % press any key to continue

# The second link is
    L2 = Link(D=0, A=1, alpha=0)
pause % press any key to continue

# Now we need to join these into a serial-link robot manipulator.
    bot2 = Robot([L1, L2], name='my 2-link robot')
    bot2.showlinks()
    
# The displayed robot object shows a lot of details. It also has a number
# of properties such as the number of joints.
    bot2.n
pause % press any key to continue

# Given the joint angles q1=0.8 and q2=0.4 (~23 deg) we can determine
# the pose of the robot's end-effector,
    fkine(bot2, [0.8, 0.4])
# which is referred to as the forward kinematics of the robot. This, and
# the inverse kinematics are covered in separate demos.

pause % press any key to continue

# Finally we can draw a stick figure of our robot.

    bot2.set_handle('p3D', 0)              # this is a planar (2D) manipulator
    bot2.set_handle('mag', 0.5)            # notational drawing scale factor
    bot2.set_plotopt('xlim', [-3.0, 3.0])  # set plot axis limits to view arm
    bot2.set_plotopt('ylim', [-3.0, 3.0])  #  and end-effector reference frame
    rbplot(bot2, [0.8, 0.4])
pause % press any key to continue

# We are not limited to just two dimensional robotic manipulators. By just
# modifying the 'D', 'A' and 'alpha' DH parameters of the first and second
# links, and adding a third link, we can create an upright three dimensional
# 3-link robot.
    L1 = Link(D=0.5, A=0, alpha=pi/2);
    L2 = Link(D=0.0, A=1, alpha=0.0);
    L3 = Link(D=0.0, A=1, alpha=0.0);
    bot3 = Robot([L1, L2, L3], name='my 3-link robot');
    
##  bot3.set_handle('p3D', 1)              # default setting when robot created
    bot3.set_handle('mag', 0.5)            # notational drawing scale factor
    bot3.set_plotopt('xlim', [-3.0, 3.0])  # set plot axis limits to view arm
    bot3.set_plotopt('ylim', [-3.0, 3.0])  #  and end-effector reference frame
    bot3.set_plotopt('zlim', [-3.0, 3.0])  #  with no dimensional distortions
    rbplot(bot3, [0.2, 0.8, 0.4])  # plot shows red line from zmin to robot base
    
# Note that the first link revolves about the world space z-axis, but the
# remaining two links revolve about axes in the world xy plane. This is
# the result of adding 'twist' angle alpha of pi/2 rad (90 deg) to the
# first link.
pause % press any key to continue

# This completes the 'my-robot' demo.  Consult the demo README file for
# recommended viewing sequence of other RTB for Python demos.
echo off
'''

    p.parsedemo(s)

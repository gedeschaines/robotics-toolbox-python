# Copyright (C) 1993-2002, by Peter I. Corke

import _robot
from robot import parsedemo as p
import sys

if __name__ == '__main__':

    s ='''
echo on
##----------------------------------------------------------------##
##    Robotics Toolbox (RTB) for Python Animation Capabilities    ##
##----------------------------------------------------------------##
    
# First we will define a robot; in this case selecting the Puma 560.
    
    from robot.puma560 import *
    
# To display an n-dof robot manipulator arm in a specific pose, pass
# the robot object and a 1xn joint array to the rbplot() function as
# shown in the following example for the Puma 560 in its zero angle
# pose.

    rbplot(p560, qz);

# The drawn line segments do not necessarily correspond to robot links,
# but join the origins of sequential link coordinate frames.
#
# A small right-angle coordinate frame is drawn on the end of the robot
# to show the wrist orientation. The orthogonal x, y and z frames axes
# are denoted with red, green and blue line segments respectively. The
# base of the robot is shown as a thick red line segment.
#
pause % press any key to continue
    close()

# The trajectory demonstration has shown how a joint coordinate trajectory
# may be generated; in this case from the zero pose to the resting pose.
    t = arange(0.0,2.0,0.056);   % generate a time vector
    [Q,_,_] = jtraj(qz, qr, t);  % generate joint coordinates trajectory

# The function rbplot() animates a blue stick figure robot moving through
# a given joint trajectory.

    p560.set_handle('fig', None)  % force new plot figure window
    rbplot(p560, Q);

# A shadow appears on the ground which helps to give some better idea of
# the 3D object's orientation. A stationary green stick figure denotes
# the object's initial pose.
#
pause % press any key to continue

# We can also place additional robots into a figure. Let's make a clone
# of the Puma robot, but change its name and base location.
 
    p560_2 = p560.copy();
    p560_2.name = 'another Puma';
    p560_2.base = transl(-0.5, 0.5, 0);
    rbplot(p560_2, Q, hold=True);

pause % press any key to continue

# We can animate both robots simultanously.

    close()
    [Q_2,_,_] = jtraj(qn, qr, t);
    rbplot(p560, Q);
    rbplot(p560_2, Q_2);

pause % press any key to continue

# We can also have multiple views of the same robot.
#

    close()
    rbplot(p560, qr);
### view(40,50);  ### Not implemented
    rbplot(p560, qr);
    rbplot(p560, qz);

pause % press any key to continue

# Sometimes it's useful to be able to manually drive the robot around to
# get an understanding of how it works.

### Not implemented.
###
### drivebot(p560)  % Not implemented in RTB for Python as of July 2018.
###

# use the sliders to control the robot (in fact both views).  Hit the red
# quit button when you are done.

echo off
'''

    p.parsedemo(s)

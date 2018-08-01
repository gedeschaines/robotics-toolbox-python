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
# may be generated; in this case from the zero pose to the ready pose.
    t = arange(0.0,2.0,0.056);   % generate a time vector
    [Q,_,_] = jtraj(qz, qr, t);  % generate joint coordinates trajectory

# The function rbplot() animates a blue stick figure robot moving through
# a given joint trajectory.

    p560.set_handle('fig', None)  % force new plot figure window
    rbplot(p560, Q);

# A shadow appears on the ground which helps to give some better idea of
# the 3D robot's orientation. A stationary green stick figure denotes
# the robot's initial pose and dashed line connects the end-effector's
# initial position to the target position shown as a red 'x'. Target
# position in this case is the end-effector's expected position at the
# pose computed for the last entry in joint coordinates trajectory 'Q'.
#
pause % press any key to continue
    
# We can create figures for multiple robots and also place additional
# robots into a figure. Let's make a clone of the Puma robot, but change
# its name and base location.
 
    p560_2 = p560.copy();
    p560_2.name = 'another Puma';
    p560_2.base = transl(-0.5, 0.5, 0);
    rbplot(p560_2, qz, phold=True);

pause % press any key to continue
    close()
    
# We can animate both robots simultaneously on separate figures,

    [Q_2,_,_] = jtraj(qn, qr, t);  % create joint trajectory for p560_2
    rbplot(p560, Q);      % p560 motion from zero pose to ready pose
    rbplot(p560_2, Q_2);  % p560_2 motion from nominal pose to ready pose

pause % press any key to continue

# or animate both robots simultaneously on the same figure.

    close('all')
    rbplot(p560, Q);  % Note: this starts an animator to render p560
##                    %       and should be allowed to run to completion 
##                    %       to prevent flickering animation when the
##                    %       next call to rbplot starts an animator for
##                    %       p560_2 which also renders p560.
pause % press any key to continue
    rbplot(p560_2, Q_2, phold=True);  % Note: this will animate p560_2
##                                    %       and re-animate p560.
pause % press any key to continue
    close('all')
    
# We can also have multiple views of the same robot.

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

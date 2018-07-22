""" Robotics Toolbox for Python -- Test rbplot with 2D robot
"""

import _robot
from robot import parsedemo as p
import sys

if __name__ == '__main__':

    s = '''
from robot.fourlink2d import *

## The robot pose plots will be automatically displayed. Do not
## close the plots. Just press 'enter' key to continue testing.

# Create two copies of the four-link robot; move 1st upward and
# move 2nd downward.
fl2d_1 = fl2d.copy();
fl2d_1.name = "Four-link #1";
fl2d_1.base = transl(0.0, 1.0, 0.0);
fl2d_2 = fl2d.copy();
fl2d_2.name = "Four-link #2";
fl2d_2.base = transl(0.0, -5.0, 0.0);

# Create joint space trajectory 'Q', from nominal pose to target pose.
(Q,_,_) = jtraj(qn, qt, 51);

# Simultaneously display animation of each robot in motion through joint
# space trajectory 'Q' on two separate plots.
rbplot(fl2d_1, Q, title="test_rbplot2d: 1st Robot along Q");
rbplot(fl2d_2, Q, title="test_rbplot2d: 2nd Robot along Q");
pause
close('all');

# Utilize forward and inverse kinematic functions to create a joint
# space trajectory 'Qik', from nominal pose to target pose, for the
# 2nd robot. 
t0 = fkine(fl2d_2, qn);
t1 = fkine(fl2d_2, qt);
Traj = ctraj(t0, t1, 51);
Qik = ikine(fl2d_2, Traj, q0=qn, m=[1,1,1,1,0,0], verbose=0);

# Show 1st robot motion along 'Q' and then 2nd robot motion along 'Qik'
# on the same plot.
rbplot(fl2d_1, Q, title="Robot #1 along Q and Robot #2 along Qik");
pause
rbplot(fl2d_2, Qik, phold=True);
pause
'''
    p.parsedemo(s)

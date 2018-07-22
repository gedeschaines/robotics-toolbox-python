""" Robotics Toolbox for Python -- Test rbplot with 3D robot
"""

import _robot
from robot import parsedemo as p
import sys

if __name__ == '__main__':

    s = '''
from robot.puma560 import *

## The robot pose plots will be automatically displayed. Do not
## close the plots. Just press 'enter' key to continue testing.

# Create two copies of the puma560; move 2nd by [-0.5,0.5,0.0].
p560_1 = p560.copy();
p560_1.name = "Puma 560 #1";
p560_2 = p560.copy();
p560_2.name = "Puma 560 #2";
p560_2.base = transl(-0.5, 0.5, 0.0);

# Create joint space trajectory 'Q', from nominal pose to ready pose.
(Q,_,_) = jtraj(qn, qr, 51);

# Simultaneously display animation of each robot in motion through joint
# space trajectory 'Q' on two separate plots.
rbplot(p560_1, Q, title="Test rbplot3d: 1st Robot along Q");
rbplot(p560_2, Q, title="Test rbplot3d: 2nd Robot along Q");
pause
close('all');

# Utilize forward and inverse kinematic functions to create a joint
# space trajectory 'Qik', from nominal pose to ready pose, for the
# 2nd robot.
t0 = fkine(p560_2, qn);
t1 = fkine(p560_2, qr);
Traj = ctraj(t0, t1, 51);
Qik = ikine(p560_2, Traj, qn, [1,1,1,1,1,1], verbose=0);

# Show 1st robot motion along 'Q' and then 2nd robot motion along 'Qik'
# on the same plot.
rbplot(p560_1, Q, title="Robot #1 along Q and Robot #2 along Qik");
pause
rbplot(p560_2, Qik, phold=True);
pause
'''
    p.parsedemo(s)

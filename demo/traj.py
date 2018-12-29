# Copyright (C) 1993-2013, by Peter I. Corke
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

    s = '''
# Frequently we want to define a smooth sequence of positions (or poses) from
# one point to another. First consider the 1-dimensional case.
#
# We define the start and end position

    p0 = -1
    p1 = 2
pause % press any key to continue
# and a smooth path from p0 to p1 in 50 time steps is given by

    [p, pd, pdd] = tpoly(p0, p1, 50);
    type(p)
    ndim(p)
    (nr,nc) = p.shape
    
# which we see has 50 rows.
pause % press any key to continue
# We can plot this

    k = range(1,len(p)+1);  % Align with Matlab/Octave
    plot(p);
    show(block=False);

# and see that it does indeed move smoothly from p0 to p1 and that
# the initial and final derivative (and second derivative) are zero.

pause % press any key to continue

# We can also get the velocity and acceleration

    [p, pd, pdd] = tpoly(p0, p1, 50, None, None, False);
    clf();
    k = range(1,len(p)+1);  % Align with Matlab/Octave
    subplot(3,1,1);
    plot(k,p);
    xlabel('Time'); 
    ylabel('p');
    subplot(3,1,2);
    plot(k,pd);
    xlabel('Time');
    ylabel('pd');
    subplot(3,1,3);
    plot(k,pdd);
    xlabel('Time');
    ylabel('pdd');
    show(block=False);
    
# This path is a 5th order polynomial and it suffers from the disadvantage
# that the velocity is mostly below the maximum possible value.

pause % press any key to continue

# An alternative is

    [p, pd, pdd] = lspb(p0, p1, 50, None, False);
    clf();
    k = range(1,len(p)+1);  % Align with Matlab/Octave
    subplot(3,1,1);
    plot(k,p);
    xlabel('Time'); 
    ylabel('p');
    subplot(3,1,2);
    plot(k,pd);
    xlabel('Time');
    ylabel('pd');
    subplot(3,1,3);
    plot(k,pdd);
    xlabel('Time');
    ylabel('pdd');
    show(block=False);
        
# which we see has a trapezoidal velocity profile.

pause % press any key to continue    

#
# Frequently the start and end values are vectors, not scalars, perhaps a 3D
# position or Euler angles.  In this case we apply the scalar trajectory function
# to a vector with

    [p, pd, pdd] = mtraj(tpoly, [0., 1., 2.], [2., 1., 0.], 50);
    type(p)
    ndim(p)
    (nr,nc) = shape(p)
    
# and p again has one row per time step, and one column per vector dimension

    clf();
    k = range(1,len(p)+1);  % Align with Matlab/Octave
    plot(p);
    show(block=False);
pause % press any key to continue
#---
# Finally, we may wish to interpolate poses. We will define a start and end pose

    T0 = transl(0.4, 0.2, 0) * trotx(pi);
    T1 = transl(-0.4, -0.2, 0.3) * troty(pi/2) * trotz(-pi/2);

# and a smooth sequence between them in 50 steps is

    T = ctraj(T0, T1, 50);
    type(T)
    len(T)
    (j,k) = T[0].shape
  
# which is 50 4x4 arrays. The first pose is

    T[0]

# and the 10th pose is

    T[9]
pause % press any key to continue

# We can plot the motion of this coordinate frame by

    close()
    tranimate(T, None, rec=1, movie="./images")
pause % press any key to continue
'''

    p.parsedemo(s)


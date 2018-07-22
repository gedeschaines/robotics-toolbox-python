# Copyright (C) 1993-2002, by Peter I. Corke

# $Log: rtfddemo.m,v $
# Revision 1.4  2002/04/02 12:26:48  pic
# Handle figures better, control echo at end of each script.
# Fix bug in calling ctraj.
#
# Revision 1.3  2002/04/01 11:47:17  pic
# General cleanup of code: help comments, see also, copyright, remnant dh/dyn
# references, clarification of functions.
#
# $Revision: 1.4 $

import _robot
from robot import parsedemo as p
import sys


if __name__ == '__main__':

    s = '''
#   help(fdyn)

# Forward dynamics is the computation of joint accelerations given position
# and velocity state, and actuator torques. It is useful in simulation of
# a robot control system.
#
# Consider the Puma 560,
    from robot.puma560 import *
    n = p560.n
    
# at rest in the zero angle pose.
    q0 = qz
    qd0 = zeros((1,n))
    qdd0 = zeros((1,n))
    
# With zero applied joint torques,
    torque = zeros((1,n))
pause % press any key to continue

# the manipulator joint accelerations would be determined through inverse 
# dynamics by employing a Recursive Newton-Euler technique to first compute 
# the manipulator inertia matrix,
    M = rne(p560, mat(ones((n,1)))*q0, zeros((n,n)), eye(n), gravity=zeros(3))
pause % press any key to continue
    
# then compute gravity and Coriolis induced torques at each joint,   
    tau = rne(p560, q0, qd0, qdd0)
    
# before finally solving for joint accelerations as:
    qdd = inv(M) * (torque.T - tau.T)
pause % press any key to continue

# Or more easily, manipulator joint accelerations may be computed by simply 
# calling the accel() function.
    qdd = accel(p560, q0, qd0, torque)
pause % press any key to continue

# To be useful for simulation this function must be integrated. This
# variant of fdyn() uses the SciPy ode('dopri5') algorithm to integrate
# the joint acceleration. It also allows for a user written function
# to compute the joint torque as a function of manipulator state.

# To simulate the motion of the Puma 560 from rest in the zero angle
# pose with zero applied joint torques, the differential equations of
# motion for the manipulator link/joint dynamics must be integrated.

    tic();                                           % performing integration ...
    [t, q, qd] = fdyn(p560.nofriction(), 0.0, 5.0);  % ( This may take a few minutes! )

# done.
    toc()
#
# and the resulting motion can be plotted versus time
#
    figure(1);
    subplot(3,1,1);
    plot(t,q[:,0]*180/pi);
    title("Joint Angles");
    xlabel('Time (s)');
    ylabel('Joint 1 (deg)');
    subplot(3,1,2);
    plot(t,q[:,1]*180/pi);
    xlabel('Time (s)');
    ylabel('Joint 2 (deg)');
    subplot(3,1,3);
    plot(t,q[:,2]*180/pi);
    xlabel('Time (s)');
    ylabel('Joint 3 (deg)');
    show(block=False);
    
    figure(2);
    subplot(2,1,1);
    plot(t,q[:,0]*180/pi);
    title("Joint 1 Rotation and Rotation Rate");
    xlabel('Time (s)');
    ylabel('Joint 1 (deg)');
    subplot(2,1,2);
    plot(t,qd[:,0]*180/pi);
    xlabel('Time (s)');
    ylabel('Joint 1 (deg/sec)');
    show(block=False);
#
# Clearly the robot is collapsing under gravity, but it is interesting to 
# note that rotational velocity of the upper and lower arm are exerting 
# centripetal and Coriolis torques on the waist joint, causing it to rotate.

pause % press any key to continue
    close('all')
#
# This can be shown in animation also
    rbplot(p560, q);

pause % press any key to continue

    close('all');
echo off
'''
    p.parsedemo(s)


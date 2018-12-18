% Copyright (C) 1993-2013, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

%%begin
echo off

% Forward dynamics is the computation of joint accelerations given position
% and velocity state, and actuator torques. It is useful in simulation of 
% a robot control system.
%
% Consider a Puma 560,
mdl_puma560;
n = p560.n

% at rest in the zero angle pose.

q0 = qz
qd0 = zeros(1,n)
qdd0 = zeros(1,n)

% With zero applied joint torques,

torque = zeros(1,n)
  
% the manipulator joint accelerations would be determined through inverse 
% dynamics by employing a Recursive Newton-Euler technique to first compute 
% the manipulator inertia matrix,

M = rne(p560, ones(n,1)*q0, zeros(n,n), eye(n), [0;0;0])

% then compute gravity and Coriolis induced torques at each joint,

tau = rne(p560, q0, qd0, zeros(1,n))

% before finally solving for joint accelerations as: 

qdd = inv(M) * (torque' - tau')

% Or more easily, manipulator joint accelerations may be computed by simply 
% calling the accel() function.

qdd = accel(p560, q0, qd0, torque)

% To be useful for simulation this function must be integrated. This 
% variant of fdyn() uses the MATLAB function ode45() to integrate the
% joint acceleration. It also allows for a user written function to
% compute the joint torque as a function of manipulator state.
%
% To simulate the motion of the Puma 560 from rest in the zero angle 
% pose with zero applied joint torques

tic
% perform integration ( This may take a few minutes! ) ...
[t q qd] = fdyn(nofriction(p560), 5.0);
% done.
toc

% and the resulting motion can be plotted versus time

subplot(3,1,1)
plot(t,q(:,1))
xlabel('Time (s)')
ylabel('Joint 1 (rad)')
subplot(3,1,2)
plot(t,q(:,2))
xlabel('Time (s)')
ylabel('Joint 2 (rad)')
subplot(3,1,3)
plot(t,q(:,3))
xlabel('Time (s)')
ylabel('Joint 3 (rad)')

% Clearly the robot is collapsing under gravity, but it is interesting to 
% note that rotational velocity of the upper and lower arm are exerting 
% centripetal and Coriolis torques on the waist joint, causing it to rotate.

% This can be shown in animation also
clf
p560.plot(q)

echo off

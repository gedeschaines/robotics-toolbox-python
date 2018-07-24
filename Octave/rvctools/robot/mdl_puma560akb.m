%MDL_PUMA560AKB Create model of Puma 560 manipulator
%
%      mdl_puma560akb
%
% MDL_PUMA560AKB is a script that creates the workspace variable p560m 
% which describes the kinematic and dynamic characterstics of a Unimation
% Puma 560 manipulator modified DH conventions.
%
% Also defines the workspace vectors:
%   qz, qz_m        zero joint angle configuration
%   qr, qr_m        vertical 'READY' configuration
%   qstretch,qs_m   arm is stretched out in the X direction
%
% References::
% -  "The Explicit Dynamic Model and Inertial Parameters of the Puma 560 Arm"
%    Armstrong, Khatib and Burdick
%    1986
%
% See also SerialLink, mdl_puma560, mdl_stanford, mdl_twolink.


% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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

clear L

% link DH parameters
%            theta    d        a    alpha
L(1) = Link([  0      0        0       0       0], 'modified');
L(2) = Link([  0      0.2435   0      -pi/2    0], 'modified');
L(3) = Link([  0     -0.0934   0.4318  0       0], 'modified');
L(4) = Link([  0      0.4331  -0.0203  pi/2    0], 'modified');
L(5) = Link([  0      0        0      -pi/2    0], 'modified');
L(6) = Link([  0      0        0       pi/2    0], 'modified');

% link mass
L(1).m = 0;
L(2).m = 17.4;
L(3).m = 4.8;
L(4).m = 0.82;
L(5).m = 0.34;
L(6).m = .09;

% link COG wrt link coordinate frame
%         rx        ry       rz
L(1).r = [0,        0,       0     ];
L(2).r = [0.068,    0.006,  -0.016 ];
L(3).r = [0,       -0.070,   0.014 ];
L(4).r = [0,        0,      -0.019 ];
L(5).r = [0,        0,       0     ];
L(6).r = [0,        0,       0.032 ];

% link inertia matrix about link COG
%         Ixx      Iyy      Izz      Ixy  Iyz  Ixz
L(1).I = [0,       0,       0.35,    0,   0,   0];
L(2).I = [0.13,    0.524,   0.539,   0,   0,   0];
L(3).I = [0.066,   0.0125,  0.066,   0,   0,   0];
L(4).I = [1.8e-3,  1.8e-3,  1.3e-3,  0,   0,   0];
L(5).I = [0.3e-3,  0.3e-3,  0.4e-3,  0,   0,   0];
L(6).I = [0.15e-3, 0.15e-3, 0.04e-3, 0,   0,   0];

% actuator motor inertia (motor referred)
L(1).Jm =  291e-6;
L(2).Jm =  409e-6;
L(3).Jm =  299e-6;
L(4).Jm =  35e-6;
L(5).Jm =  35e-6;
L(6).Jm =  35e-6;

% actuator gear ratio
L(1).G =  -62.6111;
L(2).G =  107.815;
L(3).G =  -53.7063;
L(4).G =   76.0364;
L(5).G =   71.923;
L(6).G =   76.686;

% viscous friction (motor referenced)
% unknown

% Coulomb friction (motor referenced)
% unknown

%
% some useful poses
%
qz = [0 0 0 0 0 0];           % zero angles, L shaped pose
qr = [0 -pi/2 pi/2 0 0 0];    % ready pose, arm up
qstretch = [0 0 pi/2 0 0 0];  % horizontal along x-axis

# following assignments avoid pose name collisions with mdl_puma560std
qz_m = qz;
qr_m = qr;
qs_m = qstretch;

p560m = SerialLink(L, 'name', 'Puma560-AKB', ...
                      'manufacturer', 'Unimation', 'comment', 'AK&B');
clear L
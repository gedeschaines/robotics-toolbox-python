%MDL_FOURLINK3D Load kinematic and dynamic data for a simple upright 4-link
% mechanism
%
%     mdl_fourlink3d
%
% Defines the object 'fl3d' in the current workspace which describes the
% kinematic and dynamic characterstics of a simple upright 4-link mechanism.
%
% Example based on IK_Solver 3D 4-link chain
%
% Assume links with all mass (unity) concentrated at the joints.
%
% Also defines the vectors qz, qn and qt which correspond to the zero joint
% angle, a nominal joint angle and a targeted joint angle configuration 
% respectively.
%
% See also mdl_puma560, mdl_puma560akb, mdl_stanford, mdl_phantomxo, mdl_twolink

clear L

% link DH parameters
%            theta   d       a     alpha sigma
L(1) = Link([0.0,    2.0,    0.0,  pi/2,     0]);
L(2) = Link([0.0,    0.0,    2.0,  0.0,      0]);
L(3) = Link([0.0,    0.0,    2.0,  0.0,      0]);
L(4) = Link([0.0,    0.0,    1.0,  0.0,      0]);

% link mass
L(1).m = 1.0;
L(2).m = 1.0;
L(3).m = 1.0;
L(4).m = 1.0;

% link COG wrt link coordinate frame
%         rx       ry       rz
L(1).r = [0.0,     0.0,     2.0];
L(2).r = [2.0,     0.0,     0.0];
L(3).r = [2.0,     0.0,     0.0];
L(4).r = [1.0,     0.0,     0.0];

% link inertia matrix about link COG
%         Ixx       Iyy       Izz     Ixy  Iyz  Ixz
L(1).I = [0.0,      0.0,      4.0,      0,   0,   0];
L(2).I = [0.0,      4.0,      0.0,      0,   0,   0];
L(3).I = [0.0,      2.0,      0.0,      0,   0,   0];
L(4).I = [0.0,      0.5,      0.0,      0,   0,   0];

% actuator motor inertia (motor referred)
L(1).Jm = 0.0;
L(2).Jm = 0.0;
L(3).Jm = 0.0;
L(4).Jm = 0.0;

% actuator gear ratio
L(1).G = 1.0;
L(2).G = 1.0;
L(3).G = 1.0;
L(4).G = 1.0;

% viscous friction (motor referenced)
% unknown

% Coulomb friction (motor referenced)
% unknown

%
% some useful poses
%
qz = [0.0, 0.0, 0.0, 0.0];  % zero angles, straight along x-axis pose
qn = [pi/4, pi/7.2, -pi/7.2, pi/18];   % nominal pose
qt = [0.0323078, 0.8323767, -1.3306788, -0.0075720];  % targeted pose
% prevent name collision with other robots
qz_fl3d = qz;
qn_fl3d = qn;
qt_fl3d = qt;

# create robot with serial-link manipulator
fl3d = SerialLink(L, 'name', 'Simple upright four link');
fl3d.plotopt = {'nobase', 'ortho', 'nowrist', 'nojoints', ...
                'workspace', [-5.0 5.0 -5.0 5.0 0.0 5.0], 'mag', 0.5};
clear L


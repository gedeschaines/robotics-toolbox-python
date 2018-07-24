%MDL_FOURLINK2D Load kinematic and dynamic data for a simple planar 4-link
% mechanism
%
%     mdl_fourlink2d
%
% Defines the object 'fl2d' in the current workspace which describes the
% kinematic and dynamic characterstics of a simple planar 4-link mechanism.
%
% Example based on IK_Solver 2D 4-link chain
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
L(1) = Link([0.0,    0.0,    2.0,  0.0,      0]);
L(2) = Link([0.0,    0.0,    2.0,  0.0,      0]);
L(3) = Link([0.0,    0.0,    1.0,  0.0,      0]);
L(4) = Link([0.0,    0.0,    1.0,  0.0,      0]);

% link mass
L(1).m = 1.0;
L(2).m = 1.0;
L(3).m = 1.0;
L(4).m = 1.0;

% link COG wrt link coordinate frame
%         rx       ry       rz
L(1).r = [2.0,     0.0,     0.0];
L(2).r = [2.0,     0.0,     0.0];
L(3).r = [1.0,     0.0,     0.0];
L(4).r = [1.0,     0.0,     0.0];

% link inertia matrix about link COG
%         Ixx       Iyy       Izz     Ixy  Iyz  Ixz
L(1).I = [0.0,      0.0,      4.0,      0,   0,   0];
L(2).I = [0.0,      0.0,      2.0,      0,   0,   0];
L(3).I = [0.0,      0.0,      0.5,      0,   0,   0];
L(4).I = [0.0,      0.0,      0.5,      0,   0,   0];

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
qn = [pi/7.2, pi/12.0, pi/18.0, pi/36.0];   % nominal pose
qt = [0.53938, 0.98726, 0.85419, 0.48104];  % targeted pose
% prevent name collision with other robots
qz_fl2d = qz;
qn_fl2d = qn;
qt_fl2d = qt;

# create robot with serial-link manipulator
fl2d = SerialLink(L, 'name', 'Simple planar four link');
fl2d.plotopt = {'nobase', 'ortho', 'nowrist', 'nojoints', ...
                'workspace', [-6.0 6.0 -6.0 6.0 -1.0 1.0], 'mag', 1.0};
clear L


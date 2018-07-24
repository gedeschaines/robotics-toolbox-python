%MDL_PHANTOMXO Create model of PhantomX pincher manipulator
%
%      mdl_phantomxo
%
% Script creates the workspace variable px which describes the 
% kinematic characteristics of a PhantomX Pincher Robot, a 4 
% joint hobby class manipulator by Trossen Robotics.
%
% Also define the workspace vectors:
%   qz -  zero joint angle configuration
%   qn -  a nominal joint angle configuration
%
% Notes::
% - the x-axis is forward, and the z-axis is upwards.
% - uses standard DH conventions.
% - Tool centrepoint is middle of the fingertips.
% - all translational units in mm.
%
% Reference::
%
% - http://www.trossenrobotics.com/productdocs/assemblyguides/phantomx-basic-robot-arm.html

clear L

% link DH parameters
%            theta   d        a     alpha
L(1) = Link([0.0,   40.0,     0.0,  -pi/2,  0], 'standard');
L(2) = Link([0.0,    0.0,  -105.0,   pi,    0], 'standard', 'offset', pi/2);
L(3) = Link([0.0,    0.0,  -105.0,   0,     0], 'standard');
L(4) = Link([0.0,    0.0,  -105.0,   0,     0], 'standard');

% Note alpha_2 = pi, needed to account for rotation axes of joints 3 
% and 4 having opposite sign to joint 2.
%
% s='Rz(q1) Tz(L1) Ry(q2) Tz(L2) Ry(q3) Tz(L3) Ry(q4) Tz(L4)'
% DHFactor(s)

px = SerialLink(L, 'name', 'PhantomX', 'manufacturer', 'Trossen Robotics');
qz = [0.0, 0.0,  0.0,  0.0];  % zero joint angle pose, arm along -x axis
qn = [0.0, 0.6, -1.0, -1.2];  % a nominal pose
% prevent name collision with other robots
qz_px = qz;
qn_px = qn;

px.tool = trotz(-pi/2) * trotx(pi/2);

clear L
% Test Kinematics for Stanford Arm

echo off;

mdl_stanford;
stanf

echo on;

% These are required when number of ikine optional arguments > 2
qz = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  %  initial joint vector
m6 = [1, 1, 1, 1, 1, 1]              %  mask array

%% Forward and Inverse Kinematics Test -- nominal poses

q1o = [1.0, 2.0, 0.25, 3.0, 2.0, 1.0]
T1  = fkine(stanf, q1o)
q1i = ikine(stanf, T1, qz, m6, 'verbose','alpha', 0.25, 'ilimit', 1000)
fkine(stanf, q1i)

q2o = [1.0, 2.0, 1.00, 0.0, 2.0, 3.0]
T2  = fkine(stanf, q2o)
q2i = ikine(stanf, T2, qz, m6, 'verbose', 'alpha', 0.25, 'ilimit', 1000)
fkine(stanf, q2i)

%% Trajectory case for Kinematics Test

Qt1o = [[0,0,0,0,0,0]; [1,0,1,0,1,0]; [1,0,0.8,0,1,0]]
Tt1  = fkine(stanf, Qt1o)
Qt1i = ikine(stanf, Tt1, qz, m6, 'verbose', 'alpha', 0.4, 'ilimit', 3000)
fkine(stanf, Qt1i)

%% Manipulability Test

qmp = [pi/3, pi/6, 0.3, -3*pi/4, -2*pi/5, pi/6]
mpy = maniplty(stanf, qmp, 'method', 'yoshikawa', 'axes', 'all')
mpa = maniplty(stanf, qmp, 'method', 'asada', 'axes', 'all')

echo off;

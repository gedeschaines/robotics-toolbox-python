% Test inverse kinematics for Puma 560 standard

echo off

mdl_puma560;
p560

echo on

% at nominal pose
qn
t = fkine(p560, qn)
q = ikine6s(p560, t)
fkine(p560, q)
ikine(p560, t, [0, 0.7, 3, 0, 0.7, 0], [1, 1, 1, 1, 1, 1], 'verbose=2')

echo off


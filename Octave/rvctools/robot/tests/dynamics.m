% Test dynamics primatives

q1 = mat(ones(1,6))

% test the RNE primitive for DH using standard Puma560
mdl_puma560;
p560

#echo on

% -- use qz with three state parameters
qz
rne(p560, qz, qz, qz)
rne(p560, qz, qz, qz, [1,2,3])
rne(p560, qz, qz, qz, [0,0,9.81], [1,2,3,0,0,0])
rne(p560, qz, 0.2*q1, qz, [0,0,9.81], [0,0,0,0,0,0])
rne(p560, qz, 0.2*q1, 0.2*q1)

% -- use qr with three state parameters
qr
rne(p560, qr, qz, qz)
rne(p560, qr, qz, qz, [1,2,3])

% -- use qr with single state parameters
z = [qr qz qz]
rne(p560, z, [1,2,3])
rne(p560, z, [0,0,9.81], [1,2,3,0,0,0])
rne(p560, [z;z;z])

% test the RNE primitive for MDH using modified Puma560
cd ..
run ./robot/mdl_puma560akb
cd ./test
p560m

% -- use qz with three state parameters
qz
rne(p560m, qz, qz, qz)
rne(p560m, qz, qz, qz, [1,2,3])
rne(p560m, qz, qz, qz, [0,0,9.81], [1,2,3,0,0,0])
rne(p560m, qz, 0.2*q1, qz)
rne(p560m, qz, 0.2*q1, 0.2*q1)

% -- use qr with three state parameters
qr
rne(p560m, qr, qz, qz)
rne(p560m, qr, qz, qz, [1,2,3])

% -- use qr with single state parameters
z = [qr qz qz]
rne(p560m, z, [1,2,3])
rne(p560m, z, [0,0,9.81], [1,2,3,0,0,0])
rne(p560m, [z;z;z])

% at zero pose
gravload(p560, qz)
gravload(p560, qz, [9,0,0])
gravload(p560, [qz;qz;qz])

inertia(p560, qz)
inertia(p560, [qz;qr])

accel(p560, qz, 0.2*q1, 0.5*q1)

% need to pick a non-singular configuration for cinertia()
cinertia(p560, qn)

coriolis(p560, qn, 0.5*q1)

% along trajectory
[q, qd, qdd] = jtraj(qz, qr, 20)
rne(p560, q, qd, qdd)

echo off

p560.plot(q,'joints','wrist','shadow')


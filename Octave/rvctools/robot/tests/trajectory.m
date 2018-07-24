% Test trajectory primitives

warning ("off", 'Octave:possible-matlab-short-circuit-operator');

echo off

mdl_puma560;
p560

echo on

[q,qd,qdd] = jtraj(qz, qr, 20)
q
qd
qdd

[q,qd,qdd] = jtraj(qz, qr, 20, 0.1*mat(ones(1,6)), -0.1*mat(ones(1,6)))
q
qd
qdd

[q,qd,qdd] = jtraj(qz, qr, [0:0.2:10])
q

t1 = trotx(0.1) * transl(0.2, 0.3, 0.4)
t1
t2 = troty(-0.3) * transl(-0.2, -0.3, 0.6)
t2
ctraj(t1, t2, 5)
ctraj(t1, t2, [0:0.1:1])

echo off


% Test quaternion primitives

warning ("off", 'Octave:possible-matlab-short-circuit-operator');
echo on

q = Quaternion()
q2 = Quaternion([1 0 0 0])
%%assertEqual(q,q2)

q = Quaternion(rotx(0.3))
q2 = Quaternion(trotx(0.3))
%%assertEqual(q,q2)

%%q.R();
%%q.T();

q+q2
q*q2

% test the values
q/q2
q^3
q.inv()
q.norm()
q.unit()
q.scale(0)
q.scale(1)

%test these equal qi and q
q.dot([1 2 3])
q*0.1
q/0.1
q*[1 2 3]
q*[1 2 3]'
q.plot()

Quaternion( 0.1 )
Quaternion( mat([1,2,3]), 0.1 )
Quaternion( rotx(0.1) )
Quaternion( trotx(0.1) )
Quaternion( Quaternion(0.1) )
Quaternion( mat([1,2,3,4]) )
Quaternion( [1,2,3,4] )
Quaternion( mat([1,2,3]), 0.1 )

q1 = Quaternion( rotx(0.1) )
q1.norm()
q1.unit()
q1.norm()
q1.double()
q1.r()
q1.tr()

q1 = Quaternion( rotx(0.1) )
q2 = Quaternion( roty(0.2) )
q1_t = q1
q2_t = q2

q1*2
2*q1
q1*q1
q1+q2
q1-q2
q1*q2
q1^1
q1^2

q2.inv()
inv(q2*q2)
q2*q2^(-1)
q1/q2

v1 = [0, 1, 0]
q1*v1
q1*(v1')

q1
q2
q1.interp(q2, 0)
q1.interp(q2, 1)
q1.interp(q2, 0.5)
qi = q1.interp(q2, [0, .2, .5, 1]);

q1-q1_t
q2-q2_t

echo off

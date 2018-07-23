class ptb:
    '''
# Robotics Toolbox for Python - Update 1
#
# Based on Matlab Toolbox Version 7  April-2002
# Updated kinematics and dynamics based on Matlab Toolbox Version 9  July-2018
#
# What's new.
#   Readme.md   - New features and enhancements in this version.
#
# Homogeneous transformations
#   eul2tr      - Euler angle to transform 
#   oa2tr       - orientation and approach vector to transform 
#   rotx        - transform for rotation about X-axis 
#   roty        - transform for rotation about Y-axis 
#   rotz        - transform for rotation about Z-axis 
#   rpy2tr      - roll/pitch/yaw angles to transform
#   rt2tr       - rotation matrix and translation vector to transform
#   tr2angvec   - convert rotation matrix to angle-vector form
#   tr2eul      - transform to Euler angles 
#   tr2rot      - transform to rotation submatrix
#   tr2rpy      - transform to roll/pitch/yaw angles
#   transl      - set or extract the translational component of a transform 
#   trnorm      - normalize a transform 
#   
# Quaternions
#   /           - divide quaternion by quaternion or scalar
#   *           - multiply quaternion by a quaternion or vector
#   inv         - invert a quaternion 
#   norm        - norm of a quaternion 
#   plot        - display a quaternion as a 3D rotation
#   qinterp     - interpolate quaternions
#   unit        - unitize a quaternion 
#
# Kinematics
#   diff2tr     - differential motion vector to transform 
#   fkine       - compute forward kinematics 
#   ikine       - compute inverse kinematics 
#   ikine560    - compute inverse kinematics for Puma 560 like arm
#   jacob0      - compute Jacobian in base coordinate frame
#   jacobn      - compute Jacobian in end-effector coordinate frame
#   tr2diff     - transform to differential motion vector 
#   tr2jac      - transform to Jacobian 
#   
# Dynamics
#   accel       - compute forward dynamics
#   cinertia    - compute Cartesian manipulator inertia matrix 
#   coriolis    - compute centripetal/coriolis torque
#   fdyn        - forward dynamic integration using 'lsode' method
#   friction    - joint friction
#   gravload    - compute gravity loading 
#   inertia     - compute manipulator inertia matrix 
#   itorque     - compute inertia torque 
#   nofriction  - remove friction from a robot object 
#   rne         - inverse dynamics using Recursive Newton-Euler
#   wtrans      - transform wrench (force/moment 6-vector)
#   
# Trajectory generation
#   ctraj       - Cartesian trajectory 
#   jtraj       - joint space trajectory
#   mpoly       - multi-axis trajactory
#   tpoly       - scalar polynomial trajectory
#   trinterp    - interpolate transform s
#   
# Graphics
#   qplot       - joint angle trajectory plot
#   rbplot      - plot/animate robot
#   trplot      - plot transform frame plot
#   tranimate   - animate transform frame
#
# Other
#   manipblty   - compute manipulability 
#   unit        - unitize a vector
#
# Creation of robot models.
#   link        - construct a robot link object
#   phantomx    - PhantomX Pincher Robot data
#   puma560     - Puma 560 data 
#   puma560akb  - Puma 560 data (modified Denavit-Hartenberg)
#   robot       - construct a robot object 
#   stanford    - Stanford arm data 
#   twolink     - simple 2-link example
#   fourlink2d  - simple 4-link 2D planar manipulator arm
#   fourlink3d  - simple 4-link 3D upright manipultor arm
#
# Demonstrations.
#   transform.py    transforms and quaternions
#   trajectory.py   trajectories
#   animation.py    animations
#   fkine.py        forward kinematics
#   jacobian.py     Jacbians
#   ikine.py        inverse dynamics
#   idyn.py         inverse dynamics, RNE etc.
#   fdyn.py         forward dynamics
#   

# Copyright (C) 2002, by Peter I. Corke
#   $Log: Contents.m,v $
#   Revision 1.2  2002/05/26 23:01:33  pic
#   Remove functions no longer supported.
#
#   Revision 1.1  2002/04/01 11:47:11  pic
#   General cleanup of code: help comments, see also, copyright, remnant dh/dyn
#   references, clarification of functions.
#
#   $Revision: 1.2 $
'''
    pass;

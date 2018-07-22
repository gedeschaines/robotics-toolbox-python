"""
Robot manipulability operations.

Python implementation by: Luis Fernando Lara Tobar and Peter Corke.
Based on original Robotics Toolbox for Matlab code by Peter Corke.
Permission to use and copy is granted provided that acknowledgement
of the authors is made.

@author: Luis Fernando Lara Tobar and Peter Corke
"""

from numpy import *
from robot.jacobian import jacob0
from robot.dynamics import inertia
from numpy.linalg import inv,eig,det,svd
from robot.utility import *


def manipblty(robot, q, *args):
    '''
    MANIPBLTY Manipulability measure

        - M = MANIPBLTY(ROBOT, Q)
        - M = MANIPBLTY(ROBOT, Q, WHICH)

    Computes the manipulability index for the manipulator at the given pose.

    For an n-axis manipulator Q may be an n-element vector, or an m x n
    joint space trajectory.

    If Q is a vector MANIPBLTY returns a scalar manipulability index.
    If Q is a matrix MANIPBLTY returns a column vector of manipulability 
    indices for each pose specified by Q.

    The argument WHICH can be either 'y[oshikawa]' (default) or 'a[sada]'
    and selects one of two manipulability measures.
    Yoshikawa's manipulability measure gives an indication of how far 
    the manipulator is from singularities and thus able to move and 
    exert forces uniformly in all directions.

    Asada's manipulability measure is based on the manipulator's
    Cartesian inertia matrix.  An n-dimensional inertia ellipsoid
        X' M(q) X = 1
    gives an indication of how well the manipulator can accelerate
    in each of the Cartesian directions.  The scalar measure computed
    here is the ratio of the smallest/largest ellipsoid axis.  Ideally
    the ellipsoid would be spherical, giving a ratio of 1, but in
    practice will be less than 1.

    @see: inertia, jacob0
    '''        
    n = robot.n
    q = mat(q)
    w = array([])
    
    which = 'yoshikawa'
    if args is not None and (len(args) > 0):
         which = '%s' % args[0]
    
    if which.startswith('y'):
        if numrows(q)==1:
            return yoshi(robot,q)
        for Q in q:
            w = concatenate((w,array([yoshi(robot,Q)])))
    elif which.startswith('a'):
        if numrows(q)==1:
            return asada(robot,q)
        for Q in q:
            w = concatenate((w,array([asada(robot,Q)])))
    return mat(w)

def yoshi(robot,q):
    J = jacob0(robot,q)
    return sqrt(det(J*J.T))

def asada(robot,q):
    J = jacob0(robot,q)
    Ji = inv(J)
    M = inertia(robot,q)
    Mx = Ji.T*M*Ji
    e = eig(Mx)[0]
    return real( e.min(0)/e.max(0) )
        

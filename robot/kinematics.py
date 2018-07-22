"""
Robot kinematic operations.

Python implementation by: Luis Fernando Lara Tobar and Peter Corke.
Based on original Robotics Toolbox for Matlab code by Peter Corke.
Permission to use and copy is granted provided that acknowledgement
of the authors is made.

@author: Luis Fernando Lara Tobar and Peter Corke
"""

from numpy import *
from robot.utility import *
from robot.transform import *
from robot.jacobian import *
from numpy.linalg import norm, pinv, cond
from math import *
import warnings
import traceback

def fkine(robot, q):
    """
    Computes the forward kinematics for each joint space point defined by C{q}.
    ROBOT is a robot object.

    For an n-axis manipulator C{q} is an 1xn element vector or an m x n matrix
    of robot joint coordinates.

    If C{q} is a vector it is interpreted as the generalized joint coordinates,
    and C{fkine} returns a 4x4 homogeneous transformation for the tool of
    the manipulator.

    If C{q} is a matrix, the rows are interpreted as the generalized
    joint coordinates for a sequence of points along a trajectory.  q[i,j] is
    the j'th joint parameter for the i'th trajectory point.  In this case
    C{fkine} returns a list of matrices for each point
    along the path.

    The robot's base or tool transform, if present, are incorporated into the
    result.

    @type robot: Robot instance
    @param robot: The robot
    @type q: vector
    @param q: joint coordinate
    @see: L{Link}, L{Robot}, L{ikine}
    """

    q = mat(q)
    n = robot.n
    
    if numrows(q)==1 and numcols(q)==n:
        t = robot.base
        for i in range(0,n):
            t = t * robot.links[i].tr(q[0,i])
        t = t * robot.tool
        return t
    else:
        if numcols(q) != n:
            error('RTB:fkine: bad data')
        t = []
        for qv in q:        # for each trajectory point
            tt = robot.base
            for i in range(0,n):
                tt = tt * robot.links[i].tr(qv[0,i])
            t.append(tt*robot.tool)
        return t


def ikine(robot, tr, q0=None, m=None, verbose=0, **args):
    """
    Inverse manipulator kinematics.
    Computes the joint coordinates corresponding to the end-effector transform C{tr}.
    Typically invoked as

        - Q = IKINE(ROBOT, T)
        - Q = IKINE(ROBOT, T, Q)
        - Q = IKINE(ROBOT, T, Q, M)

    Uniqueness
    ==========
    Note that the inverse kinematic solution is generally not unique, and
    depends on the initial guess C{q} (which defaults to 0).

    Iterative solution
    ==================
    Solution is computed iteratively using the pseudo-inverse of the
    manipulator Jacobian.

    Such a solution is completely general, though much less efficient
    than specific inverse kinematic solutions derived symbolically.

    This approach allows a solution to obtained at a singularity, but
    the joint angles within the null space are arbitrarily assigned.

    Operation on a trajectory
    =========================
    If C{tr} is a list of transforms (a trajectory) then the solution is calculated
    for each transform in turn.  The return values is a matrix with one row for each
    input transform.  The initial estimate for the iterative solution at
    each time step is taken as the solution from the previous time step.

    Fewer than 6DOF
    ===============
    If the manipulator has fewer than 6 DOF then this method of solution
    will fail, since the solution space has more dimensions than can
    be spanned by the manipulator joint coordinates.  In such a case
    it is necessary to provide a mask matrix, C{m}, which specifies the
    Cartesian DOF (in the wrist coordinate frame) that will be ignored
    in reaching a solution.  The mask matrix has six elements that
    correspond to translation in X, Y and Z, and rotation about X, Y and
    Z respectively.  The value should be 0 (for ignore) or 1.  The number
    of non-zero elements should equal the number of manipulator DOF.

    For instance with a typical 5 DOF manipulator one would ignore
    rotation about the wrist axis, that is, M = [1 1 1 1 1 0].


    @type robot: Robot instance
    @param robot: The robot
    @type tr: homgeneous transformation
    @param tr: End-effector pose
    @type q: nx1 vector
    @param q: initial estimate of joint coordinate
    @type m: vector
    @param m: mask vector
    @rtype: vector
    @return: joint coordinate
    @see: L{fkine}, L{tr2diff}, L{jacob0}, L{ikine560}
    """
    from robot.jacobian import jacob0
    
    #solution control parameters

    if 'debug' in args and args['debug'] == True : print("args %s" % args)

    n = robot.n

    if q0 is None:
        # robot.ikine(tr)
        q0 = mat(zeros((n,1)))
    else:
        # robot.ikine(tr, q)
        q0 = mat(q0).flatten().T

    if (q0 is not None) and (m is not None):
        # robot.ikine(tr, q, m)
        m = array(m, dtype='int').flatten()
        if len(m) != 6:
            error('Mask vector should have 6 elements')
        if len(m.nonzero()[0]) != robot.n:
            error('Mask vector must have same number of 1s as robot DOF')
    else:
        if n < 6:
            print("For a manipulator with fewer than 6DOF a mask vector argument should be specified")
        m = ones((6),dtype='int') 
        
    J0 = jacob0(robot, q0.T)  # jacob0 passes a [1 x n] to fkine; returns a [6 x n]
    J0 = J0[m>0,:]
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        try:
            condJ0 = cond(J0)
        except RuntimeWarning as err:
            traceback.print_exc()
            condJ0 = inf
            
    if condJ0 > 100.0:
        warning('RTB:ikine:singular',\
                'Initial joint configuration results in a (near-)singular configuration;\n'
                '                    this may slow convergence.')
    
    ## solve function block

    e = mat(zeros((6,1)))
    revolutes = [l.sigma == 0 for l in robot.links]
    
    def solve(robot, i, e, Tr, q, m, verbose=0, debug=0, \
              ilimit=1000, stol=1e-6, alpha=0.9, psdoinv=True, varstep=False, \
              piter=False, pstep=10):
        """
        i    - index of Tr
        e    - 6x1 matrix of iteration error
        Tr   - 4x4 homogeneous transformation matrix
        q    - nx1 robot manipulator joint vector
        mask - 1x6 joint mask array
        """
        
        if not ishomog(Tr):
            error('Tr must be 4*4 matrix')
            
        if debug : print("%s %s %s %s %s" % (ilimit, stol, alpha, psdoinv, varstep))
        
        n = robot.n
        
        # initialize state for the ikine loop
        count = 0                                           # for checking interation limit
        nm = inf                                            # for checking convergence
        eprev = mat([inf, inf, inf, inf, inf, inf]).T       # for checking divergence
        save_q = mat(empty((n,1)))                          # for adjusting step size
        save_e = mat([inf, inf, inf, inf, inf, inf]).T      # 
        save_eprev = mat([inf, inf, inf, inf, inf, inf]).T  #

        while True:
            # update the count and test against iteration limit
            count = count + 1;
            if count > ilimit:
                warning('RTB:ikine:solve', \
                        'iteration limit %d exceeded (row %d), final err %f' % (ilimit, i, nm))
                #q = nan*ones((n,1))
                break
                
            # compute the error
            Tq = fkine(robot, q.T)  # fkine expects a [1 x n] or an [npoints x n]
            
            e[0:3,0] = transl(Tr - Tq)[0:3,0]
            Rq = t2r(Tq)
            [th, nv] = tr2angvec(Rq.T*t2r(Tr))
            e[3:6,0] = (th*mat(nv).flatten().T)[0:3,0]
            
            # optionally adjust the step size
            if varstep:
                # test against last best error, only consider the DOF of
                # interest
                if norm(e[m>0,0]) < norm(save_e[m>0,0]):
                    # error reduced,
                    # let's save current state of solution and rack up the step size
                    save_q = q.copy()
                    save_e = e.copy()
                    save_eprev = eprev.copy()
                    alpha = alpha * (2.0**(1.0/8))
                    if verbose > 1:
                        print('step %d: raise alpha to %f\n' % (count, alpha))
                else:
                    # rats!  error got worse,
                    # restore to last good solution and reduce step size
                    q = save_q.copy()
                    e = save_e.copy()
                    eprev = save_eprev.copy()
                    alpha = alpha * 0.5
                    if verbose > 1:
                        print('step %d: drop alpha to %f\n' % (count, alpha))
        
            # compute the Jacobian
            J = jacob0(robot, q.T)  # jacob0 passes a [1 x n] to fkine; returns a [6 x n]
            
            # compute change in joint angles to reduce the error, 
            # based on the square sub-Jacobian
            if psdoinv:
                dq = alpha * (pinv(J[m>0,:]) * e[m>0,0])
            else:
                dq = alpha * ((J[m>0,:].T) * e[m>0,0])
                
            # update the estimated solution
            q = q + dq
            
            # wrap angles for revolute joints
            for k in range(0,n):
                if revolutes[k]:
                   if q[k] >  pi: q[k] = q[k] - 2*pi; continue
                   if q[k] < -pi: q[k] = q[k] + 2*pi; continue
            
            # compute current error metric (i.e., RSS of e vector)
            nm = norm(e[m>0,0])
            
            # diagnostic stuff
            if (verbose > 1) or (piter and ((count % pstep) == 1)):
                print('%d/%d: |e| = %f' % (i, count, nm))
                print('       e  = %s' % e.T)
                print('       dq = %s' % dq.T)
            
            # check for divergence 
            if nm > 2.0*norm(eprev[m>0,0]):
                warning('RTB:ikine:solve', \
                        'solution diverging at step %d, try reducing alpha' % count)
            eprev = e.copy()
            
            # check for convergence
            if nm <= stol:
                break
       
        if verbose:
            print('%d iterations\n' % count)
            
        return q, e, count
        
    ## end of solve function block 

    if isinstance(tr, list):
        # trajectory list case
        npoints = len(tr)
        qt = mat(zeros((npoints,n)))
        tcount = 0
        for i in range(0, npoints):
            [q, e, count] = solve(robot, i, e, tr[i], q0, m, verbose=verbose, **args)
            q0 = q
            qt[i] = q.T
            tcount += count
        if verbose and (npoints > 1):
           print('TOTAL %d iterations\n' % tcount)
        return qt
        
    elif (isinstance(tr, matrix) or isinstance(tr, ndarray)) and ndim(tr) == 3:
        # trajectory matrix case
        npoints = tr.shape[0]
        qt = mat(zeros((npoints,n)))
        tcount = 0
        for i in range(0, npoints):
            [q, e, count] = solve(robot, i, e, tr[i], q0, m, verbose=verbose, **args)
            q0 = q
            qt[i] = q.T
            tcount += count
        if verbose and (npoints > 1):
           print('TOTAL %d iterations\n' % tcount)    
        return qt
        
    elif ishomog(tr):
        # single transform case
        [q, e, count] = solve(robot, 0, e, tr, q0, m, verbose=verbose, **args)
        qt = q.T
        return qt
        
    else:
        # unknown trajectory
        error('tr must be 4*4 matrix')


def ikine560(robot, T, configuration=''):
    """
    Inverse kinematics for Puma 560-like robot, ie. 6-axis with a spherical wrist.

    The optional C{configuration} argument specifies the configuration of the arm in
    the form of a string containing one or more of the configuration codes
       - 'l' or 'r'    lefty/righty
       - 'u' or 'd'    elbow
       - 'n' or 'f'    wrist flip or noflip.

    The default configuration is 'lun'.

    Reference
    =========

    Inverse kinematics for a PUMA 560 based on the equations by Paul and Zhang
    From The International Journal of Robotics Research
    Vol. 5, No. 2, Summer 1986, p. 32-44.

    @author: Robert Biro (gt2231a@prism.gatech.edu) with Gary Von McMurray, GTRI/ATRP/IIMB, Georgia Institute of Technology, 2/13/95.

    @type robot: Robot instance
    @param robot: The robot
    @type T: homogeneous transformation
    @param T: End-effector pose
    @type configuration: string
    @param configuration: manipulator configuration comprising the letters: lrudnf
    @rtype: vector
    @return: joint coordinate
    """

    if robot.n != 6:
        error('Solution only applicable for 6DOF manipulator');

    if robot.mdh:
        error('Solution only applicable for standard DH conventions');


    # recurse over a list of transforms
    if isinstance(T, list):
        theta = [];
        for t in T:
            theta.append( ikine560(robot, t, configuration) );

        return theta;
        
    # recurse over a matrix of transforms  
    if (isinstance(T, matrix) or isinstance(T, ndarray)) and ndim(T) == 3:
        (nt,i,j) = T.shape
        theta = [];
        for k in range(0,nt):
            t = T[i]
            theta.append( ikine560(robot, t, configuration) );

        return theta;
        
    if not ishomog(T):
        error('T is not a homog xform');

    L = robot.links;
    a1 = L[0].A;
    a2 = L[1].A;
    a3 = L[2].A;

    for i in range(3, 6):
        if L[i].A != 0:
            error('wrist is not spherical')

    d1 = L[0].D;
    d2 = L[1].D;
    d3 = L[2].D;
    d4 = L[3].D;


    # undo base transformation
    T = linalg.inv(robot.base) * T;

    # The following parameters are extracted from the Homogeneous
    # Transformation as defined in equation 1, p. 34

    Ox = T[0,1];
    Oy = T[1,1];
    Oz = T[2,1];

    Ax = T[0,2];
    Ay = T[1,2];
    Az = T[2,2];

    Px = T[0,3];
    Py = T[1,3];
    Pz = T[2,3];

    # The configuration parameter determines what n1,n2,n4 values are used
    # and how many solutions are determined which have values of -1 or +1.

    configuration = configuration.lower();

    n1 = -1;    # L
    n2 = -1;    # U
    n4 = -1;    # N
    if 'l' in configuration:
        n1 = -1;

    if 'r' in configuration:
        n1 = 1;

    if 'u' in configuration:
        if n1 == 1:
            n2 = 1;
        else:
            n2 = -1;

    if 'd' in configuration:
        if n1 == 1:
            n2 = -1;
        else:
            n2 = 1;

    if 'n' in configuration:
        n4 = 1;

    if 'f' in configuration:
        n4 = -1;


    theta = zeros( (6,1) );

    #
    # Solve for theta(1)
    #
    # r is defined in equation 38, p. 39.
    # theta(1) uses equations 40 and 41, p.39,
    # based on the configuration parameter n1
    #

    r = sqrt(Px**2 + Py**2);
    if n1 == 1:
        theta[0] = atan2(Py,Px) + asin(d3/r);
    else:
        theta[0] = atan2(Py,Px) + pi - asin(d3/r);


    #
    # Solve for theta(2)
    #
    # V114 is defined in equation 43, p.39.
    # r is defined in equation 47, p.39.
    # Psi is defined in equation 49, p.40.
    # theta(2) uses equations 50 and 51, p.40, based on the configuration
    # parameter n2
    #

    V114 = Px*cos(theta[0]) + Py*sin(theta[0]);
    r = sqrt(V114**2 + Pz**2);

    x = (a2**2-d4**2-a3**2+V114**2+Pz**2) / (2.0*a2*r);
    if abs(x) > 1:
        error('point not reachable');
    Psi = acos(x);

    theta[1] = atan2(Pz,V114) + n2*Psi;

    #
    # Solve for theta(3)
    #
    # theta(3) uses equation 57, p. 40.
    #

    num = cos(theta[1])*V114+sin(theta[1])*Pz-a2;
    den = cos(theta[1])*Pz - sin(theta[1])*V114;
    theta[2] = atan2(a3,d4) - atan2(num, den);

    #
    # Solve for theta(4)
    #
    # V113 is defined in equation 62, p. 41.
    # V323 is defined in equation 62, p. 41.
    # V313 is defined in equation 62, p. 41.
    # theta(4) uses equation 61, p.40, based on the configuration
    # parameter n4
    #

    V113 = cos(theta[0])*Ax + sin(theta[0])*Ay;
    V323 = cos(theta[0])*Ay - sin(theta[0])*Ax;
    V313 = cos(theta[1]+theta[2])*V113 + sin(theta[1]+theta[2])*Az;
    theta[3] = atan2((n4*V323),(n4*V313));
    #[(n4*V323),(n4*V313)]

    #
    # Solve for theta(5)
    #
    # num is defined in equation 65, p. 41.
    # den is defined in equation 65, p. 41.
    # theta(5) uses equation 66, p. 41.
    #

    num = -cos(theta[3])*V313 - V323*sin(theta[3]);
    den = -V113*sin(theta[1]+theta[2]) + Az*cos(theta[1]+theta[2]);
    theta[4] = atan2(num,den);
    #[num den]

    #
    # Solve for theta(6)
    #
    # V112 is defined in equation 69, p. 41.
    # V122 is defined in equation 69, p. 41.
    # V312 is defined in equation 69, p. 41.
    # V332 is defined in equation 69, p. 41.
    # V412 is defined in equation 69, p. 41.
    # V432 is defined in equation 69, p. 41.
    # num is defined in equation 68, p. 41.
    # den is defined in equation 68, p. 41.
    # theta(6) uses equation 70, p. 41.
    #

    V112 = cos(theta[0])*Ox + sin(theta[0])*Oy;
    V132 = sin(theta[0])*Ox - cos(theta[0])*Oy;
    V312 = V112*cos(theta[1]+theta[2]) + Oz*sin(theta[1]+theta[2]);
    V332 = -V112*sin(theta[1]+theta[2]) + Oz*cos(theta[1]+theta[2]);
    V412 = V312*cos(theta[3]) - V132*sin(theta[3]);
    V432 = V312*sin(theta[3]) + V132*cos(theta[3]);
    num = -V412*cos(theta[4]) - V332*sin(theta[4]);
    den = - V432;
    theta[5] = atan2(num,den);
    #[num den]

    return mat(theta).T;
    

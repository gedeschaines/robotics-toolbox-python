"""
Robot dynamics operations.

Python implementation by: Luis Fernando Lara Tobar and Peter Corke.
Based on original Robotics Toolbox for Matlab code by Peter Corke.
Permission to use and copy is granted provided that acknowledgement
of the authors is made.

@author: Luis Fernando Lara Tobar and Peter Corke
"""

from numpy import *
from numpy.linalg import inv
from scipy.integrate import odeint, ode
from robot.utility import *
from robot import jacobian as Jac
from robot.transform import *


def accel(robot, *args):
    """
    Compute manipulator forward dynamics, the joint accelerations that result
    from applying the actuator torque to the manipulator robot in state q and qd.

       - qdd = accel(robot, q, qd, torque)
       - qdd = accel(robot, [q qd torque])

    Uses the method 1 of Walker and Orin to compute the forward dynamics.
    This form is useful for simulation of manipulator dynamics, in
    conjunction with a numerical integration function.

    @type robot: Robot object, n-axes
    @param robot: The robot
    @rtype: n-vector
    @return: Joint coordinate acceleration
    @bug: Should handle the case of q, qd are matrices
    @see: L{rne}, L{robot}
    
    @see: L{disclaimer_rtb}
    """

    n = robot.n
    if len(args) == 1:
        q = mat(args[0])[0,0:n].T          # a vector
        qd = mat(args[0])[0,n:2*n].T       # a vector
        torque = mat(args[0])[0,2*n:3*n]   # an array
    else:
        q = mat(args[0])
        if numcols(q) == robot.n:
            q = q.flatten().T              # a vector
            qd = mat(args[1]).flatten().T  # a vector   
        torque = mat(args[2])              # an array
     
    # Compute current manipulator inertia matrix
    #   Torques resulting from unit acceleration of each joint with
    #   no gravity
    M = rne(robot, mat(ones((n,1)))*q.T, zeros((n,n)), eye(n), gravity=zeros(3))

    # Compute gravity and coriolis torque
    #   torques resulting from zero acceleration at given velocity &
    #   with gravity acting
    tau = rne(robot, q.T, qd.T, zeros((1,n)))  # tau returned as an array
    
    qdd = inv(M) * (torque.flatten().T - tau.T)  # qdd is a vector
    
    return qdd.T  # returns qdd as an array


def fdyn2(x, t, flags, robot, torqfun):   # for odeint()
#def fdyn2(t, x, flags, robot, torqfun):  # for ode45()
    """
    FDYN2  private function called by FDYN

        XDD = FDYN2(T, X, FLAG, ROBOT, TORQUEFUN)

    Called by FDYN to evaluate the robot velocity and acceleration for
    forward dynamics.  T is the current time, X = [Q QD] is the state vector,
    ROBOT is the object being integrated, and TORQUEFUN is the string name of
    the function to compute joint torques and called as

        TAU = TORQUEFUN(T, X)

    if not given zero joint torques are assumed.

    The result is XDD = [QD QDD].

    @see: L{fdyn}

    MOD HISTORY
        4/99 add object support

    $Log: fdyn2.m,v $
    Revision 1.2  2002/04/14 10:14:04  pic
    Added support for extra command line arguments passed to torqfun.
    Update comments.

    Revision 1.1  2002/04/01 11:47:12  pic
    General cleanup of code: help comments, see also, copyright, remnant dh/dyn
    references, clarification of functions.

    $Revision: 1.2 $
    Copyright (C) 1999-2002, by Peter I. Corke
    
    @see: L{disclaimer_rtb}
    """
    
    if robot is None:
        error('Missing robot in calling function fdyn() args tuple')
        
    n = robot.n  # number of joints
    
    q = array([x[0:n]])     # joint positions (angular or linear)
    qd = array([x[n:2*n]])  # joint velocities (angular or linear)
    
    # evaluate the torque function if one is given
    if torqfun is not None:
        tau = torqfun(t, q, qd, flags)
    else:
        tau = mat(zeros((n,1))).flatten().T
                
    qdd = accel(robot, q, qd, tau)
    
    xd = array([qd,qdd],dtype='float64').flatten()

    return xd
    
    
def ode45(f, x0, t, flags, robot, torqfun, debug, ode_printmessg):
    if len(t) < 2:
       print("error: ode45: t array have a least two values.")
       return x0
    
    dt = t[1] - t[0]
    r = ode(f)
    r.set_integrator('dopri5', first_step= dt/2, max_step=dt/2.0, verbosity=0)
    r.set_f_params(flags, robot, torqfun)
    r.set_initial_value(x0, t=t[0])
    k = len(t)
    Y = zeros((k,size(x0)))
    Y[0,:] = x0
    i = 1
    while r.successful() and i < k:
        r.integrate(t[i])
        if debug : print("%g %g" % (r.t, r.y))
        Y[i,:] = r.y
        i = i + 1
        
    if i == k and ode_printmessg:
        print("Integration successful.")
        
    return Y


def fdyn(robot, t0, t1, args=(), debug=0, ode_printmessg=True, ode_full_output=False):
    """
    Forward dynamic integration to generate time profile of joint coordinates (q) and
    velocities (qd) over time period [t0, t1] using function fdyn2() to calculate the
    joint accelerations for applied actuator torques, external loads and frictional 
    forces at specific manipulator states q and qd.

       - [t, q, qd] = fdyn(robot, t0, t1, *arg, **kwargs)

    @type robot: Robot object, n-axes
    @param robot: The robot
    @param t0: integration start time (sec)
    @param t1: integration stop time (sec)
    @param args: optional fdyn2() inputs -- (torqfun, q and qd)
    @param kwargs: fdyn() debug level #, odeint() printmessg and full_output params
    @rtype: one k length array, and two k x n size arrays
    @return: time (t), joint coordinates (q), joint coordinate velocities (qd)

    @see: L{fdyn2}, L{accel}, L{nofriction}, L{rne}, L{Robot}, L{ode45}.

    Copyright (C) 1993 Peter Corke

    MOD HISTORY
      4/99 add object support
    $Log: fdyn.m,v $
    Revision 1.4  2002/04/14 10:14:04  pic
    Added support for extra command line arguments passed to torqfun.
    Update comments.

    Revision 1.3  2002/04/01 11:47:13  pic
    General cleanup of code: help comments, see also, copyright, remnant dh/dyn
    references, clarification of functions.

    $Revision: 1.4 $

    Copyright (C) 1999 Peter Corke
    
    @see: L{disclaimer_rtb}
    """
    
    n = robot.n  # number of joints
    
    if args is None or len(args) < 1:
        torqfun = None
        x0 = array(zeros(2*n),dtype='float64').flatten()
    elif len(args) < 2:
        torqfun = args[0]
        x0 = array(zeros(2*n),dtype='float64').flatten()
    elif len(args) < 3:
        torqfun = args[0]
        x0 = array([args[1], zeros(n)],dtype='float64').flatten()
    else:
        torqfun = args[0]
        x0 = array([args[1], args[2]],dtype='float64').flatten()

    if debug:
        print("fdyn: x0 = %s" % x0)
        print("    : torqfun = %s" % torqfun)
        ode_printmesg = True
        ode_full_output = True
        
    dt = 0.01
    k = int(floor((t1+dt/2-t0)/dt)) + 1
    t = array(array([range(k)],dtype='float64')*dt + t0).flatten()
    
    output = odeint(fdyn2, x0, t, args=({}, robot, torqfun), \
                    printmessg=ode_printmessg, full_output=ode_full_output)
    
    """               
    output = ode45(fdyn2, x0, t, (), robot, torqfun, debug, ode_printmessg)
    """
    
    if ode_full_output:
       Y = output[0]
       infodict = output[1]
    else:
       Y = output
       infodict = None
       
    if debug and infodict is not None:
        print("* odeint-infodict:")
        for item in infodict.iterkeys():
            print("--+ %s:\n%s" % (item, infodict[item]))
    
    if size(Y.shape) == 2 and len(Y[0,:]) == 2*n:
        q  = Y[:,0:n]
        qd = Y[:,n:2*n]
    else:
        t  = t[0]
        q  = x0[0:n]
        qd = x0[n:2*n]

    return [t, q, qd]
    
    
def coriolis(robot, q, qd):
    """
    Compute the manipulator Coriolis matrix

    c = coriolis(robot, q, qd)

    Returns the n-element Coriolis/centripetal torque vector at the specified
    pose and velocity.

    If C{q} and C{qd} are row vectors, the result is a row vector
    of joint torques.
    If C{q} and C{qd} are matrices, each row is interpreted as a joint state
    vector, and the result is a matrix each row being the corresponding joint
    torques.

    @type q: M{m x n} matrix
    @type q: Joint coordinate
    @type qd: M{m x n} matrix
    @type qd: Joint coordinate velocity
    @type robot: Robot object, n-axes
    @param robot: The robot
    @rtype: M{m x n} matrix
    @return: Joint coordinate acceleration
    @see: L{robot}, L{rne}, L{itorque}, L{gravload}
    """
    
    n = robot.n
    q = mat(q)
    qd = mat(qd)
    
    if numcols(q) != n:
        emsg = 'q must have %d columns' % (n)
        error(emsg)

    if numcols(qd) != robot.n:
        emsg = 'qd must have %d columns' % (n)
        error(emsg)
    
    # we need to create a clone robot with no friciton, since friction
    # is also proportional to joint velocity.
    
    robot2 = robot.nofriction('all')

    if numrows(q) > 1:
        if numrows(q) != numrows(qd):
            error('for trajectory q and qd must have same number of rows')

        C = []
        for i in range(0,numrows(q)):
            C = concatenate(3, C, coriolis(robot2, q[i,:], qd[i,:]))
        return C
    
    N = robot2.n
    C = zeros((N,N))
    Csq = zeros((N,N))

    # find the torques that depend on a single finite joint speed,
    # these are due to the squared (centripetal) terms
    #
    # set QD = [1 0 0 ...] then resulting torque is due to qd_1^2
    for j in range(0,N):
        QD = zeros(N)
        QD[j] = 1
        tau = rne(robot2, q, QD, zeros(size(q)), gravity=zeros(3))
        Csq[:,j] = Csq[:,j] + tau[:]

    # find the torques that depend on a pair of finite joint speeds,
    # these are due to the product (Coridolis) terms
    
    # set QD = [1 1 0 ...] then resulting torque is due to 
    #    qd_1 qd_2 + qd_1^2 + qd_2^2
    
    for j in range(0,N):
        for k in range(j+1,N):
            # find a product term  qd_j * qd_k
            QD = zeros(N)
            QD[j] = 1
            QD[k] = 1
            tau = rne(robot2, q, QD, zeros(size(q)), gravity=zeros(3))
            C[:,k] = C[:,k] + (tau[:] - Csq[:,k] - Csq[:,j]) * qd[0,j]
    
    C = C + Csq * diag(qd);
    
    return C


def inertia(robot, q):
    """
    Compute the manipulator inertia matrix

    inertia(robot, q)

    Returns the M{n x n} symmetric inertia matrix which relates joint torque
    to joint acceleration for the robot in pose C{q}.

    If C{q} is a matrix then return a list of inertia matrices, corresponding
    to the joint coordinates from rows of C{q}.

    @type q: M{m x n} matrix
    @type q: Joint coordinate
    @type robot: Robot object, n-axes
    @param robot: The robot
    @rtype: m-list of M{n x n} matrices
    @return: List of inertia matrices
    @see: L{cinertia}, L{rne}, L{robot}
    """
    n = robot.n
    q = mat(q)
    if numrows(q) > 1:
        ilist = [];
        for row in q:
            I = rne(robot, ones((n, 1))*row, zeros((n,n)), eye(n), gravity=zeros(3))
            ilist.append( I )
        return ilist
    else:
        return rne(robot, ones((n, 1))*q, zeros((n,n)), eye(n), gravity=zeros(3))


def cinertia(robot, q):
    """
    Compute the Cartesian (operational space) manipulator inertia matrix

    m = cinertia(robot, q)

    Return the M{6 x 6} inertia matrix which relates Cartesian force/torque to
    Cartesian acceleration.

    @type q: n-vector
    @type q: Joint coordinate
    @type robot: Robot object, n-axes
    @param robot: The robot
    @rtype: M{6 x 6} matrix
    @return: Cartesian inertia matrix
    @bug: Should handle the case of q as a matrix
    @see: L{inertia}, L{rne}, L{robot}
    """

    J = Jac.jacob0(robot, q)
    Ji = inv(J)
    M = inertia(robot, q)
    return Ji.T*M*Ji


def gravload(robot, q, gravity=None):
    """
    Compute the gravity loading on manipulator joints

    taug = gravload(robot, q)
    taug = gravload(robot, q, grav)

    Compute the joint gravity loading for the manipulator C{robot} in the
    configuration C{q}.

    If C{q} is a row vector, the result is a row vector of joint torques.
    If C{q} is a matrix, each row is interpretted as a joint state vector, and
    the result is a matrix each row being the corresponding joint torques.

    Gravity vector can be given explicitly using the named gravity keyword, otherwise
    it defaults to the value of the C{robot} object.

    @type q: M{m x n} matrix
    @type q: Joint coordinate
    @type grav: 3-vector
    @type grav: Gravitation acceleration, overrides C{robot} (optional)
    @type robot: Robot object, n-axes
    @param robot: The robot
    @rtype: M{m x n} matrix
    @return: Gravity torque
    @see: L{rne}, L{robot}
    """
    q = mat(q)
    if numcols(q) != robot.n:
        raise 'Insufficient columns in q'
    if gravity == None:
        tg = rne(robot, q, zeros(shape(q)), zeros(shape(q)))
    else:
        tg = rne(robot, q, zeros(shape(q)), zeros(shape(q)), gravity=gravity)
    return tg


def itorque(robot, q, qdd):
    """
    Compute the manipulator inertia torque

	    tau = itorque(robot, q, qdd)

    Returns the n-element inertia torque vector at the specified pose and
    acceleration, that is,

        taui = inertia(q)*qdd

    C{robot} describes the manipulator dynamics and kinematics.
    If C{q} and C{qdd} are row vectors, the result is a row vector of joint
    torques.
    If C{q} and C{qdd} are matrices, each row is interpretted as a joint state
    vector, and the result is a matrix each row being the corresponding joint
    torques.

    If C{robot} contains non-zero motor inertia then this will included in the
    result.

    @see: rne, coriolis, inertia, gravload.
    """

    return rne(robot, q, zeros(shape(q)), qdd, gravity=zeros(3))


def ospace(robot, q, qd):
    """
    Return Operational Space dynamic matrices

        (Lambda, mu, p) = ospace(robot, q, qd)
    @see: rne
    @bug: not tested
    """
    q = mat(q)
    qd = mat(qd)
    M = inertia(robot, q)
    C = coriolis(robot, q, qd)
    g = gravload(robot, q)
    J = Jac.jacob0(robot, q)
    Ji = inv(J)
    print("Ji\n%s\n\n" % Ji)
    print("J\n%s\n\n" % J)
    print("M\n%s\n\n" % M)
    print("C\n%s\n\n" % C)
    print("g\n%s\n\n" % g)
    Lambda = Ji.T*M*Ji
    mu = J.T*C - Lamba*H*qd
    p = J.T*g
    return Lambda, mu, p


def rne(robot, *args, **options):
    """
    Compute inverse dynamics via recursive Newton-Euler formulation.

    tau = rne(robot, q, qd, qdd)
    tau = rne(robot, [q qd qdd])

    Returns the joint torque required to achieve the specified joint position,
    velocity and acceleration state.

    Options
    =======

        One or more options can be provided as named arguments.

        Gravity
        -------
        Gravity vector is an attribute of the robot object but this may be
        overriden by providing a gravity acceleration vector [gx gy gz] using the
        named argument gravity

        tau = rne(robot, ..., gravity=[gx, gy, gz])


        External force/moment
        ---------------------
        An external force/moment acting on the end of the manipulator may also be
        specified by a 6-element vector [Fx Fy Fz Mx My Mz].

        tau = rne(robot, ..., fext=[fx,fy,fz, mx,my,mz])

        where Q, QD and QDD are row vectors of the manipulator state; pos, vel,
        and accel.

        Debug
        -----

        tau = rne(robot, ..., debug=n)

        Use the debug named argument to enable
            - 0 no messages
            - 1 display results of forward and backward recursions
            - 2 display print R and p*

    @see: L{robot}, L{accel}, L{inertia}, L{coriolis}, L{gravload}
    @note: verified against Matlab toolbox
    """

    n = robot.n

    a1 = mat(args[0])
    if numcols(a1) == 3*n:
        # single state parameter: [q qd qdd]
        q = a1[:,0:n]
        qd = a1[:,n:2*n]
        qdd = a1[:,2*n:3*n]
    else:
        # three state parameters: q, qd, qdd
        np = numrows(a1)
        q = a1
        qd = mat(args[1])
        qdd = mat(args[2])
        if numcols(a1) != n or numcols(qd) != n or numcols(qdd) != n or numrows(qd) != np or numrows(qdd) != np:
            error('inconsistant sizes of q, qd, qdd')

    # process options: gravity,  fext, debug
    debug = 0;
    gravity = robot.gravity;
    fext = mat(zeros((6,1))).T

    for k,v in options.items():
        if k == "gravity":
            gravity = arg2array(v);
            if not isvec(gravity, 3):
                error('gravity must be 3-vector')
            gravity = mat(gravity).T
        elif k == "fext":
            fext = arg2array(v);
            if not isvec(fext, 6):
                error('fext must be 6-vector')
            fext = mat(fext).T
        elif k == "debug":
            debug = v
        else:
            error('unknown option')

    if robot.ismdh():
        tau = _rne_mdh(robot, q, qd, qdd, gravity, fext, debug=debug)
    else:
        tau = _rne_dh(robot, q, qd, qdd, gravity, fext, debug=debug)
    return tau


def _rne_dh(robot, Q, Qd, Qdd, grav, fext, debug=0):

    z0 = mat([0,0,1]).T
    n = robot.n
    np = numrows(Q)
    tau = mat(zeros((np,n)))

    for p in range(0,np):
        q = Q[p,:].T
        qd = Qd[p,:].T
        qdd = Qdd[p,:].T

        Fm = []
        Nm = []
        pstarm = []
        Rm = []
        
        # rotate base velocity and acceleration into L1 frame
        Rb = t2r(robot.base).T;
        w = Rb*mat(zeros((3,1)))
        wd = Rb*mat(zeros((3,1)))
        v = Rb*mat(zeros((3,1)))
        vd = Rb*grav[:]

        #
        # init some variables, compute the link rotation matrices
        #
        for j in range(0,n):
            link = robot.links[j]
            Tj = link.tr(q[j,0])
            Rm.append(t2r(Tj))
            if link.sigma == 0:
                D = link.D
            else:
                D = q[j,0]
            alpha = link.alpha
            pstar = mat([[link.A],[D*sin(alpha)],[D*cos(alpha)]])
            
            pstarm.append(pstar)

            if debug > 1:
                print("Rm:\n%s" % Rm[j])
                print("pstarm:\n%s" % pstarm[j].T)

        #
        # the forward recursion
        #
        for j in range(0,n):
            link = robot.links[j]
            Rt = Rm[j].T   # transpose!!
            pstar = pstarm[j]
            r = link.r

            #
            # statement order is important here
            #
            
            if link.sigma == 0:
                # revolute axis
                wd = Rt*(wd + z0*qdd[j,0]+crossp(w,z0*qd[j,0]))
                w = Rt*(w + z0*qd[j,0])
                #v = crossp(w,pstar) + Rt*v
                vd = crossp(wd,pstar) + crossp(w,crossp(w,pstar)) + Rt*vd

            else:
                # prismatic axis
                w = Rt*w
                wd = Rt*wd
                vd = Rt*(z0*qdd[j,0]+vd) + \
                     crossp(wd,pstar) + 2*crossp(w,Rt*z0*qd[j,0]) + crossp(w,crossp(w,pstar))

            vhat = crossp(wd,r.T) + crossp(w,crossp(w,r.T)) + vd
            F = link.m*vhat
            N = link.I*wd + crossp(w,link.I*w)
            Fm.append(F)
            Nm.append(N)

            if debug:
                print("")
                print("w:\t%f\t%f\t%f"%(w[0,0], w[1,0], w[2,0]))
                print("wd:\t%f\t%f\t%f"%(wd[0,0], wd[1,0], wd[2,0]))
                print("vd:\t%f\t%f\t%f"%(vd[0,0], vd[1,0], vd[2,0]))
                print("vdbar:\t%f\t%f\t%f"%(vhat[0,0], vhat[1,0], vhat[2,0]))
                print("")

        #
        # the backward recursion
        #

        fext = fext.flatten().T
        f = fext[0:3,0]           # force/moments on end of arm
        nn = fext[3:6,0]

        for j in range(n-1,-1,-1):
            link = robot.links[j]
            pstar = pstarm[j]

            #
            # order of these statements is important, since both
            # mm and f are functions of previous f.
            #

            if j == n-1:
                R = mat(eye(3,3))
            else:
                R = Rm[j+1]
            if debug : print("\n%1d R: %s" % (j,R))
            r = link.r.T
            nn = R*(nn + crossp(R.T*pstar,f)) + crossp(pstar+r,Fm[j]) + Nm[j]
            f = R*f + Fm[j]
            if debug:
                print("")
                print("f:\t%f\t%f\t%f"%(f[0,0],f[1,0],f[2,0]))
                print("nn:\t%f\t%f\t%f"%(nn[0,0],nn[1,0],nn[2,0]))
                print("")

            R = Rm[j]
            if debug : print("Rm[%1d]: %s" % (j,R))
            if link.sigma == 0:
                # revolute
                tau[p,j] = nn.T*(R.T*z0) + link.G**2*link.Jm*qdd[j,0] + link.friction(qd[j,0])
            else:
                # prismatic
                tau[p,j] = f.T*(R.T*z0) + link.G**2*link.Jm*qdd[j,0] + link.friction(qd[j,0])
            if debug:
                print("")
                print("t: %s" % t)
                print("")

        # this last bit needs work/testing
        R = Rm[1];
        nn = R*(nn);
        f = R*f;
        af = array([f.T.flatten()]).flatten()
        ann = array([nn.T.flatten()]).flatten()
        wbase = mat(array([af , ann])).T
            
    return tau


def _rne_mdh(robot, Q, Qd, Qdd, grav, fext, debug=0):

    z0 = mat([0,0,1]).T
    n = robot.n
    np = numrows(Q)
    tau = mat(zeros((np,n)))

    for p in range(0,np):
        q = Q[p,:].T
        qd = Qd[p,:].T
        qdd = Qdd[p,:].T

        Fm = []
        Nm = []
        Pm = []
        Rm = []
        w = mat(zeros((3,1)))
        wd = mat(zeros((3,1)))
        v = mat(zeros((3,1)))
        vd = grav.flatten().T

        #
        # init some variables, compute the link rotation matrices
        #
        for j in range(0,n):
            link = robot.links[j]
            Tj = link.tr(q[j,0])
            Rm.append(t2r(Tj))
            if link.sigma == 0:
                D = link.D
            else:
                D = q[j,0]
            alpha = link.alpha
            Pm.append(mat([[link.A],[-D*sin(alpha)],[D*cos(alpha)]])) # (i-1) P i
            if debug > 1:
                print("Rm:\n%s" % Rm[j])
                print("Pm:\n%s" % Pm[j].T)

        #
        # the forward recursion
        #
        for j in range(0,n):
            link = robot.links[j]
            R = Rm[j].T   # transpose!!
            P = Pm[j]
            Pc = link.r

            #
            # trailing underscore means new value
            #

            if link.sigma == 0:
                # revolute axis
                w_ = R*w + z0*qd[j,0]
                wd_ = R*wd + crossp(R*w,z0*qd[j,0]) + z0*qdd[j,0]
                #v = crossp(w,P) + R*v
                vd_ = R*(crossp(wd,P) + crossp(w,crossp(w,P)) + vd)

            else:
                # prismatic axis
                w_ = R*w
                wd_ = R*wd
                #v = R*(z0*qd[j,0] + v) + crossp(w,P)
                vd_ = R*(crossp(wd,P) + crossp(w,crossp(w,P)) + vd) + 2*crossp(R*w,z0*qd[j,0]) + z0*qdd[j,0]

            # update variables
            w = w_
            wd = wd_
            vd = vd_

            vdC = crossp(wd,Pc) + crossp(w,crossp(w,Pc)) + vd
            F = link.m*vdC
            N = link.I*wd + crossp(w,link.I*w)
            Fm.append(F)
            Nm.append(N)

            if debug:
                print("")
                print("w:\t%f\t%f\t%f"%(w[0,0], w[1,0], w[2,0]))
                print("wd:\t%f\t%f\t%f"%(wd[0,0], wd[1,0], wd[2,0]))
                print("vd:\t%f\t%f\t%f"%(vd[0,0], vd[1,0], vd[2,0]))
                print("vdbar:\t%f\t%f\t%f"%(vdC[0,0], vdC[1,0], vdC[2,0]))
                print("")

        #
        # The backward recursion
        #

        fext = fext.flatten().T
        f = fext[0:3,0]           # force/moments on end of arm
        nn = fext[3:6,0]

        for j in range(n-1,-1,-1):
            link = robot.links[j]

            #
            # order of these statements is important, since both
            # mm and f are functions of previous f.
            #

            if j == n-1:
                R = mat(eye(3,3))
                P = mat([[0],[0],[0]])
            else:
                R = Rm[j+1]
                P = Pm[j+1]   # i/P/(i+1)
            Pc = link.r
            f_ = R*f + Fm[j]
            nn_ = Nm[j] + R*nn + crossp(Pc,Fm[j]) + crossp(P,R*f)
            f = f_
            nn = nn_
            if debug:
                print("")
                print("f:\t%f\t%f\t%f"%(f[0,0],f[1,0],f[2,0]))
                print("nn:\t%f\t%f\t%f"%(nn[0,0],nn[1,0],nn[2,0]))
                print("")

            if link.sigma == 0:
                # revolute
                tau[p,j] = nn.T*z0 + link.G**2*link.Jm*qdd[j,0] - link.friction(qd[j,0])
            else:
                # prismatic
                tau[p,j] = f.T*z0 + link.G**2*link.Jm*qdd[j,0] - link.friction(qd[j,0])
    return tau


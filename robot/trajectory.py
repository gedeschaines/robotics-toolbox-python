"""
Trajectory primitives.

Python implementation by: Luis Fernando Lara Tobar and Peter Corke.
Based on original Robotics Toolbox for Matlab code by Peter Corke.
Permission to use and copy is granted provided that acknowledgement
of the authors is made.

@author: Luis Fernando Lara Tobar and Peter Corke
"""

from numpy import *
from pylab import *
from robot.utility import *
from robot import transform as T
from inspect import isfunction


def jtraj(q0, q1, tv, qd0=None, qd1=None):
    """
    Compute a joint space trajectory between points C{q0} and C{q1}.
    The number of points is the length of the given time vector C{tv}.
    If {tv} is a scalar, it is taken as the number of points.

    A 7th order polynomial is used with default zero boundary conditions
    for velocity and acceleration.  Non-zero boundary velocities can be
    optionally specified as C{qd0} and C{qd1}.

    As well as the trajectory, M{q{t}}, its first and second derivatives
    M{qd(t)} and M{qdd(t)} are also computed.  All three are returned as
    a tuple. Each of these is an M{m x n} matrix, with one row per time
    step, and one column per joint parameter.

    @type q0: m-vector
    @param q0: initial state
    @type q1: m-vector
    @param q1: final state
    @type tv: n-vector or scalar
    @param tv: time step vector or number of steps
    @type qd0: m-vector
    @param qd0: initial velocity (default 0)
    @type qd1: m-vector
    @param qd1: final velocity (default 0)
    @rtype: tuple
    @return: (q, qd, qdd), a tuple of M{m x n} matrices
    @see: L{ctraj}

    """
    if isinstance(tv,(int,int32,float,float64)):
        tscal = float(1)
        tv = float(tv)                  # ensures t will be array of floats
        t = mat(arange(0,tv)).T/(tv-1)  # normalized tv in range [0.0, 1.0)
    else:
        tv = arg2array(tv);
        tscal = float(max(tv))  # ensures t will be array of floats
        t = mat(tv).T / tscal   # normalized tv in range [0.0, 1.0]

    q0 = arg2array(q0)
    q1 = arg2array(q1)

    if qd0 is None:
        qd0 = zeros((shape(q0)))
    else:
        qd0 = arg2array(qd0);
    if qd1 is None:
        qd1 = zeros((shape(q1)))
    else:
        qd1 = arg2array(qd1)

    # compute the polynomial coefficients
    A = 6*(q1 - q0) - 3*(qd1 + qd0)*tscal
    B = -15*(q1 - q0) + (8*qd0 + 7*qd1)*tscal
    C = 10*(q1 - q0) - (6*qd0 + 4*qd1)*tscal
    E = qd0*tscal # as the t vector has been normalized
    F = q0

    tt = concatenate((power(t,5),power(t,4),power(t,3),power(t,2),t,ones(shape(t))),1)
    c = vstack((A, B, C, zeros(shape(A)), E, F))
    qt = tt * c

    # compute velocity
    c = vstack((zeros(shape(A)),5*A,4*B,3*C,zeros(shape(A)),E))
    qdt = tt * c / tscal

    # compute acceleration
    c = vstack((zeros(shape(A)),zeros(shape(A)),20*A,12*B,6*C,zeros(shape(A))))
    qddt = tt * c / (tscal**2)

    return qt, qdt, qddt


def ctraj(t0, t1, r):
    """
    Cartesian trajectory between two points.
    
    Compute a Cartesian trajectory between poses C{t0} and C{t1}.
    The number of points is the length of the path distance vector C{r}.
    Each element of C{r} gives the distance along the path, and their
    values must be in the range [0 1].

    If {r} is a scalar it is taken as the number of points, and the points
    are equally spaced between C{t0} and C{t1}.
    
    traj = ctraj(t0, t1, N) is a Cartesian trajectory (Nx4x4) from pose t0 to t1
    with N points that follow a trapezoidal velocity profile along the path.
    The Cartesian trajectory is a homogeneous transform sequence and the first 
    subscript being the point index, that is, T(i,:,:) is the i'th point along
    the path.

    traj = ctraj(t0, t1, R) as above but the elements of R (Nx1) specify the 
    fractional distance along the path, and these values are in the range [0 1].
    The i'th point corresponds to a distance R(i) along the path.
    
    The trajectory is a row matrix of homogeneous transform matrices.

    @type t0: homogeneous transform
    @param t0: initial pose
    @type t1: homogeneous transform
    @param t1: final pose
    @type r: an int or vector of floats
    @param r: number of steps from 0 to 1, or monotonically increasing values in range [0,1]
    @rtype: row matrix of M{4x4} matrices
    @return: Cartesian trajectory
    @see: L{lspb}, L{trinterp}, L{jtraj}
    """
    traj3Dmat = False;
    
    if isinstance(r, (int,int32,float,float64)):
        #i = arange(0.0,r+1)
        #r = i/float(r)
        [s,sd,sdd] = lspb(0.0, 1.0, r)
    else:
        s = arg2array(r)
    
    if any(s>1.0) or any(s<0.0):
        error('path position values (R) must 0<=R<=1')
    
    if traj3Dmat:
        # allocate 3D matrix for trajectory, each row holding a 2D transform matrix
        (nr,nc) = mat(t0).shape
        traj = zeros((len(s),nr,nc))
        # generate Cartesian trajectory, starting at t0 and finishing at t1
        traj[0,:,:] = t0[:,:]                      # Note: MatLab stores the
        for i in range(1,len(s)):                  #       the sequence of 2D
            t = T.trinterp(t0, t1, float(s[i]))    #       transform matrices as 
            traj[i,:,:] = t[:,:]                   #       traj[:,:,i] = t[:,:]
    else:
        # generate sequence of 2D transform matrices from t0 to t1; store in a list
        traj = []
        for i in range(0,len(s)):
            traj.append( T.trinterp(t0, t1, float(s[i])) )
            
    return traj


def lspb(q0, qf, MorT, *args):
    """
    Linear segment s with parabolic blend.

    [s, sd, sdd] = lspb(q0, qf, M) is a scalar trajectory (Mx1) that varies 
    smoothly from q0 to qf in M steps using a constant velocity segment and 
    parabolic blends (a trapezoidal path). Velocity and acceleration can be
    optionally returned as sd (Mx1) and sdd (Mx1).
 
    [s, sd, sdd] = lspb(q0, qf, M, V) as above, but specifies the velocity 
    of the linear segment which is normally computed automatically.

    [s, sd, sdd] = lspb(q0, qf, T) as above but specifies the trajectory in 
    terms of the length of the time vector T (Mx1).

    [s, sd, sdd] = lsbp(q0, qf, T, V) as above but specifies the velocity of 
    the linear segment which is normally computed automatically and a time
    vector T (Mx1).
    
    Notes::
    - Optional arguments are qd0, qdf and Plot.
    - Set Plot True to display plots of s, sd and sdd.

    @type q0: float
    @param q0: initial joint position
    @type qf: float
    @param qf: final joint position
    @type MorT: an int or vector of floats
    @param MorT: number of time steps, or array of monotonically increasing time values
    @type V: None or float
    @param V: optional linear segment velocity
    @type Plot: None or Boolean
    @param Plot: optional plots of s, sd and sdd
    @rtype: three Mx1 vectors
    @return: joint position (s), velocity (sd) and acceleration (sdd)
    @see: L{tpoly}, L{jtraj}, L{mtraj}
    """
    t0 = MorT
    if shape(MorT) == ():
        t_scalar = True
        t = arange(0.0,MorT)
    else:
        t_scalar = False
        t = MorT.flatten()
        
    if q0 == qf:
        p = ones((len(t),1)) * q0
        pd = zeros((len(t),1))
        pdd = zeros((len(t),1))
        
        s = mat(p)
        sd = mat(pd)
        sdd = mat(pdd)
        
        return s, sd, sdd
        
    tf = max(t)
    
    if args is None or len(args) == 0:
        V = None;    Plot = False
    elif len(args) == 1:
        V = args[0]; Plot = False
    else:
        V = args[0]; Plot = args[1]
        
    if V is None:
        V = (qf-q0)/tf * 1.5
    else:
        V = abs(V) * sign(qf-q0)
        if abs(V) < abs(qf-q0)/tf:
            error('V too small\n')
        elif abs(V) > 2*abs(qf-q0)/tf:
            error('V too big\n')

    tb = (q0 - qf + V*tf)/V
    a = V/tb
    
    p = zeros((len(t),1))
    pd = zeros((len(t),1))
    pdd = zeros((len(t),1))
    
    for i in range(0,len(t)):
        ti = t[i]

        if ti <= tb:
            # initial blend
            p[i] = q0 + a/2*ti**2
            pd[i] = a*ti
            pdd[i] = a
        elif ti <= (tf-tb):
            # linear motion
            p[i] = (qf+q0-V*tf)/2 + V*ti
            pd[i] = V
            pdd[i] = 0
        else:
            # final blend
            p[i] = qf - a/2*tf**2 + a*tf*ti - a/2*ti**2
            pd[i] = a*tf - a*ti
            pdd[i] = -a

    if Plot:
        if t_scalar:
           # for scalar time steps, axis is labeled 1 .. M
           xt = t+1
        else:
           # for vector time steps, axis is labeled by vector T
           xt = t

        clf()
        subplot(311)
        # highlight the accel, coast, decel phases with different
        # colored markers
        hold(True)
        k = xt <= tb
        plot(xt[k], p[k], 'r-o')
        k = (xt >= tb) & (xt <= (tf-tb))
        plot(xt[k], p[k], 'b-o')
        k = xt >= (tf-tb)
        plot(xt[k], p[k], 'g-o')
        grid
        ylabel('s')
        hold(False)

        subplot(312)
        plot(xt, pd); grid; ylabel('sd')
            
        subplot(313)
        plot(xt, pdd); grid; ylabel('sdd')
        
        if not t_scalar:
            xlabel('time')
        else:
            for c in get(gcf, 'Children'):
                set(c, 'XLim', [1, t0])
        show()
        
    s = mat(p)
    sd = mat(pd)
    sdd = mat(pdd)

    return s, sd, sdd
 

def tpoly(q0, qf, MorT, *args):
    """
    Generate scalar polynomial trajectory s.
 
    [s, sd, sdd] = tpoly(q0, qf, M) is a scalar trajectory (Mx1) that varies 
    smoothly from q0 to qf in M steps using a quintic (5th order) polynomial.
    Velocity and acceleration can be optionally returned as (Mx1) arrays sd
    and sdd respectively.
 
    [s, sd, sdd] = tpoly(q0, qf, T) as above, but specifies the trajectory in 
    terms of the length of the time vector T (Mx1).

    [s, sd, sdd] = tpoly(q0, qf, M, qd0, qdf) as above, but specifies initial 
    and final joint velocity for the trajectory.
    
    [s, sd, sdd] = tpoly(q0, qf, T, qd0, qdf) as above, but specifies time
    vector T with initial and final joint velocity for the trajectory.
   
    Notes::
    - Optional arguments are qd0, qdf and Plot.
    - Set Plot True to display plots of s, sd and sdd.

    @type q0: float
    @param q0: initial joint position
    @type qf: float
    @param qf: final joint position
    @type MorT: an int or vector of floats
    @param MorT: number of time steps, or array of monotonically increasing time values
    @type qd0: None or float
    @param qd0: optional initial joint velocity
    @type qdf: None or float
    @param qdf: optional final joint velocity
    @type Plot: None or Boolean
    @param Plot: optional plots of s, sd and sdd
    @rtype: three Mx1 vectors
    @return: joint position (s), velocity (sd) and acceleration (sdd)
    @see: L{lspb}, L{jtraj}, L{mtraj}
    """
    
    t0 = MorT
    if shape(MorT) == ():
        t_scalar = True
        t = arange(0.0,MorT)
    else:
        t_scalar = False
        t = MorT.flatten()
        
    if args is None or len(args) == 0:
        qd0 = 0.0;     qdf = 0.0;     Plot = False
    elif len(args) == 1:
        qd0 = args[0]; qdf = 0.0;     Plot = False
    elif len(args) == 2:
        qd0 = args[0]; qdf = args[1]; Plot = False
    else:
        qd0 = args[0]; qdf = args[1]; Plot = args[2]
    
    if qd0 is None: qd0 = 0.0
    if qdf is None: qdf = 0.0
    
    tf = max(t)
    
    # solve for the polynomial coefficients using least squares
    X = array([[0,           0,           0,         0,       0,   1],
               [tf**5,       tf**4,       tf**3,     tf**2,   tf,  1],
               [0,           0,           0,         0,       1,   0],
               [5*tf**4,     4*tf**3,     3*tf**2,   2*tf,    1,   0],
               [0,           0,           0,         2,       0,   0],
               [20*tf**3,    12*tf**2,    6*tf,      2,       0,   0]])
    coeffs = asarray(linalg.solve(X, array([q0, qf, qd0, qdf, 0, 0]).T).T)
    
    # coefficients of derivatives
    c = array([5, 4, 3, 2, 1])
    coeffs_d = coeffs[0:5] * c[0:5]
    coeffs_dd = coeffs_d[0:4] * c[1:5]

    # evaluate the polynomials
    p = polyval(coeffs, t)
    pd = polyval(coeffs_d, t)
    pdd = polyval(coeffs_dd, t)

    if Plot:
        if t_scalar:
            # for scalar time steps, axis is labeled 1 .. M
            xt = t+1
        else:
            # for vector time steps, axis is labeled by vector T
            xt = t

        clf()
        subplot(311)
        plot(xt, p); grid; ylabel('s')

        subplot(312)
        plot(xt, pd); grid; ylabel('sd')
            
        subplot(313)
        plot(xt, pdd); grid; ylabel('sdd')
        
        if not t_scalar:
            xlabel('time')
        else:
            for c in get(gcf, 'Children'):
                set(c, 'XLim', [1,t0])
        show()
        
    s = mat(p).T
    sd = mat(pd).T
    sdd = mat(pdd).T
            
    return s, sd, sdd


def mtraj(tfunc, q0, qf, MorT, *args):
    """
    Multi-axis trajectory Q between two points.
 
    [Q, Qd, Qdd] = mtraj(tfunc, q0, qF, M) is a multi-axis trajectory (MxN) 
    varying from state q0 (1xN) to qF (1xN) according to the scalar trajectory 
    function tfunc in M steps. Joint velocity and acceleration can be optionally 
    returned as Qd (MxN) and Qdd (MxN) respectively. The trajectory outputs have
    one row per time step, and one column per axis.
    
    [Q, Qd, Qdd] = mtraj(tfunc, q0, qF, T) as above but specifies the trajectory 
    length in terms of the length of the time vector T (Mx1).
    
    The shape of the trajectory is given by the scalar trajectory function tfunc
    
      [s, sd, sdd] = tfunc(q0, qf, MorT, *args)
      
    and possible values of tfunc include lspb for a trapezoidal trajectory, or
    tpoly for a polynomial trajectory.

    Notes::
    - If the last optional argument is True, then Q, QD, and QDD are plotted.
    - When tfunc is tpoly, the result is functionally equivalent to jtraj except 
      that no initial velocities can be specified. JTRAJ is computationally a 
      little more efficient.
   
    @type q0: float 1xN array
    @param q0: initial joint positions
    @type qf: float 1xN array
    @param qf: final joint positions
    @type MorT: an int or vector of floats
    @param MorT: number of time steps, or array of monotonically increasing time values
    
    if tfunc is lspb then args may be
    @type V: None or float
    @param V: optional linear segment velocity
    @type Plot: None or Boolean
    @param Plot: optional plots of Q, Qd and Qdd
    
    if tfunc is tpoly, then args may be
    @type qd0: None or float
    @param qd0: optional initial joint velocity
    @type qdf: None or float
    @param qdf: optional final joint velocity
    @type Plot: None or Boolean
    @param Plot: optional plots of s, sd and sdd
       
    @rtype: three MxN vectors
    @return: joint positions (Q), velocities (Qd) and accelerations (Qdd)
    @see: L{jtraj}, L{lspb}, L{tpoly}
    """

    if not isfunction(tfunc):
        error('first argument must be a function handle')
        
    M0 = MorT
    if shape(M0) == ():
        M_scalar = True
        M = MorT
    else:
        M_scalar = False
        M = len(MorT)
        
    if len(q0) != len(qf):
        error('q0 and qf must be same length')
     
    if args is None or len(args) == 0:
        Plot = False
    elif args[-1] is True:
        Plot = True
    else:
        Plot = False

    Q = mat(zeros((M, len(q0))))
    Qd = mat(zeros((M, len(q0))))
    Qdd = mat(zeros((M, len(q0))))

    for i in range(0, len(q0)):
        # for each axis
        [s, sd, sdd] = tfunc(q0[i], qf[i], MorT, *args)
        Q[:,i] = s[:]
        Qd[:,i] = sd[:]
        Qdd[:,i] = sdd[:]
        
    if Plot:
        if M_scalar:
            t = range(1, M0+1)
        else:
            t = MorT.flatten()
            
        clf()
        subplot(311)
        plot(t, Q); grid; ylabel('q')

        subplot(312)
        plot(t, Qd); grid; ylabel('qd')
            
        subplot(313)
        plot(t, Qdd); grid; ylabel('qdd')
        
        if not M_scalar:
            xlabel('time')
        else:
           for c in get(gcf, 'Children'):
               set(c, 'XLim', [1, M0])
        show()
            
    return Q, Qd, Qdd
    

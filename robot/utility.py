"""
Python toolbox utility and helper functions.


Python implementation by: Luis Fernando Lara Tobar and Peter Corke.
Based on original Robotics Toolbox for Matlab code by Peter Corke.
Permission to use and copy is granted provided that acknowledgement
of the authors is made.

@author: Luis Fernando Lara Tobar and Peter Corke
"""

from numpy import *
import time


################ Matlab tic/toc timer

global t_bgn
t_bgn = time.time()

def tic():
    global t_bgn
    t_beg = time.time()
    return

def toc():
    global t_beg
    t_end = time.time()
    return 'Elapsed time is %.5f seconds.' % (t_end - t_bgn)

################ data shape queries

def isempty(v):
    if v is None:
        return True
    try:
        return len(v) == 0
    except TypeError:
        return False
        
def ishomog(tr):
    """
    True if C{tr} is a 4x4 homogeneous transform.

    @note: Only the dimensions are tested, not whether the rotation
    submatrix is orthonormal.

    @rtype: boolean
    """
    if tr is None:
        return False
    try:
        size = tr.shape
        return size == (4,4)
    except AttributeError:
        return False

def isrot(r):
    """
    True if C{r} is a 3x3 matrix.

    @note: Only the dimensions are tested, not whether the matrix
    is orthonormal.

    @rtype: boolean
    """
    if r is None:
        return False
    try:
        size = r.shape
        return size == (3,3)
    except AttributeError:
        return False

def isvec(v, l=3):
    """
    True if C{v} is an l-vector.

    @param v: object to test
    @type l: integer
    @param l: length of vector (default 3)

    @rtype: boolean
    """
    if v is None:
        return False
    try:
        size = v.shape
        return size == (l,1) or size == (1,l) or size == (l,)
    except AttributeError:
        return False

def numcols(m):
    """
    Number of columns in a matrix.

    @type m: matrix
    @return: the number of columns in the matrix.
    return m.shape[1];
    """
    return m.shape[1];

def numrows(m):
    """
    Number of rows in a matrix.

    @type m: matrix
    @return: the number of rows in the matrix.
    return m.shape[1];
    """
    return m.shape[0];


################ vector operations

def unit(v):
    """
    Unit vector.

    @type v: vector
    @rtype: vector
    @return: unit-vector parallel to C{v}
    """
    return mat(v / linalg.norm(v))


def crossp(v1, v2):
    """
    Vector cross product.

    @note Differs from L{numpy.cross} in that vectors can be row or
    column.

    @type v1: 3-vector
    @type v2: 3-vector
    @rtype: 3-vector
    @return: Cross product M{v1 x v2}
    """
    v1=mat(v1)
    v2=mat(v2)
    v1=v1.reshape(3,1)
    v2=v2.reshape(3,1)
    v = matrix(zeros( (3,1) ))
    v[0] = v1[1]*v2[2] - v1[2]*v2[1]
    v[1] = v1[2]*v2[0] - v1[0]*v2[2]
    v[2] = v1[0]*v2[1] - v1[1]*v2[0]
    return v


################ misc support functions

def arg2array(arg):
    """
    Convert a 1-dimensional argument that is either a list, array or matrix to an
    array.

    Useful for functions where the argument might be in any of these formats:::
            func(a)
            func(1,2,3)

            def func(*args):
                if len(args) == 1:
                    v = arg2array(arg[0]);
                elif len(args) == 3:
                    v = arg2array(args);
             .
             .
             .

    @rtype: array
    @return: Array equivalent to C{arg}.
    """
    if isinstance(arg, (matrix, ndarray)):
        s = arg.shape;
        if len(s) == 1:
            return array(arg);
        if min(s) == 1:
            return array(arg).flatten();

    elif isinstance(arg, list):
        return array(arg);

    elif isinstance(arg, (int, float, float32, float64)):
        return array([arg]);

    raise ValueError;

################ error and warning handlers 

import traceback;

def error(s):
    """
    Common error handler. Display the error string, execute a traceback then raise
    an execption to return to the interactive prompt.
    """
    print("Robotics toolbox error: %s" % s)

    #traceback.print_exc();
    raise ValueError()
    
    
def warning(sfnc,smsg):
    """
    Common warning handler. Display the warning strings - function and message.
    """
    print("%s: %s" % (sfnc, smsg))
    
    
if __name__ == "__main__":
    print("%s" % arg2array(1))
    print("%s" % arg2array(1.0))
    print("%s" % arg2array( mat([1,2,3,4]) ))
    print("%s" % arg2array( mat([1,2,3,4]).T ))
    print("%s" % arg2array( array([1,2,3]) ))
    print("%s" % arg2array( array([1,2,3]).T ))
    print("%s" % arg2array( [1,2,3]))


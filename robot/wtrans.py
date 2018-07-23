#WTRANS Transform a wrench between coordinate frames
#
# WT = WTRANS(T, W) is a wrench (6x1) in the frame represented by the
# homogeneous transform T (4x4) corresponding to the world frame wrench
# W (6x1).
#
# The wrenches W and WT are 6-vectors of the form [Fx Fy Fz Mx My Mz].
#
# See also TR2DELTA, TR2JAC.

# Copyright (C) 1993-2011, by Peter I. Corke
#
# This file is part of The Robotics Toolbox for Matlab (RTB).
#
# RTB is free software: you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# RTB is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
# the GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with RTB. If not, see <http://www.gnu.org/licenses/>.

from transform import transl, t2r
from utility import *

def wtrans(T, W):
    """
    Apply homogeneous transform T on wrench W.
    Although W may be an array or matrix, special attention
    is given to ensure operands in the vector math operations
    are all matrices.

    @type T: 4x4 NumPy matrix
    @parma T: homogeneous transform matrix
    @type W : 1x6 or 6x1 vector as an array or matrix
    @param W: wrench vector
    @rtype Wt: 1x6 or 6x1 vector as an array or matrix
    @return Wt: transformed wrench
    """
    # coerce W into a 1x6 flattened array, but remember its
    # shape and type so as to be returned as same.
    if isvec(W, l=6):
        if W.shape == (6,):     # already flattened array
            W_isrow = True
            W_ismatrix = False
        elif W.shape == (1,6):  # 1x6 array or matrix
            W_isrow = True
            W_ismatrix = isinstance(W, matrix)
            W = array(W).flatten()
        else:                   # 6x1 array or matrix
            W_isrow = False
            W_ismatrix = isinstance(W, matrix)
            W = array(W.T).flatten()
    elif isinstance(W, list) and len(W) == 6:
        W_isrow = True
        W_ismatrix = False
        W = arg2array(W)
    else:
        raise ValueError

    f = mat(W[0:3]).T  # coerce to [3,1] vector
    m = mat(W[3:6]).T  # coerce to [3,1] vector
    # add moment due to translated force component
    k = crossp(f, transl(T)) + m
    # apply rotation to force and moment components
    ft = t2r(T).T * f
    mt = t2r(T).T * k
    # pack force and moment components into wrench vector
    Wt = vstack((ft, mt))
    if W_isrow:
        Wt = Wt.flatten()
    if not W_ismatrix:
        Wt = array(Wt)
    return Wt

# Robotics Toolbox for Python
# Copyright (C) 1993-2013, by Peter I. Corke
#
# These files are part of The Robotics Toolbox for MATLAB (RTB).
#
# RTB is free software: you can redistribute it and/or modify it under
# the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# RTB is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
# the GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# (LICENSE-LGPLv3) along with RTB. If not, see <http://www.gnu.org/licenses/>.
#
# http://www.petercorke.com

__doc__ = """
The Robotics Toolbox for Python
Based on the Matlab version
Peter Corke 2007
"""

__info__ = """
Robotics Toolbox for Python
Based on Matlab Toolbox Version 7  April-2002

What's new.
  Readme      - New features and enhancements in this version.

Homogeneous transformations
  eul2tr      - Euler angle to transform
  oa2tr       - orientation and approach vector to transform
  rotx        - transform for rotation about X-axis
  roty        - transform for rotation about Y-axis
  rotz        - transform for rotation about Z-axis
  rpy2tr      - roll/pitch/yaw angles to transform
  tr2eul      - transform to Euler angles
  tr2rot      - transform to rotation submatrix
  tr2rpy      - transform to roll/pitch/yaw angles
  transl      - set or extract the translational component of a transform
  trnorm      - normalize a transform

Quaternions
  /           - divide quaternion by quaternion or scalar
  *           - multiply quaternion by a quaternion or vector
  inv         - invert a quaternion
  norm        - norm of a quaternion
  plot        - display a quaternion as a 3D rotation
  qinterp     - interpolate quaternions
  unit        - unitize a quaternion

Kinematics
  diff2tr     - differential motion vector to transform
  fkine       - compute forward kinematics
  ikine       - compute inverse kinematics
  ikine560    - compute inverse kinematics for Puma 560 like arm
  jacob0      - compute Jacobian in base coordinate frame
  jacobn      - compute Jacobian in end-effector coordinate frame
  tr2diff     - transform to differential motion vector
  tr2jac      - transform to Jacobian

Dynamics
  accel       - compute forward dynamics
  cinertia    - compute Cartesian manipulator inertia matrix
  coriolis    - compute centripetal/coriolis torque
  friction    - joint friction
  ftrans      - transform force/moment
  gravload    - compute gravity loading
  inertia     - compute manipulator inertia matrix
  itorque     - compute inertia torque
  nofriction  - remove friction from a robot object
  rne         - inverse dynamics

Trajectory generation
  ctraj       - Cartesian trajectory
  jtraj       - joint space trajectory
  trinterp    - interpolate transform s

Graphics
  drivebot    - drive a graphical  robot
  plot        - plot/animate robot

Other
  manipblty   - compute manipulability
  unit        - unitize a vector

Creation of robot models.
  link        - construct a robot link object
  puma560     - Puma 560 data
  puma560akb  - Puma 560 data (modified Denavit-Hartenberg)
  robot       - construct a robot object
  stanford    - Stanford arm data
  twolink     - simple 2-link example
"""

######################
#   Import Section   #
######################

from numpy import *

# Import Link Constructor section
from .Link import *

# Import Robot Constructor section
from .Robot import *

# utility
from .utility import *

# Import transformations section
from .transform import *

from .jacobian import *

# import kinematics section
from .kinematics import *

from .manipblty import *

# import trajectories section
from .trajectory import *

# import Quaternion constructor section
from .Quaternion import *

# import dynamics section
from .dynamics import *

# import plot section
from .plot import *

# import robot models sections
#
# These are imported on an as-needed basis.
#
# from puma560 import *
# from puma560akb import *
# from stanford import *
# from twolink import *


def pinfo():
    """
    Presents Robotics Toolbox for Python information.
    """
    print('%s' % __info__)

    
def disclaimer_rtb():
    """
    # Robotics Toolbox for Python
    # Copyright (C) 1993-2013, by Peter I. Corke
    #
    # These files are part of The Robotics Toolbox for MATLAB (RTB).
    #
    # RTB is free software: you can redistribute it and/or modify it under
    # the terms of the GNU Lesser General Public License as published by
    # the Free Software Foundation, either version 3 of the License, or
    # (at your option) any later version.
    #
    # RTB is distributed in the hope that it will be useful, but
    # WITHOUT ANY WARRANTY; without even the implied warranty of
    # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
    # the GNU Lesser General Public License for more details.
    #
    # You should have received a copy of the GNU Lesser General Public License
    # (LICENSE-LGPLv3) along with RTB. If not, see <http://www.gnu.org/licenses/>.
    #
    # http://www.petercorke.com
    """
    pass
    

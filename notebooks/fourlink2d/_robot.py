""" Imports Robotics Toolbox module

    This file is a temporary component of Robotics Toolbox for Python - Update1
    development effort used to facilitate execution of Python scripts and Jupyter
    notebooks from within an unpacked robotics-toolbox-python-update1 distribution
    directory work space. If the RTB for Python - Update1 robot module is installed
    then 'import _robot' statements in may be replaced with 'import robot as robot'.
"""

import os, sys

try:
    import robot as robot
except ImportError:
    thisdir = os.path.dirname(__file__)
    libdir = None
    
    # The following assumes this file is in a ./notebooks/example subdirectory
    # with the ./notebooks subdirectory adjacent to ./robot in an unpacked
    # robotics-toolbox-python-update1 distribution directory hierarchy on
    # an Ubuntu 14.04 x86_64 linux platform with Python 2.7 or Python 3.
    
    if sys.version_info.major == 2:
        libdir = os.path.join(thisdir, '../../build/lib.linux-x86_64-2.7')
    if sys.version_info.major == 3:
        libdir = os.path.join(thisdir, '../../build/lib')
    
    if libdir is not None:
        # first try to find robot module in build/lib directory
        if os.path.isdir(os.path.join(libdir,'/robot')):
            if libdir not in sys.path:
                print('Using Robotics Toolbox module %s/robot' % libdir)
                sys.path.insert(0, libdir)
        else:
            # now try robot module source directory
            libdir = os.path.join(thisdir, '../..')
            if os.path.isdir(os.path.join(libdir,'robot')):
                if libdir not in sys.path:
                    print('Using Robotics Toolbox module %s/robot' % libdir)
                    sys.path.insert(0, libdir)
            else:
                print("Could not locate Robotics Toolbox module.")
                sys.exit(1);
    else:
        print("Could not locate Robotics Toolbox module")
        sys.exit(1);
        

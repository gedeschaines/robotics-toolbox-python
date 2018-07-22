## robotics-toolbox-python-update1 ##
Robotics Toolbox for Python - Update 1

This is a 1st update on the 'old first attempt' at creating an open Python version of the Robotics Toolbox for MATLAB (RTB). Although the RTB for Python repository has languished over the past decade, there may still be some interest in its revival; even in its dated and rudimentary form. At this point, there is no intent to morph RTB for Python into another feature rich robot simulation tool extensively dependent on 3rd party modules and libraries. However, it should be noted that the present RTB for Python architecture relies exclusively on the 'pylab' convenience module, a practice not recommended by [**Matplotlib**](https://matplotlib.org/faq/usage_faq.html#matplotlib-pyplot-and-pylab-how-are-they-related) developers.

#### Getting Started ####

Download, unpack and run demos and tests as described in the README files, or optionally just build and install with the provided setup.py script (e.g., python setup.py install). Best use experience with RBT for Python is in IPython's interactive Pylab shell mode. For example, to display an animation of the Puma 560 (standard) robot transition from a nominal pose to a resting pose, invoke IPython in the demo or test directory and enter the six commands as shown below.

$ ipython --matplotlib

In \[1]: %pylab

In \[2]: import _robot

In \[3]: from robot import *

in \[4]: from robot.puma560 import *

In \[5]: (Q,\_,\_) = jtraj(qn,qr,51)

In \[6]: rbplot(p560, Q)

The above will generate an animation similiar to that shown in this animated gif converted from an mp4 video clip created with 'rec=1' passed as a third argument to rbplot() on input line \[6] above.

![Puma_560 animation recorded by rbplot()](./docs/Puma_560.gif)

#### Guidelines for Update 1 ####

1) Minimize changes to facilitate code comparison against sourced Python and MATLAB scripts.
2) Maintain the 'look and feel' of the original RTB functional interface.
4) Preserve content of function's descriptive comment blocks, but modify to reflect code changes and adhere to original epydoc docstring conventions.
5) Retain copyright, license and disclaimer notices.

#### Accomplishments for Update 1 ####

1) RTB for Python is now compatible with Python 2 and 3 variants.
2) Revitalized forward and inverse dynamics functionality.
3) Incorporated capability to display and record simple animated 2D and 3D plots of frame transforms and robot manipulator poses.
4) Extended functionality to run additional demo and test cases provided in RTB for Python and MATLAB distributions.
5) Updated inverse kinematics in RTB for Python to match RTB version 9.
6) Fixed several code errors, and made some minor improvements -- notably in parsedemo and testparser.
7) Corrected semantic and syntactic mistakes in some comments.
8) Removed insidious leading tabs and unnecessary terminating semi-colons from executable statements when encountered during code editing.

Although this effort is conducted on an Ubuntu 14.04 x86_64 Linux system with the Python, NumPy, Matplotlib and Octave versions identified below, the updated RTB for Python should work on non-Linux systems which support Python 2 or 3, and SciPy.
* Python 2.7.6, IPython 1.2.1, NumPy 1.8.2, Matplotlib 1.3.1
* Python 2.7.6, IPython 5.2.2, NumPy 1.12.0, Matplotlib 1.3.1
* Python 3.4.3, IPython 5.3.0, NumPy 1.14.5, Matplotlib 2.2.2
* Octave 4.0.2

While Octave is not required to use RTB for Python, it is being utilized to run RTB MATLAB demo and test scripts for comparison with updated RTB for Python demos and tests. Several MATLAB and Octave scripts in the RTB v9.8 distribution required modificaton to execute in Octave 4.0.2. The modified scripts are provided in the ./Octave/rvctools subdirectory of this distribution. As with the Octave scripts in the RTB distribution, simply copy the scripts to the corresponding ./rvctools subdirectories in the default or user specific Octave function search path.

#### Further Attention ####

* Non-convergent inverse kinematic iterations do not produce the same error norms among Python 2, 3 and Octave, which may indicate an implementation error in the ikine function or affects of possible differences in numerical precision of utilized linear algebra library functions.
* RTB for Python and Octave 4 currently use 'lsode' method to integrate differential equations of motion for manipulator dynamics, but ode45 is available for both and could be a user selectable feature.
* Encountered several instances where fundamental differences between MATLAB and Python array/matrix operations ([refer here](https://docs.scipy.org/doc/numpy/user/numpy-for-matlab-users.html)), type coercion and copy properties caused program errors in RBT for Python. Unidentified instances may still exist.
* Enhancements to robot pose graphics capabilities and features will require more sophisticated data structures for the Robot class (i.e., for the handle and plotopt attributes) and the introduction of a Plot class to eliminate plot module global variables and address redrawing event handling issues.
* RTB for Python module import statements require refinement to restrict overly broad inclusions of non-essential module elements and to address circular references.
* Organization of modules into an object-oriented design pattern would improve the usability and maintainablity of RTB for Python source code.
* Clamping joint movements in the ikine solver iteration loop is needed to produce realistic manipulator motion.

#### Attributions ####

* Luis Fernando Lara Tobar and Peter Corke for **Robotics Toolbox for Python** provided as the [robitics-toolbox-python](https://github.com/petercorke/robotics-toolbox-python/) project.
* Yeison Cardona for setup script and Python 3 enhancements to the **Robotics Toolbox for Python** provided in the PyPi [robotics-toolbox 0.1](https://pypi.org/project/robotics-toolbox/) project.

#### Disclaimers ####

* See the file [DISCLAIMER-RTB](./DISCLAIMER-RTB)
* See the file [DISCLAIMER-GED](./DISCLAIMER-GED)

#### Baseline Sources ####

* robotics-toolbox-python-master on June 23, 2018 from https://github.com/petercorke/robotics-toolbox-python
* robotics-toolbox-python v 1.8 on June 23, 2018 from https://code.google.com/archive/p/robotics-toolbox-python/downloads
* yeisoneng-robotics-toolbox-python-7aa5be5645f1 on June 23, 2018 from https://bitbucket.org/yeisoneng/robotics-toolbox-python
* robotics-toolbox-matlab-master on June 23, 2018 from https://github.com/petercorke/robotics-toolbox-matlab
* Robotics Toolbox v 9.8 on June 23, 2018 from http://www.petercorke.com/RTB/

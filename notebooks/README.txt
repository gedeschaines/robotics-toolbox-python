This directory currently holds Jupyter 4 notebook examples executed with a
Python 3 kernel. The content and structure of the ./notebooks directory is
subject to change as the Robotics Toolbox for Python - Update1 development
effort continues.

These Python notebooks have been successfully executed with the following
configurations.

Ubuntu 14.04, Python 3.4.3, IPython 5.3.0, Jupyter 4.3.0, Matplotlib 2.2.2
Ubuntu 16.04, Python 3.5.2, IPython 7.1.1, Jupyter 4.4.0, Matplotlib 3.0.2

The notebook examples can be run locally on a user's platform by invoking
'jupyter notebook' in this directory and then selecting an example folder
from those shown in the presented web browser page. Optionally, Python
source files corresponding to each Jupyter notebook example can be run in
IPython using a Python 3 kernel, by entering the example folder (i.e.,
subdirectory) and invoking IPython in a shell command terminal as follows.

$ cd ./example
$ ipython3

In [1]: %pylab

In [2]: run example.py

The kinematic2d and dynamics3d examples invoke the GNU Octave application with
a hard-coded system command line execution statement. Path and execution options
in the statement may require modification in accordance with user's OS platform
and installed Octave version. Specifically, on Ubuntu platforms an apt package
manager installs the Octave excutable as /usr/bin/octave, whereas a locally
built Octave application executable could be installed as /usr/local/bin/octave
or possibly /opt/octave/bin/octave. Additionally, example folders contain the
same _robot.py and .octaverc files, both of which specify hard-coded file paths
that may need to be modified to reflect a user's OS platform.

There have been numerous issues reported about Octave 4.2.x execution failure
generating an "octave exited with signal 6" message. In my case, this problem
was traced to the runtime load library path permitting Qt5 libraries other
than those associated with the Ubuntu 16.04 Octave application package to be
loaded. One possible fix involves invoking Octave with -W (--no-window-system)
option. However, this option precludes graphical output and consequently cannot
be used when executing Octave scripts of the kinematic2d and dynamics3d examples
since plots and animations are created.

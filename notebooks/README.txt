This directory currently holds Jupyter 4 notebook examples executed with a
Python 3 kernel. The content and structure of the ./notebooks directory is
subject to change as the Robotics Toolbox for Python - Update1 development
effort continues.

These Python notebooks have been successfully executed with the following
configurations.

Ubuntu 14.04, Python 3.4.3, IPython 5.3.0, Jupyter 4.3.0, Matplotlib 2.2.2
Ubuntu 16.04, Python 3.5.2, IPython 7.1.1, Jupyter 4.4.0, Matplotlib 3.0.2

The dynamics3d example invokes the GNU Octave application with a hard coded
system command line execution statement. The path and execution options in
the statement may require modification in accordance with user's OS platform
and installed Octave version. Specifically on an Ubuntu 16.04 platform, the
apt package manager installs the Octave 4.2.2 excutable as /usr/bin/octave.

There have been numerous issues reported about Octave 4.2.x execution failure
generating an "octave exited with signal 6" message. In my case, this problem
was traced to the runtime load library path permitting Qt5 libraries other
than those associated with the Ubuntu 16.04 Octave package to be loaded. One
possible fix involves invoking Octave with the -W (--no-window-system) option.
However, this option precludes graphical output and consequently cannot be
used when executing the octave_fdyn3d.m script of the dynamics3d example since
plots and animations are created.

""" Robotics Toolbox for Python -- Plotting routines

    robot/plot.py
    
    This file contains routines derived from functions provided in The
    Robotics Toolbox for MATLAB (RTB) which produce joint trajectory
    plots, frame transform plots and animation, and robot pose plots
    and animation.
    
    Public Functions
    
      + qplot      - joint trajectory plot
      + rbplot     - robot 2D and 3D pose plots and animation
      + trplot     - frame transform plot
      + tranimate  - frame transform animation
    
    Attributions
    
      + Luis Fernando Lara Tobar and Peter Corke, originators of
        the Robotics Toolbox for Python project which is available
        at https://github.com/petercorke/robotics-toolbox-python/.
    
    Disclaimers
    
      @see: L{disclaimer_rtb}
      @see: L{disclaimer_ged}
"""

import sys
import os
import time
from math import ceil

import warnings

warnings.simplefilter(action='ignore', category=FutureWarning)

import logging

logging.basicConfig(filename='matplotlib.log',level=logging.INFO)
logger = logging.getLogger('matplotlib')
logger.setLevel(logging.CRITICAL)

try:
    import numpy as np
    import matplotlib as mpl
except ImportError:
    print("* Error: NumPy and matplotlib package required.")
    print("         Suggest installing the SciPy stack.")
    sys.exit()

# Necessary matplotlib components.
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Line3D

# Imported into a PyLab application?
try:
   plot_isinteractive = isinteractive()
except NameError:
   plot_isinteractive = plt.isinteractive()

# Necessary robot components.
import _robot
from robot.utility import *
from robot.transform import *
from robot.trajectory import *
from robot.kinematics import *
from robot.Quaternion import *


def disclaimer_ged():
    """
    # Robotics Toolbox for Python -- Plotting routines
    # Copyright (c) 2018, Gary Deschaines
    # All rights reserved.
    #
    #
    # Redistribution and use in source and binary forms, with or without
    # modification, are permitted provided that the following conditions 
    # are met:
    #   * Redistributions of source code must retain the above copyright
    #     notice, this list of conditions and the following disclaimer.
    #   * Redistributions in binary form must reproduce the above copyright
    #     notice, this list of conditions and the following disclaimer in the
    #     documentation and/or other materials provided with the distribution.
    #
    # THIS SOFTWARE IS PROVIDED BY GARY DESCHAINES ``AS IS'' AND ANY EXPRESS OR
    # IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
    # OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
    # IN NO EVENT SHALL GARY DESCHAINES BE LIABLE FOR ANY DIRECT, INDIRECT,
    # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
    # NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    # DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    # THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
    # THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    """
    pass


def qplot(t, *args, **opts):
    """
    QPLOT Plot joint angles

    QPLOT(Q) is a convenience function to plot joint angle trajectories (Mx6) for
    a 6-axis robot, where each row represents one time step.

    The first three joints are shown as solid lines, the last three joints (wrist)
    are shown as dashed lines. A legend is also displayed.

    QPLOT(T, Q) as above but displays the joint angle trajectory versus time T (Mx1).

    @type t: t: M array
    @param t: joint trajectory times
    @type q: Mxn matrix (M rows of 1xn arrays)
    @param q: joint trajectory angles
    @type opts: kwargs
    @paran opts: option plot parameters (only 'title' for now)

    @see: L{jtraj}, L{rbplot}

    Copyright (C) 1993-2011, by Peter I. Corke

    @see: L{disclaimer_rtb}
    """
    title = 'qplot'

    if args is None or len(args) < 1:
        q = t
        t = arange(0,numrows(q))
    else:
        q = args[0]

    if numcols(q) < 6:
        error("qplot: expected 1x6 joint array")
    if numrows(q) == 1:
        q = np.mat(q).flatten()

    if 'title' in opts:
        title = opts['title']

    plt.clf()
    plt.plot(t[:], q[:,0], label='q1')
    plt.plot(t[:], q[:,1], label='q2')
    plt.plot(t[:], q[:,2], label='q3')
    plt.plot(t[:], q[:,3], '--', label='q4')
    plt.plot(t[:], q[:,4], '--', label='q5')
    plt.plot(t[:], q[:,5], '--', label='q6')
    plt.xlim([t[0], t[-1]])
    plt.grid()
    plt.xlabel('time')
    plt.ylabel('q')
    plt.legend()
    plt.title(title)

    if __name__ == '__main__' and not plot_isinteractive:
        plt.show(block=True)
    else:
        plt.show(block=False)

###
### Robot manipulator link chain animation plotting routines.
###

rbplotAnims2D = {}   # dict of instantiated 2D rbplot animation objects
rbplotClose2D = {}   # dict of instantiated 2D rbplot close handlers
rbplotResize2D = {}  # dict of instantiated 2D rbplot resize handlers
rbplotRbots2D = {}   # dict of 2D robots plotted per figure
rbplotLines2D = {}   # line artists drawn on each active 2D rbplot figure
rbplotText2D  = {}   # text artists drawn on each active 2D rbplot figure

rbplotAnims3D = {}   # dict of instantiated 3D rbplot animation objects
rbplotClose3D = {}   # dict of instantiated 3D rbplot close handlers
rbplotResize3D = {}  # dict of instantiated 3D rbplot resize handlers
rbplotRbots3D = {}   # dict of 3D robots plotted per figure
rbplotLines3D = {}   # line artists drawn on each active 3D rbplot figure
rbplotText3D  = {}   # text artists drawn on each active 3D rbplot figure


def _rbclose2d(event):
    """
    Close event handler for 2D robot plotting to remove figure, animator,
    close handler and resize handler from global dictionaries
    :param event:
    :return:
    """
    global rbplotRbots2D   # dict of 2D robots plotted per figure
    global rbplotAnims2D   # dict of 2D rbplot animators per figure
    global rbplotClose2D   # dict of 2D rbplot close handlers per figure
    global rbplotResize2D   # dict of 2D rbplot resize handlers per figure

    fig = event.canvas.figure
    if fig is not None:
        fign = fig.number
        if plt.fignum_exists(fign):
            if fign in rbplotRbots2D:
                rbplotRbots2D.pop(fign)
            if fign in rbplotAnims2D:
                rbplotAnims2D.pop(fign)
            if fign in rbplotClose2D:
                event.canvas.mpl_disconnect(rbplotClose2D[fign])
                rbplotClose2D.pop(fign)
            if fign in rbplotResize2D:
                event.canvas.mpl_disconnect(rbplotResize2D[fign])
                rbplotResize2D.pop(fign)


def _rbclose3d(event):
    """
    Close event handler for 3D robot plotting to remove figure, animator,
    close handler and resize handler from global dictionaries
    :param event:
    :return:
    """
    global rbplotRbots3D   # dict of 3D robots plotted per figure
    global rbplotAnims3D   # dict of 3D rbplot animators per figure
    global rbplotClose3D   # dict of 3D rbplot close handlers per figure
    global rbplotResize3D  # dict of 3D rbplot resize handlers per figure

    fig = event.canvas.figure
    if fig is not None:
        fign = fig.number
        if plt.fignum_exists(fign):
            if fign in rbplotRbots3D:
                rbplotRbots3D.pop(fign)
            if fign in rbplotAnims3D:
                rbplotAnims3D.pop(fign)
            if fign in rbplotClose3D:
                event.canvas.mpl_disconnect(rbplotClose3D[fign])
                rbplotClose3D.pop(fign)
            if fign in rbplotResize3D:
                event.canvas.mpl_disconnect(rbplotResize3D[fign])
                rbplotResize3D.pop(fign)


def _rbresize2d(event): ### Currently unused
    """
    Resize event handler for 2D robot plotting.
    :param event:
    :return:
    """
    global rbplotAnims2D  # dict of 2D rbplot animators per figure

    fig = event.canvas.figure
    if fig is not None:
        fign = fig.number
        if plt.fignum_exists(fign):
            if fign in rbplotAnims2D:
                ax = fig.gca(animated=True)  # Note: Does not work for Python v3.
                for a in ax.get_children():  # Works for v2 even though anim init
                    fig.draw_artist(a)       # func called after a resize event.
                fig.canvas.draw()


def _rbresize3d(event): ### Currently unused
    """
    Resize event handler for 3D robot plotting.
    :param event:
    :return:
    """
    global rbplotAnims3D  # dict of 3D rbplot animators per figure

    fig = event.canvas.figure
    if fig is not None:
        fign = fig.number
        if plt.fignum_exists(fign):
            if fign in rbplotAnims3D:
                ax = fig.gca(animated=True)  # Note: Does not work for Python v3
                for a in ax.get_children():  # Works for v2 even though anim init
                    fig.draw_artist(a)       # func called after a resize event.
                fig.canvas.draw()


def _rbinit2d():
    """
    Initialization function for 2D animation of robot manipulator link chain.
    
    @see: L{rbplot}, L{_rbplot_add_artists2d}, L{_rbanim2d}
    """
    global rbplotRbots2D, rbplotLines2D, rbplotText2D

    fig = plt.gcf()  # this assumes current figure is being initialized

    if fig is not None \
            and plt.fignum_exists(fig.number) \
            and fig.number in rbplotRbots2D \
            and rbplotLines2D[fig.number][0] in fig.gca().get_lines():
            
        fign = fig.number
        lines2d = rbplotLines2D[fign]
        text2d = rbplotText2D[fign]
        
        if not plot_isinteractive or \
            (fign in rbplotAnims2D and len(rbplotAnims2D[fign]._save_seq) == 0):
            # Note: Referencing FuncAnimation private attribute _save_seq is
            # a kludge to prevent erasing artists drawn in last frame. This
            # does not work for matplotlab versions 1.5.1 and onward since
            # FuncAnimation _init_draw() resets the _save_seq list. Setting
            # blitting off seems to be the appropriate solution to loss of
            # last drawn frame when figure resizing occurs.
            #
            # only clear lines and text added for last robot drawn on figure
            for a in lines2d[-6:]:  # Note: 6 lines drawn per robot
                a.set_data([], [])
  
            for a in text2d[-2:]:   # Note: 2 text fields displayed per robot
                a.set_text('')
                
        return lines2d + text2d

    else:
        return []


def _rbanim2d(nf, fign, robot, Q, lines2d, text2d):
    """
    Callback function for 2D animation of robot manipulator link chain.

    @type nf: int
    @param nf: animation frame number (initial frame is 0)
    @type fign: int
    @param fign: rbplot figure number (not currently used)
    @type robot: RTB robot object
    @param robot: the n-dof robot manipulator link chain to animate
    @type Q: Mxn matrix (M rows of 1xn arrays)
    @param Q: the manipulator chain joint space trajecto

    @see: L{rbplot}, L{_rbplot_add_artists2d}
    """
    if nf >= numrows(Q):
        # just in case FuncAnimation initialized with frames > numrows(Q)
        return lines2d + text2d
        
    q = np.mat(Q[nf])
    
    # retrieve robot links and plotting parameters
    
    n = robot.n
    L = robot.links

    fps = robot.get_handle('fps')
    mag = robot.get_handle('mag')
    
    # allocate storage for base and each link node
    
    x  = np.zeros(n+1)
    y  = np.zeros(n+1)
    xs = np.zeros(n+1)
    ys = np.zeros(n+1)
    
    # initialize link chain lines
    
    b = transl(robot.base)  # note: b is a 3x1 matrix
   
    x[0] = b[0,0]
    y[0] = b[1,0]

    # compute the link transforms, and record the origin 
    # of each frame for the animation.

    t = robot.base
    
    for j in range(1,n+1):
        t = t * L[j-1].tr(q[0,j-1])
        x[j]  = t[0,3]
        y[j]  = t[1,3]

    t = t * robot.tool
    
    # save initial pose in Cartesian coordinates
    
    if nf == 0:
        robot.set_handle('x0', x.copy())
        robot.set_handle('y0', y.copy())
    
    x0 = robot.get_handle('x0')
    y0 = robot.get_handle('y0')

    # Note: Artists for this robot are at the end of 
    # rbplotLines2D[fign] and rbplotText2D[fign] lists,
    # where fign is the figure number associated with
    # the FuncAnimation object drawing the artists.
    
    # draw the effector to target line
    
    xyzTgt = robot.get_handle('xyzTgt')
    
    xe = x0[-1]
    ye = y0[-1]
    xt = xyzTgt[0]
    yt = xyzTgt[1]
    
    lines2d[-6].set_data([xt], [yt])          # target location
    lines2d[-5].set_data([xe, xt], [ye, yt])  # effector to target line
    
    # draw the robot stick figure
    
    lines2d[-4].set_data(x0,y0)  # initial pose
    lines2d[-3].set_data(x,y)    # current pose

    # draw the wrist frame
    
    xv = t*(mat([mag,0,0,1]).T)
    yv = t*(mat([0,mag,0,1]).T)

    lines2d[-2].set_data([t[0,3], xv[0]], [t[1,3], xv[1]])  # x-axis
    lines2d[-1].set_data([t[0,3], yv[0]], [t[1,3], yv[1]])  # y-axis
             
    # name and frame time
    
    text2d[-2].set_text(robot.name)
    text2d[-1].set_text('time = %.3f' % (float(nf)/fps))  # representative time
    
    return lines2d + text2d


def _rbinit3d():
    """
    Initialization function for 3D animation of robot manipulator link chain.
    
    @see: L{rbplot}, L{_rbplot_add_artists3d}, L{_rbanim3d}
    """
    global rbplotRbots3D, rbplotLines3D, rbplotText3D

    fig = plt.gcf()  # this assumes current figure is being initialized

    if fig is not None \
            and plt.fignum_exists(fig.number) \
            and fig.number in rbplotRbots3D \
            and rbplotLines3D[fig.number][0] in fig.gca().get_lines():
            
        fign = fig.number
        lines3d = rbplotLines3D[fign]
        text3d = rbplotText3D[fign]

        if plot_isinteractive or \
            (fign in rbplotAnims3D and len(rbplotAnims3D[fign]._save_seq) == 0):
            # Note: Referencing FuncAnimation private attribute _save_seq is
            # a kludge to prevent erasing artists drawn in last frame. This
            # does not work for matplotlab versions 1.5.1 and onward since
            # FuncAnimation _init_draw() resets the _save_seq list. Setting
            # blitting off seems to be the appropriate solution to loss of
            # last drawn frame when figure resizing occurs.
            #
            # only clear lines and text added for last robot drawn on figure
            for a in lines3d[-9:]:  # Note: 9 lines drawn per robot
                a.set_data([], [])
                a.set_3d_properties([])

            for a in text3d[-2:]:   # Note: 2 text fields displayed per robot
                a.set_text('')
    
        return lines3d + text3d

    else:
        return []


def _rbanim3d(nf, fign, robot, Q, lines3d, text3d):
    """
    Callback function for 3D animation of robot manipulator link chain

    @type nf: int
    @param nf: animation frame number (initial frame is 0)
    @type fign: int
    @param fign: rbplot figure number (not currently used)
    @type robot: RTB robot object
    @param robot: the n-dof robot manipulator link chain to animate
    @type Q: Mxn matrix (M rows of 1xn arrays)
    @param Q: the manipulator chain joint space trajectory

    @see: L{rbplot}, L{_rbplot_add_artists3d}
    """
    
    if nf >= numrows(Q):
        # just in case FuncAnimation initialized with frames > numrows(Q)
        return lines3d + text3d
    
    q = np.mat(Q[nf])
    
    # retrieve robot links and plotting parameters
    
    n = robot.n
    L = robot.links

    fps = robot.get_handle('fps')
    mag = robot.get_handle('mag')
    zmin = robot.get_handle('zmin')

    # allocate storage for base, and each link node
    
    x  = np.zeros(n+1)
    y  = np.zeros(n+1)
    z  = np.zeros(n+1)
    xs = np.zeros(n+1)
    ys = np.zeros(n+1)
    zs = np.zeros(n+1)
    
    # initialize link chain and shadow lines
    
    b = transl(robot.base)  # note: b is a 3x1 matrix
    
    x[0]  = b[0,0]
    y[0]  = b[1,0]
    z[0]  = b[2,0]
    xs[0] = b[0,0]
    ys[0] = b[1,0]
    zs[0] = zmin

    # compute the link transforms, and record the origin 
    # of each frame for the animation.

    t = robot.base
    for j in range(1,n+1):
        t = t * L[j-1].tr(q[0,j-1])
        x[j]  = t[0,3]
        y[j]  = t[1,3]
        z[j]  = t[2,3]
        xs[j] = t[0,3]
        ys[j] = t[1,3]
        zs[j] = zmin
    t = t * robot.tool

    # save initial pose in Cartesian coordinates
    
    if nf == 0:
        robot.set_handle('x0', x.copy())
        robot.set_handle('y0', y.copy())
        robot.set_handle('z0', z.copy())
        
    x0 = robot.get_handle('x0')
    y0 = robot.get_handle('y0')
    z0 = robot.get_handle('z0')

    # Note: Artists for this robot are at the end of 
    # rbplotLines3D[fign] and rbplotText3D[fign] lists,
    # where fign is the figure number associated with
    # the FuncAnimation object drawing the artists.
    
    # draw the effector to target line
    
    xyzTgt = robot.get_handle('xyzTgt')
    
    xe = x0[-1]
    ye = y0[-1]
    ze = z0[-1]
    xt = xyzTgt[0]
    yt = xyzTgt[1]
    zt = xyzTgt[2]
    
    lines3d[-9].set_data([xt], [yt])          # target location
    lines3d[-9].set_3d_properties([zt])
    lines3d[-8].set_data([xe, xt], [ye, yt])  # effector to target line
    lines3d[-8].set_3d_properties([ze, zt])
    
    # draw the robot stick figure and the shadow

    lines3d[-7].set_data(xs,ys)                            # shadow
    lines3d[-7].set_3d_properties(zs)
    lines3d[-6].set_data([b[0,0],b[0,0]],[b[1,0],b[1,0]])  # base
    lines3d[-6].set_3d_properties([zmin,b[2,0]])
    lines3d[-5].set_data(x0,y0)                            # initial pose
    lines3d[-5].set_3d_properties(z0)
    lines3d[-4].set_data(x,y)                              # current pose
    lines3d[-4].set_3d_properties(z)

    # draw the wrist frame

    xv = t*(mat([mag,0,0,1]).T)
    yv = t*(mat([0,mag,0,1]).T)
    zv = t*(mat([0,0,mag,1]).T)

    lines3d[-3].set_data([t[0,3], xv[0]], [t[1,3], xv[1]])  # x-axis
    lines3d[-3].set_3d_properties([t[2,3], xv[2]])
    lines3d[-2].set_data([t[0,3], yv[0]], [t[1,3], yv[1]])  # y-axis
    lines3d[-2].set_3d_properties([t[2,3], yv[2]])
    lines3d[-1].set_data([t[0,3], zv[0]], [t[1,3], zv[1]])  # z-axis
    lines3d[-1].set_3d_properties([t[2,3], zv[2]])
    
    # name and frame time

    text3d[-2].set_text(robot.name)
    text3d[-1].set_text('time = %.3f' % (float(nf)/fps))  # representative time
    
    return lines3d + text3d


def _rbplot_add_artists2d(ax, robot):
    """
    Add 2D line and text artists to the given figure axis for representing
    the specified 2D robot's animation.

    @type ax: matplotlib.axes.AxesSubplot object
    @param ax: figure axes for line and text drawing
    @type robot: RTB Robot object
    @param robot: the robot being drawn
    @rtype lines: array of Line2D artists
    @return lines: the line artists to be drawn
    @rtype text: array of text artists
    @return text: the text artists to be drawn

    @see: L{rbplot}, L{_rbanim2d}, L{_rbinit2d}
    """

    # IMPORTANT: Introducing additional lines and text in this function
    #            requires corresponding changes to lines2d and text2d
    #            slice indices in the _rbinit2d function.

    # lines in order to be drawn
    line1 = Line2D([], [], color='r', ls=' ', lw=2.0,      # target location
                   marker='x', mew=2.0, mec='r', mfc='r')
    line2 = Line2D([], [], color='k', ls=':', lw=1.0,      # effector to target line
                   marker=' ')
    line3 = Line2D([], [], color='g', ls='-', lw=1.0,      # link chain at start
                   marker='o', mew=1.0, mec='k', mfc='g')
    line4 = Line2D([], [], color='b', ls='-', lw=2.0,      # link chain in motion
                   marker='o', mew=1.0, mec='k', mfc='b')
    line5 = Line2D([], [], color='r', ls='-', lw=1.0,      # wrist frame x-axis
                   marker=' ', mew=1.0, mec='r', mfc='r')
    line6 = Line2D([], [], color='g', ls='-', lw=1.0,      # wrist frame y-axis
                   marker=' ', mew=1.0, mec='g', mfc='g')
    ax.add_line(line1)
    ax.add_line(line2)
    ax.add_line(line3)
    ax.add_line(line4)
    ax.add_line(line5)
    ax.add_line(line6)
    lines = [line1, line2, line3, line4, line5, line6]

    # add robot name and frame time text field
    xlims = robot.get_plotopt('xlim')
    ylims = robot.get_plotopt('ylim')
    dx = abs(xlims[1]-xlims[0])/6
    dy = abs(ylims[1]-ylims[0])/24
    b = transl(robot.base).T
    pb = b + np.mat([[-dx], [-dy], [0.0]]).T
    name_text = ax.text(pb[0, 0], pb[0, 1], robot.name)
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
    text = [name_text, time_text]

    return lines, text


def _rbplot_add_artists3d(ax, robot):
    """
    Add 3D line and text artists to the given figure axis for representing
    the specified 3D robot's animation.

    @type ax: matplotlib.axes.Axes3DSubplot object
    @param ax: figure axes for line and text drawing
    @type robot: RTB Robot object
    @param robot: the robot being drawn
    @rtype lines: array of Line3D artists
    @return lines: the line artists to be drawn
    @rtype text: array of text3D artists
    @return text: the text artists to be drawn

    @see: L{rbplot}, L{_rbanim3d}, L{_rbinit3d}
    """

    # IMPORTANT: Introducing additional lines and text in this function
    #            requires corresponding changes to lines3d and text3d
    #            slice indices in the _rbinit3d function.

    # lines in order to be drawn
    line1 = Line3D([],[],[],color='r',ls=' ',lw=2.0,       # target in 3D space
                   marker='x',mew=2.0,mec='r',mfc='r')
    line2 = Line3D([],[],[],color='k',ls=':',lw=1.0,       # eff to tgt line
                   marker=' ') 
    line3 = Line3D([], [], [], color='k', ls='-', lw=2.0,  # link chain shadow
                   marker='o', mew=1.0, mec='k', mfc='k')
    line4 = Line3D([], [], [], color='r', ls='-', lw=4.0,  # robot base
                   marker='o', mew=1.0, mec='r', mfc='r')
    line5 = Line3D([], [], [], color='g', ls='-', lw=1.0,  # link chain start
                   marker='o', mew=1.0, mec='k', mfc='g')
    line6 = Line3D([], [], [], color='b', ls='-', lw=2.0,  # link chain motion
                   marker='o', mew=1.0, mec='k', mfc='b')
    line7 = Line3D([], [], [], color='r', ls='-', lw=1.0,  # wrist frame x-axis
                   marker=' ', mew=1.0, mec='r', mfc='r')
    line8 = Line3D([], [], [], color='g', ls='-', lw=1.0,  # wrist frame y-axis
                   marker=' ', mew=1.0, mec='g', mfc='g')
    line9 = Line3D([], [], [], color='b', ls='-', lw=1.0,  # wrist frame y-axis
                   marker=' ', mew=1.0, mec='b', mfc='b')
    ax.add_line(line1)
    ax.add_line(line2)
    ax.add_line(line3)
    ax.add_line(line4)
    ax.add_line(line5)
    ax.add_line(line6)
    ax.add_line(line7)
    ax.add_line(line8)
    ax.add_line(line9)
    lines = [line1, line2, line3, line4, line5, line6, line7, line8, line9]

    # add robot name and frame time text field
    xlims = robot.get_plotopt('xlim')
    ylims = robot.get_plotopt('ylim')
    zlims = robot.get_plotopt('zlim')
    dx = abs(xlims[1]-xlims[0])/40
    dy = abs(ylims[1]-ylims[0])/40
    dz = zlims[0]
    b = transl(robot.base).T
    pb = b + np.mat([[dx], [dy], [dz]]).T
    name_text = ax.text3D(pb[0, 0], pb[0, 1], pb[0, 2], robot.name)
    dx = xlims[0] + xlims[0]/10
    dy = ylims[0] + ylims[0]/10
    dz = zlims[1] + zlims[1]/2
    time_text = ax.text3D(dx, dy, 1.5, '')
    text = [name_text, time_text]

    return lines, text


def rbplot(robot, Q, phold=False, rec=0, **opts):
    """
    Display animated 2D or 3D plots of robot manipulator link chain.

    @type robot: RTB robot object
    @param robot: the n-dof robot manipulator link chain to animate
    @type Q: Mxn matrix (M rows of 1xn arrays)
    @param Q: the manipulator chain joint space trajectory
    @type phold: bool
    @param phold: plot on current figure
    @type rec: int
    @param rec: record animation if not 0
    @type opts: **kwargs
    @param opts: optional plotting parameters

    @see: L{qplot}, L{jtraj}
    """
    global rbplotRbots2D   # dict of 2D robots plotted per figure
    global rbplotAnims2D   # dict of 2D rbplot animators
    global rbplotClose2D   # dict of 2D rbplot close handlers
    global rbplotResize3D  # dict of 2D rbplot resize handlers
    global rbplotLines2D   # line artists drawn for 2D plots of Q trajectories
    global rbanimText2D    # text artists drawn for 2D plots of Q trajectories

    global rbplotRbots3D   # dict of 3D robots plotted per figure
    global rbplotAnims3D   # dict of 3D rbplot animators
    global rbplotClose3D   # dict of 3D rbplot close handlers
    global rbplotResize3D  # dict of 3D rbplot resize handlers
    global rbanimLines3D   # line artists drawn for 3D plots of Q trajectories
    global rbanimText3D    # text artists drawn for 3D plots of Q trajectories

    if Q is None:
        # no trajectory, use robot's current joint configuration
        Q = mat(robot.q)
    elif isinstance(Q, list) and len(Q) == robot.n:
        # single array of joint values, convert to matix
        Q = mat(Q).flatten()
        
    nrows = numrows(Q)
    n = robot.n
    if numcols(Q) != n:
        error('Insufficient columns in Q')
        
    # save target position in Cartesian coordinates
    
    qm = Q[nrows-1]
    tm = fkine(robot, qm)
    xyzTgt = zeros(3) 
    xyzTgt[0] = tm[0,3]
    xyzTgt[1] = tm[1,3]
    xyzTgt[2] = tm[2,3]
    robot.set_handle('xyzTgt', xyzTgt)
        
    # Processing selectors
    
    Plot3D = robot.get_handle('p3D')  # plot in 3D flag
    Record = rec                      # record movie flag
    
    # If recording animation, assign an animation writer.
    
    Writer = None
    if Record != 0:
        try:
            Writer = animation.writers['ffmpeg']
        except (KeyError, RuntimeError):
            try:
                Writer = animation.writers['avconv']
            except (KeyError, RuntimeError):
                print("Cannot record animation, no animation writers available!")
                print("Try installing ffmpeg or avconv.")
                Record = 0
                
    # Determine whether or not to add this robot to an existing figure, or
    # draw it on a new figure.

    if phold and (plt.gcf() is not None):
        fign = plt.gcf().number
        if plt.fignum_exists(fign):
            if Plot3D == 0:
                add_robot = (fign in rbplotRbots2D) and\
                            (robot.name not in rbplotRbots2D[fign])
            else:
                add_robot = (fign in rbplotRbots3D) and \
                            (robot.name not in rbplotRbots3D[fign])
        else:
            add_robot = False
    else:
        fign = None
        add_robot = False

    # Draw robot

    if fign is not None and add_robot:
        # This robot has not been added to the current active figure;
        # do not re-initialize animation (implying a hold 'on' case)

        fig = plt.gcf()
        ax = fig.gca()
        if Plot3D == 0:
            rbplotRbots2D[fign].append(robot.name)
            # add artists to draw 2D robot
            (lines, text) = _rbplot_add_artists2d(ax, robot)
            rbplotLines2D[fign] = rbplotLines2D[fign] + lines
            rbplotText2D[fign] = rbplotText2D[fign] + text
        else:
            rbplotRbots3D[fign].append(robot.name)
            # getfigure's axes
            ax = fig.gca()
            # add artists to draw 3D robot
            (lines, text) = _rbplot_add_artists3d(ax, robot)
            rbplotLines3D[fign] = rbplotLines3D[fign] + lines
            rbplotText3D[fign] = rbplotText3D[fign] + text
        fig.canvas.draw()

    else:
        # New figure to be created
        # specify figure dimensions based on record selection

        if Record == 0:
            fig = plt.figure(figsize=(8, 6), dpi=80, facecolor='white')
        else:
            fig = plt.figure(figsize=(6, 4), dpi=80, facecolor='white')

        fign = fig.number

        # Specify plotting objects and parameters.

        if 'title' in opts:
            title = opts['title']
        else:
            title = robot.name

        xlims = robot.get_plotopt('xlim')
        ylims = robot.get_plotopt('ylim')
        zlims = robot.get_plotopt('zlim')

        if Plot3D == 0:
            # add robot to rbplot 2D robot list
            if fign in rbplotRbots2D:
                rbplotRbots2D[fign] = [robot.name]
            else:
                rbplotRbots2D.update({fign: [robot.name]})
            # create axes for drawing 2D artists
            ax = fig.add_subplot(111, autoscale_on=False)
            ax.set_title(title)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_xlim(xlims)
            ax.set_ylim(ylims)
            ax.set_aspect('equal')
            ax.grid()
            # add 2D artists to axes
            (lines, text) = _rbplot_add_artists2d(ax, robot)
            if fign in rbplotLines2D:
                # replace previous 2D artist lists for this figure
                rbplotLines2D[fign] = lines
                rbplotText2D[fign] = text
            else:
                # add key and 2D artist lists for this new figure
                rbplotLines2D.update({fign: lines})
                rbplotText2D.update({fign: text})
            fig.canvas.draw()

        else:
            # add robot to rbplot 3D robot list
            if fign in rbplotRbots3D:
                rbplotRbots3D[fign] = [robot.name]
            else:
                rbplotRbots3D.update({fign: [robot.name]})
            # create axes for drawing 3D artists
            ax = fig.add_subplot(111, projection='3d')
            ax.set_title(title)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_xlim3d(xlims)
            ax.set_ylim3d(ylims)
            ax.set_zlim3d(zlims)
            #ax.set_aspect('equal')
            ax.grid()
            # add 3D artists to axes
            (lines, text) = _rbplot_add_artists3d(ax, robot)
            if fign in rbplotLines3D:
                # replace previous 3D artist lists for this figure
                rbplotLines3D[fign] = lines
                rbplotText3D[fign] = text
            else:
                # add key and 3D artist lists for this new figure
                rbplotLines3D.update({fign: lines})
                rbplotText3D.update({fign: text})
            fig.canvas.draw()

            zlim = ax.get_zlim3d()
            robot.set_handle('zmin', zlim[0])

    # Determine animation interval time. This is a hold over from the 
    # IK_Solver application where the inverse kinematic iterative solver
    # was called from frame to frame.
   
    fps       = max(robot.get_handle('fps'),1)
    tdel      = 1.0/fps
    tdel_msec = 1000.0*tdel
    tmsec0    = 1000.0*time.clock()
    if Plot3D == 0:
        _rbanim2d(0, fign, robot, Q, rbplotLines2D[fign], rbplotText2D[fign])
    else:
        _rbanim3d(0, fign, robot, Q, rbplotLines3D[fign], rbplotText3D[fign])
    tmsec1    = 1000.0*time.clock()
    tstep     = tmsec1 - tmsec0
    #interval  = ceil(tmsec1-tmsec0)  # allows faster than real-time
    interval  = tdel_msec - tstep     # approximates real-time
    """
    print("tdel_msec = %8.3f" % tdel_msec)
    print("t1 - t0   = %8.3f" % tstep)
    print("interval  = %8.3f" % interval)
    """
  
    # Assign figure close event handlers.

    if Plot3D == 0:
        cid = fig.canvas.mpl_connect('close_event', _rbclose2d)
        if fign in rbplotClose2D:
            fig.canvas.mpl_disconnect(rbplotClose2D[fign])
            rbplotClose2D[fign] = cid
        else:
            rbplotClose2D.update({fign: cid})
    else:
        cid = fig.canvas.mpl_connect('close_event', _rbclose3d)
        if fign in rbplotClose3D:
            fig.canvas.mpl_disconnect(rbplotClose3D[fign])
            rbplotClose3D[fign] = cid
        else:
            rbplotClose3D.update({fign: cid})

    """
    # Assign figure resize event handlers.
    #
    # Early attempt at preserving the last drawn pose when a 
    # resize event occurs after animation completes.
    
    if Plot3D == 0:
        cid = fig.canvas.mpl_connect('resize_event', _rbresize2d)
        if fign in rbplotResize2D:
            fig.canvas.mpl_disconnect(rbplotResize2D[fign])
            rbplotResize2D[fign] = cid
        else:
            rbplotResize2D.update({fign: cid})
    else:
        cid = fig.canvas.mpl_connect('resize_event', _rbresize3d)
        if fign in rbplotResize3D:
            fig.canvas.mpl_disconnect(rbplotResize3D[fign])
            rbplotResize3D[fign] = cid
        else:
            rbplotResize3D.update({fign: cid})
    """

    fig.canvas.mpl_connect('button_press_event', robot.onclick)

    # Specify animation parameters and assign functions.
   
    nframes = nrows
    blit = False
    if Record == 0:
        #blit = True  # blit results in image loss after resize for Matplotlib v1.5.1+
        pass

    if Plot3D == 0:
        _rbinit2d()
        anim = animation.FuncAnimation(fig, _rbanim2d,
                                       fargs=(fign, robot, Q, rbplotLines2D[fign],
                                              rbplotText2D[fign]),
                                       init_func=_rbinit2d,
                                       frames=nframes, blit=blit,
                                       interval=interval, repeat=False)
    else:
        _rbinit3d()
        anim = animation.FuncAnimation(fig, _rbanim3d,
                                       fargs=(fign, robot, Q, rbplotLines3D[fign],
                                              rbplotText3D[fign]),
                                       init_func=_rbinit3d,
                                       frames=nframes, blit=blit,
                                       interval=interval, repeat=False)

    # Preserve anim object for when this routine exits.

    if Plot3D == 0:
        if fign in rbplotAnims2D:
            rbplotAnims2D[fign] = anim
        else:
            rbplotAnims2D.update({fign: anim})
    else:
        if fign in rbplotAnims3D:
            rbplotAnims3D[fign] = anim
        else:
            rbplotAnims3D.update({fign: anim})

    # Begin animation.

    if Record == 0 or Writer is None:
        if Plot3D == 0:
            #print("Move cursor into plot region and press left mouse button")
            #print("to initiate IK solver for selected target location.")
            pass
        else:
            #print("With cursor in the plot region, press left mouse button")
            #print("to initiate IK solver for randomly positioned target.")
            pass
    else:
        plt_ion = plt.isinteractive()
        if plt_ion: plt.ioff()
        print("Creating animation video; presentation will begin shortly ...")
        writer = Writer(fps=fps,
                        metadata=dict(artist='RTB -- rbplot'),
                        bitrate=-1)
        filename = robot.name.replace(' ','_') + '.mp4'
        anim.save(filename, writer=writer)
        print("Animation saved in file %s" % filename)
        if plt_ion: plt.ion()
    
    if __name__ == '__main__' and not plot_isinteractive:
        plt.show(block=True)
    else:
        plt.show(block=False)

    robot.q = Q[-1]  # save last pose in joint coordinates
    
    return

###
### Frame transforms plotting functions.
###

def trplot(T, fig=None, **opts):
    """
    TRPLOT Draw a coordinate frame

    TRPLOT(T, OPTIONS) draws a 3D coordinate frame represented by the homogeneous
    transform T (4x4).

    H = TRPLOT(T, OPTIONS) as above but returns a handle.

    TRPLOT(H, T) moves the coordinate frame described by the handle H to
    the pose T (4x4).

    TRPLOT(R, OPTIONS) draws a 3D coordinate frame represented by the orthonormal
    rotation matrix R (3x3).

    H = TRPLOT(R, OPTIONS) as above but returns a handle.

    TRPLOT(H, R) moves the coordinate frame described by the handle H to
    the orientation R.

    Options::
    'title',text       The figure title
    ### THE FOLLOWING Options ARE NOT CURRENTLY IMPLEMENTED ###
    'color',C          The color to draw the axes, MATLAB colorspec C
    'noaxes'           Don't display axes on the plot
    'axis',A           Set dimensions of the MATLAB axes to A=[xmin xmax ymin ymax zmin zmax]
    'frame',F          The frame is named {F} and the subscript on the axis labels is F.
    'text_opts',opt    A cell array of MATLAB text properties
    'handle',H         Draw in the MATLAB axes specified by the axis handle H
    'view',V           Set plot view parameters V=[az el] angles, or 'auto'
                       for view toward origin of coordinate frame
    'arrow'            Use arrows rather than line segments for the axes
    'width',w          Width of arrow tips (default 1)
    'thick',t          Thickness of lines (default 0.5)
    '3d'               Plot in 3D using anaglyph graphics
    'anaglyph',A       Specify anaglyph colors for '3d' as 2 characters for
                       left and right (default colors 'rc'):
                         'r'   red
                         'g'   green
                         'b'   green
                         'c'   cyan
                         'm'   magenta
    'dispar',D         Disparity for 3d display (default 0.1)

    Examples::

        trplot(T, 'frame', 'A')
        trplot(T, 'frame', 'A', 'color', 'b')
        trplot(T1, 'frame', 'A', 'text_opts', {'FontSize', 10, 'FontWeight', 'bold'})

        h = trplot(T, 'frame', 'A', 'color', 'b');
        trplot(h, T2);

    3D anaglyph plot
        trplot(T, '3d');

    Notes::
    - The arrow option requires the third party package arrow3.
    - The handle H is an hgtransform object.
    - When using the form TRPLOT(H, ...) the axes are not rescaled.
    - The '3d' option requires that the plot is viewed with anaglyph glasses.
    - You cannot specify 'color'

    @see: L{tranimate}.

    Copyright (C) 1993-2011, by Peter I. Corke

    @see: L{disclaimer_rtb}
    """
    global tranim_lines3D  # line artists drawn for 3D plots of frame transforms
    global tranim_text3D   # text artists drawn for 3D plots of frame transforms

    title = ""
    if 'title' in opts:
       title = opts['title']

    if isrot(T) or ishomog(T):
        q = quaternion(T)
    elif isinstance(T, quaternion):
        q = T
    else:
        raise ValueError

    x = q * np.mat([1.0, 0.0, 0.0])
    y = q * np.mat([0.0, 1.0, 0.0])
    z = q * np.mat([0.0, 0.0, 1.0])

    if (fig is None) or not plt.fignum_exists(fig.number):
        # This will create a new figure unless hold 'on'
        fig = plt.figure(figsize=(8, 6), dpi=80, facecolor='white')
        ax = fig.add_subplot(111, projection='3d')
        # lines in order to be drawn
        line1 = Line3D([], [], [], color='r', ls='-', lw=2.0,  # x-axis
                       marker=' ', mew=1.0, mec='r', mfc='r')
        line2 = Line3D([], [], [], color='g', ls='-', lw=2.0,  # y-axis
                       marker=' ', mew=1.0, mec='g', mfc='g')
        line3 = Line3D([], [], [], color='b', ls='-', lw=2.0,  # z-axis
                       marker=' ', mew=1.0, mec='b', mfc='b')
        time_text = ax.text(-1.0, -1.0, 1.5, '', )
        ax.set_title(title)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim3d([-1.0, 1.0])
        ax.set_ylim3d([-1.0, 1.0])
        ax.set_zlim3d([-1.0, 1.0])
        # ax.set_aspect('equal')
        ax.grid()
        ax.add_line(line1)
        ax.add_line(line2)
        ax.add_line(line3)
        tranim_lines3D = [line1, line2, line3]
        tranim_text3D = [time_text]

    tranim_lines3D[0].set_data([0, x[0, 0]], [0, x[0, 1]])  # x-axis
    tranim_lines3D[0].set_3d_properties([0, x[0, 2]])
    tranim_lines3D[1].set_data([0, y[0, 0]], [0, y[0, 1]])  # y-axis
    tranim_lines3D[1].set_3d_properties([0, y[0, 2]])
    tranim_lines3D[2].set_data([0, z[0, 0]], [0, z[0, 1]])  # z-axis
    tranim_lines3D[2].set_3d_properties([0, z[0, 2]])
    fig.canvas.draw()

    return fig


def _trinit3d():
    """
    Initialization function for 3D animation of frame transforms.
    """
    global tranim_lines3D

    for l in tranim_lines3D:
        l.set_data([], [])
        l.set_3d_properties([])

    return tranim_lines3D


def _tranim3d(nf, Ttraj, tranim_lines3D):
    """
    Callback function for 3D animation of frame transforms.

    @type nf: int
    @param nf: animation frame number (initial frame is 0)
    @type Ttraj: list of nxn matrices
    @param Ttraj: the trajectory of frame transforms
    
    @see: L{tranimate}
    """
    if nf >= len(Ttraj):
        return tranim_lines3D

    T = Ttraj[nf]

    if isrot(T) or ishomog(T):
        q = quaternion(T)
    elif isinstance(T, quaternion):
        q = T
    else:
        raise ValueError

    x = q * np.mat([1.0, 0.0, 0.0])
    y = q * np.mat([0.0, 1.0, 0.0])
    z = q * np.mat([0.0, 0.0, 1.0])

    tranim_lines3D[0].set_data([0, x[0, 0]], [0, x[0, 1]])  # x-axis
    tranim_lines3D[0].set_3d_properties([0, x[0, 2]])
    tranim_lines3D[1].set_data([0, y[0, 0]], [0, y[0, 1]])  # y-axis
    tranim_lines3D[1].set_3d_properties([0, y[0, 2]])
    tranim_lines3D[2].set_data([0, z[0, 0]], [0, z[0, 1]])  # z-axis
    tranim_lines3D[2].set_3d_properties([0, z[0, 2]])

    return tranim_lines3D


def tranimate(P1, P2, nsteps=50, fps=10, rec=0, **opts):
    """
    TRANIMATE Animate a coordinate frame

    TRANIMATE(P1, P2, OPTIONS) animates a 3D coordinate frame moving from pose
    P1 to pose P2.  Poses P1 and P2 can be represented by:
      - homogeneous transformation matrices (4x4)
      - orthonormal rotation matrices (3x3)
      - Quaternion

    TRANIMATE(P, OPTIONS) animates a coordinate frame moving from the identity
    pose to the pose P represented by any of the types listed above.

    TRANIMATE(PSEQ, OPTIONS) animates a trajectory, where PSEQ is any of
      - homogeneous transformation matrix sequence (4x4xN)
      - orthonormal rotation matrix sequence (3x3xN)
      - Quaternion vector (Nx1)

    Options::
     'fps', fps    Number of frames per second to display (default 10)
     'nsteps', n   The number of steps along the path (default 50)
     'rec', 0|1    Record animation in a video file (default 0=No)
     'axis', A     Axis bounds [xmin, xmax, ymin, ymax, zmin, zmax]
     'movie', M    Save frames as a movie in filepath M

    Notes::
      - The 'rec' option saves animation frames to mpeg video file if
        a video writer, such as ffmpeg or rvconv, is available.
      - The 'movie' options saves frames as files NNNN.png.
      - When using 'movie' option ensure that the window is fully visible.
      - To convert frames to a movie use a command like:
          ffmpeg -r 10 -i %04d.png out.avi

    @see: L{trplot}.

    Copyright (C) 1993-2011, by Peter I. Corke

    @see: L{disclaimer_rtb}
    """
    global tranim_lines3D

    T1 = None
    T2 = None
    if isinstance(P1, list) and len(P1) > 0:
        if P2 is not None:
            error('tranimate: only one transform list is needed')
        T2 = []
        if isinstance(P1[0], quaternion):
            # convert quaternions to homogeneous transforms
            for Q in P1:
                T2.append(Q.tr())
        elif isrot(P1[0]):
            # convert rotation matrices to homogeneous transforms
            for R in P1:
                T2.append(r2t(R))
        elif ishomog(P1[0]):
            # already homogeneous transforms
            T2 = P1
        else:
            error('tranimate: expected quaternion, rotation or homogeoneos transforms')
        T1 = np.eye(4, 4)
    else:
        if isinstance(P1, quaternion):
            # convert quaternion to homogeneous transform
            T1 = P1.tr()
        elif isrot(P1):
            # convert rotation matrix to homogeneous transform
            T1 = r2t(P1)
        elif ishomog(P1):
            # already a homogeneous transform
            T1 = P1
        else:
            error('tranimate: expected quaternion, rotation or homogeoneos transforms')
        if isempty(P2):
            T2 = [T1.copy()]
            T1 = np.eye(4, 4)
        else:
            if isinstance(P2, quaternion):
                # convert quaternion to homogeneous transform
                T2 = [P2.tr()]
            elif isrot(P2):
                # convert rotation matrix to homogeneous transform
                T2 = [r2t(P2)]
            elif ishomog(P2):
                # already a homogeneous transform
                T2 = [P2]
            else:
                error('tranimate: expected quaternion, rotation or homogeoneos transforms')

    # at this point
    #   T1 is the initial pose
    #   T2 is the final pose
    #
    # T2 may be a sequence

    if len(T2) > 1:
        # tranimate(Ts)
        # we were passed a homogenous sequence
        Ttraj = T2
        nsteps = len(Ttraj)
    else:
        # tranimate(P1, P2)
        # create a path between them
        Ttraj = ctraj(T1, T2[0], nsteps)

    fig = trplot(eye(4, 4), fig=None, **opts)  # create a frame at the origin

    # If recording, assign an animation writer

    Writer = None
    if rec != 0:
        try:
            Writer = animation.writers['ffmpeg']
        except (KeyError, RuntimeError):
            try:
                Writer = animation.writers['avconv']
            except (KeyError, RuntimeError):
                print("Cannot record animation, no animation writers available!")
                print("Try installing ffmpeg or avconv.")
                rec = 0

    # Specify animation parameters and assign functions.

    nframes=nsteps
    blit = False
    if rec == 0:
        blit = True

    anim = animation.FuncAnimation(fig, _tranim3d, fargs=(Ttraj, tranim_lines3D),
                                   init_func=None,
                                   frames=nframes, blit=blit,
                                   interval=fps, repeat=False)

    # Begin

    if not (rec == 0 or Writer is None):
        plt_ion = plt.isinteractive()
        if plt_ion: plt.ioff()
        print("Creating animation video; presentation will begin shortly ...")
        writer = Writer(fps=fps,
                        extra_args=['-an'],
                        metadata=dict(artist='RTB -- tranimate'),
                        bitrate=-1)
        filename = 'tranimate' + '.mp4'
        anim.save(filename, writer=writer)
        print("Animation saved in file %s" % filename)
        if plt_ion: plt.ion()

    if __name__ == '__main__' and not plot_isinteractive:
        plt.show(block=True)
    else:
        plt.show(block=False)

###
### Main for quick functional checks
###

if __name__ == '__main__':

    print("matplotlib backend: %s" % mpl.get_backend())
    print("Interactive = %s" % plot_isinteractive)

    # Note: In interactive mode hold is default to 'on'.

    from robot.puma560 import *
    from robot.fourlink2d import *

    print("Each of the plotting functional checks will create a plot")
    print("figure window which must be closed for the check sequence")
    print("to continue.")

    # check trplot
    print("Checking trplot.")
    trplot(rotx(0.2), title='trplot')
    print("Done.")
    plt.show(block=True)
    
    # check tranimate
    print("Checking tranimate.")
    tranimate(rotx(0.5), None, nsteps=20, title='tranimate')
    print("Done.")
    
    # check tranimation recording
    print("Checking tranimate recording.")
    tranimate(rotx(0.5), None, nsteps=20, rec=1, title='tranimate recording')
    print("Done.")
    print("Interactive = %s" % plot_isinteractive)

    # generate a 1x2 joint space trajectory starting with fourlink2d's nominal
    # angle joint array qz
    t = np.arange(0, 2.05, 0.05)                        # generate a time vector
    (Q, _, _) = jtraj(qn, [pi/4, pi/4, pi/4, pi/4], t)  # generate joint coords trajectory

    # check rbplot 2D using fourlink2d
    # tl.handle['p3D']=0 is value set in robot/fourlink2d.py
    print("Checking rbplot with 2D four-link robot.")
    rbplot(fl2d, Q)
    print("Done.")

    # generate a 1x6 joint space trajectory starting with puma560's zero
    # angle joint array qz_s (note: fourlink's qz replaced puma560's qz)
    t = np.arange(0, 2.05, 0.05)    # generate a time vector
    (Q, _, _) = jtraj(qz_s, qr, t)  # generate joint coords trajectory

    # check qplot

    print("Checking qplot invoked with one argument.")
    if plot_isinteractive: clf(); plt.clf()
    qplot(Q, title='Invoked as qplot(q)')
    print("Done.")
    print("Checking qplot invoked with two arguments.")
    if plot_isinteractive: clf(); plt.clf()
    qplot(t, Q, title='Invoked as qplot(t,q)')
    print("Done.")

    # check rbplot 3D using puma560
    # p560.handle['p3D']=1 is value set in robot/puma560.p; also Robot init default
    print("Checking rbplot with 3D Puma 560 robot.")
    rbplot(p560, Q)
    print("Done.")

    # check rbplot 3D with two puma560 robots
    # Note: Only works if robot.plot imported after pylab.
    if plot_isinteractive:
        p560_1 = p560.copy()
        p560_1.name = "Puma 560 #1"
        p560_2 = p560.copy()
        p560_2.name = "Puma 560 #2"
        p560_2.base = transl(-0.5, 0.5, 0.0)
        print("Checking rbplot with 3D Puma 560 robots #1 and #2.")
        rbplot(p560_1, Q)
        rbplot(p560_2, Q, phold=True)
        print("Done.")

    # check rbplot animation recording
    print("Checking rbplot recording.")
    rbplot(p560, Q, rec=1)
    print("Done.")
    print("Interactive = %s" % plot_isinteractive)

    plt.close('all')
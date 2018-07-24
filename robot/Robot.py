"""
Robot object.

Python implementation by: Luis Fernando Lara Tobar and Peter Corke.
Based on original Robotics Toolbox for Matlab code by Peter Corke.
Permission to use and copy is granted provided that acknowledgement
of the authors is made.

@author: Luis Fernando Lara Tobar and Peter Corke
"""

from numpy import *
from .utility import *
from .transform import *
from .Link import *
from math import atan, cos, sin, floor
import copy
import random

class Robot(object):
    """Robot object.
    Instances of this class represent a robot manipulator
    within the toolbox.
    """

    def __init__(self, arg=None, gravity=None, base=None, tool=None, name=' ', comment=' ', manuf=' '):
        """
        Robot object constructor.  Create a robot from a sequence of Link objects.

        Several basic forms exist:
            - Robot()        create a null robot
            - Robot(robot)   create a clone of the robot object
            - Robot(links)   create a robot based on the passed links

        Various options can be set using named arguments:

            - gravity; gravitational acceleration (default=[0,0,9.81])
            - base; base transform (default 0)
            - tool; tool transform (default 0)
            - name
            - comment
            - manuf
        """

        if isinstance(arg, Robot):
            for k,v in arg.__dict__.items():
                if k == 'links':
                    self.__dict__[k] = copy.copy(v)
                elif k == 'handle':
                    self.__dict__[k] = copy.deepcopy(v)
                elif k == 'plotopt':
                    self.__dict__[k] = copy.deepcopy(v)
                else:
                    self.__dict__[k] = v
        elif arg is not None and len(arg) > 1 and isinstance(arg[0], Link):
            self.links = arg
        else:
            raise AttributeError

        # fill in default base and gravity direction
        if gravity != None:
            self.gravity = gravity
        else:
            self.gravity = [0, 0, 9.81]

        if base != None:
            self.base = base
        else:
            self.base = mat(eye(4,4))

        if tool != None:
            self.tool = tool
        else:
            self.tool = mat(eye(4,4))

        if manuf:
            self.manuf = manuf
        if comment:
            self.comment = comment
        if name:
            self.name = name

        self.q = zeros(self.n)

        self.handle = dict(p3D=1, fps=10, rec=0, button=0,
                           x0=[], y0=[], z0=[], zmin=0.0, xyzTgt=[], mag=1.0)
        self.plotopt = dict(xlim=[-1.0, 1.0], ylim=[-1.0, 1.0], zlim=[-1.0, 1.0])

        #self.lineopt = {'Color', 'black', 'Linewidth', 4}
        #self.shadowopt = {'Color', 'black', 'Linewidth', 1}

    def __str__(self):
        s = 'ROBOT(%s, %s)' % (self.name, self.config())
        return s

    def __repr__(self):
        s = ''
        if self.name:
            s += 'name: %s\n' % (self.name)
        if self.manuf:
            s += 'manufacturer: %s\n' % (self.manuf)
        if self.comment:
            s += 'commment: %s\n' % (self.comment)

        for link in self.links:
            s += str(link) + '\n'
        return s

    def __mul__(self, r2):
        r = Robot(self)        # clone the robot
        r.links = r.links + r2.links
        return r

    def copy(self):
        """
        Return a copy of this Robot object
        """
        cpy = copy.copy(self)
        cpy.links = copy.copy(self.links)
        cpy.handle = copy.deepcopy(self.handle)
        cpy.plotopt = copy.deepcopy(self.plotopt)
        return cpy

    def ismdh(self):
        return self.mdh

    def config(self):
        """
        Return a configuration string, one character per joint, which is
        either R for a revolute joint or P for a prismatic joint.
        For the Puma560 the string is 'RRRRRR', for the Stanford arm it is 'RRPRRR'.
        """
        s = ''

        for link in self.links:
            if link.sigma == 0:
                s += 'R'
            else:
                s += 'P'
        return s

    def nofriction(self, all=False):
        """
        Return a Robot object where all friction parameters are zero.
        Useful to speed up the performance of forward dynamics calculations.

        @type all: boolean
        @param all: if True then also zero viscous friction
        @see: L{Link.nofriction}
        """
        r = Robot(self)
        r.name += "-nf"
        newlinks = []
        for oldlink in self.links:
            newlinks.append( oldlink.nofriction(all) )
        r.links = newlinks
        return r

    def showlinks(self):
        """
        Shows details of all link parameters for this robot object, including
        inertial parameters.
        """

        count = 1
        if self.name:
            print('name: %s'%(self.name))
        if self.manuf:
            print('manufacturer: %s'%(self.manuf))
        if self.comment:
            print('commment: %s'%(self.comment))
        for l in self.links:
            print('Link %d------------------------' % count)
            l.display()
            count += 1
    
    def set_handle(self, key, val):
        if key in self.handle:
            self.handle[key] = val
             
    def get_handle(self, key):
        val = None
        if key in self.handle:
            val = self.handle[key] 
        return val

    def set_plotopt(self, key, val):
        if key in self.plotopt:
            self.plotopt[key] = val

    def get_plotopt(self, key):
        val = None
        if key in self.plotopt:
            val = self.plotopt[key]
        return val

    def mouse_click(self, b, x, y):
        """
        Animation - store button and (x,y) states for mouse click
        """
        self.set_handle('button', b)
        if self.get_handle('button') == 1:  # right button
            xyzTgt = zeros(3)
            if self.get_handle('p3D') == 0:
                xyzTgt[0] = x
                xyzTgt[1] = y
            else :
                theta = 0.45*pi*random.random()
                r     = random.uniform(2,4)
                xyzTgt[0] = r*cos(theta)
                xyzTgt[1] = r*sin(theta)
                xyzTgt[2] = random.uniform(1,3)
            self.set_handle('xyzTgt', xyzTgt.copy())

      
    def onclick(self, event):
        """
        Animation - mouse button pressed handler
        """
        b = event.button
        x = event.xdata
        y = event.ydata
        self.mouse_click(b, x, y)
        
    def __setattr__(self, name, value):
        """
        Set attributes of the robot object

            - robot.name = string (name of this robot)
            - robot.comment = string (user comment)
            - robot.manuf = string (who built it)
            - robot.tool = 4x4 homogeneous tranform
            - robot.base = 4x4 homogeneous tranform
            - robot.gravity = 3-vector  (gx,gy,gz)
            - robot.q = n-array
            - robot.handle = dict
            - robot.plotopt = dict

        """

        if name in ["manuf", "name", "comment"]:
            if not isinstance(value, str):
                raise ValueError('must be a string')
            self.__dict__[name] = value

        elif name == "links":
            if not isinstance(value[0], Link):
                raise ValueError('not a Link object')
            self.__dict__[name] = value
            self.__dict__['n'] = len(value)
            # set the robot object mdh status flag
            for link in self.links:
                if link.convention != self.links[0].convention:
                    raise 'robot has mixed D&H link conventions'
            self.__dict__['mdh'] = self.links[0].convention == Link.LINK_MDH

        elif name == "tool":
            if not ishomog(value):
                raise ValueError('tool must be a homogeneous transform')
            self.__dict__[name] = value

        elif name == "gravity":
            v = arg2array(value)
            if len(v) != 3:
                raise ValueError('gravity must be a 3-vector')
            self.__dict__[name] = mat(v).T

        elif name == "base":
            if not ishomog(value):
                raise ValueError('base must be a homogeneous transform')
            self.__dict__[name] = value

        elif name == "q":
            v = arg2array(value)
            if len(v) != self.__dict__['n']:
                raise ValueError('q must be an n-array')
            self.__dict__[name] = value

        elif name == "handle":
            if not isinstance(value, dict):
                raise ValueError('handle must be a dictionary')
            self.__dict__[name] = value

        elif name == "plotopt":
            if not isinstance(value, dict):
                raise ValueError('plotopt must be a dictionary')
            self.__dict__[name] = value

        else:
            raise AttributeError

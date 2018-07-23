"""
Robotics Toolbox for Python test script parser.
"""

from numpy import *
from pylab import *
from robot import *
from robot.utility import *
from robot.dynamics import *
from robot.trajectory import *
import traceback
import sys

def testparser(tests):

    set_printoptions(precision=5, suppress=True);

    pecho = False;
    
    def assertEqual(a,b):
        assert a.all() == b.all(), 'The two values are not equal.'
        
    def assertNotEqual(a,b):
        assert a.all() != b.all(), 'The two values are equal.'
        
    for line in tests.split('\n'):

        line = str.rstrip(line);
    
        if str.rfind(line, 'quit', 0, len('quit')) == 0:
            break
        if str.rfind(line, 'echo', 0, len('echo')) == 0:
            if str.rfind(line, 'echo on', 0, len('echo on')) == 0:
                pecho = True
            elif str.rfind(line, 'echo off', 0, len('echo off')) == 0:
                pecho = False
            elif len(line) == len('echo') :
                pecho = not pecho          
            continue   
        if len(line) == 0 or line[0] in '%#':
            if len(line) > 1 and line[1] in '%#':
                # always print special '%%" or '##' comment
                print('%s' % line)
                continue
            if pecho:
                # only print regular comments if echo 'on'
                print('%s' % line)
            continue

        if pecho : print(':: %s' % line)
    
        try:
            if str.rfind(line, 'from ', 0, len('from ')) == 0:
                exec(line)
                continue
            if ' = ' in line:
                e = line.split(' = ')
                exec(line)
                if line[-1] != ';':
                    for v in e[0].split(','):  # handle value lists or tuples
                        v = v.lstrip('[( ').rstrip(' )]')
                        if len(v) > 0 and v != '_':
                            print('%s =' % v)
                            result = '%s' % eval(v)
                            if result.find('array(') or result.find('matrix(') :
                                # handle numpy array and matrix
                                result = str.replace(result, ']), ',']),\n')
                                result = str.replace(result, "'", '"')
                            print('%s' % result)
                    print('')
            else:
                result = eval(line.rstrip(';'))
                if result is not None and line[-1] != ';':
                    if isinstance(result, (list, tuple)):
                        vals = ""
                        for val in result:
                            vals = vals + " " + "%s" % val
                    else:
                        vals = '%s' % result
                    if vals.find('array(') or vals.find('matrix(') :
                         # handle numpy array and matrix
                         vals = str.replace(vals, ']), ',']),\n')
                         vals = str.replace(vals, "'", '"')
                    print('%s =\n%s' % (line.rstrip(':'), vals))
                    print('')
        except AssertionError as e:
            print('AssertionError: %s' % e)
        except ValueError as e:
            print('ValueError: %s' % e)
        except NameError as e:
            print('NameError: %s' % e)
        except:
            print('Error at line <%s>' % line)
            traceback.print_exc()
            sys.exit(1)


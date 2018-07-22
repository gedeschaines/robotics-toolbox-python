"""
Robotics Toolbox for Python demo script parser
"""

from numpy import *
from pylab import *
from robot import *
from robot.utility import *
from robot.dynamics import *
from robot.trajectory import *
from robot.plot import *
from inspect import isfunction
import textwrap
import traceback
import sys
import re


set_printoptions(precision=5, suppress=True);

# handle Python 2.x and 3.x raw_input
try: input = raw_input
except NameError: pass

# we use termios to pretty up the continuation prompting, only available under Unix
has_termios = False
try:
    import termios;
    has_termios = True
except:
    pass


def parsedemo(s):
    """
    Helper to write demo files, based on minimum change from their Matlab format.
    The string contains
      - help text which is left justified and displayed directly to the screen
      - indented text is interpreted as a command, executed, and the output sent
        to screen
        
    The argument is Matlab code as per the demo files rtXXdemo.
    
    @note: Requires some minor changes to the demo file.
    @type s: string
    @param s: Demo string.
    """
    rstrip = re.compile(r'''\s*%.*$''')
    lines = s.split('\n')
    
    pecho = True
    
    name = __file__
    print('%s' % name)
    print('%s' % '='*len(name))
    if has_termios:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        new = termios.tcgetattr(fd)
        new[3] = new[3] & ~termios.ECHO      # lflags
    try:
        if has_termios:
            termios.tcsetattr(fd, termios.TCSADRAIN, new)
            
        text = ''
        for line in lines:
            if len(line.strip()) == 0:
                if pecho: print('')
            elif line[0] == '#':
                ## help text found
                
                if len(line) == 1 and text:
                    # a blank line means paragraph break, print help text string
                    if pecho: print('%s' % textwrap.fill(text, fix_sentence_endings=True))
                    text = ''
                    if pecho: print('') 
                elif len(line) > 1 and line[1] == '#':
                    # comment found; print help text string, then the comment
                    if text:
                        print('%s' % textwrap.fill(text, fix_sentence_endings=True))
                        text = ''
                    print('%s' % line)
                else:
                    # add line to help text string, minus the leading '#'
                    text += line[1:].lstrip() + '\n'
 
            else:
                ## command encountered
                
                # flush the remaining help text
                if text:
                    if pecho: print('%s' % textwrap.fill(text, fix_sentence_endings=True))
                    text = ''
                
                cmd = line.strip()
                
                # special case, quit
                if cmd.startswith('quit'):
                    break
                    
                # special case, echo
                if cmd.startswith('echo'):
                    e = cmd.split(' ')
                    if len(e) == 1 :
                       pecho = not pecho
                    elif e[1].startswith('on'):
                       pecho = True
                    elif e[1].startswith('off'):
                       pecho = False
                    continue;
                    
                # special case, pause prompts the user to continue
                if cmd.startswith('pause'):
                    if pecho: print('%s' % cmd)
                    # prompt for continuation
                    sys.stderr.write('more? ')
                    input();                  
                    # remove the prompt from the screen
                    sys.stderr.write('\r        \r');
                    continue;
                    
                if pecho: print('')
                    
                # show the command we are about to execute
                if pecho: print('>>> %s' % cmd)
                
                # if it involves an assignment then we use exec else use eval.
                # we mimic the matlab behaviour in which a trailing semicolon inhibits
                # display of the result
                try:
                    if cmd.startswith('from'):
                        exec(cmd)
                    elif cmd.startswith('help('):
                        # This may spawn a Python shell to display the help 
                        # text, pressing the 'q' key will exit help shell.
                        try:
                            eval(cmd)
                        except NameError as e:
                            print('NameError: %s' % e)
                        except TypeError as e:
                            print('TypeError: %s' % e)
                    elif ' = ' in cmd:
                        e = cmd.split('=')
                        rscmd = rstrip.sub('', cmd)
                        try:
                            exec(rscmd.rstrip(';'))
                            if pecho and rscmd[-1] != ';':
                                for v in e[0].split(','):  # handle value lists or tuples
                                    v = v.lstrip('[( ').rstrip(' )]')
                                    print('%s =' % v)
                                    print('%s' % eval(v))
                        except ValueError as e:
                            print('ValueError: %s' % e)
                        except NameError as e:
                            print('NameError: %s' % e)     
                    else:
                        rscmd = rstrip.sub('', cmd)
                        try:
                            result = eval(rscmd.rstrip(';'))
                            if result is not None and pecho and rscmd[-1] != ';':
                                result = '%s' % (result)
                                if result.find('array(') or result.find('matrix(') :
                                    # handle numpy array and matrix
                                    result = str.replace(result, ']), ',']),\n')
                                    result = str.replace(result, "'", '"')
                                print('%s' % result)
                        except ValueError as e:
                            print('ValueError: %s' % e)
                        except NameError as e:
                            print('NameError: %s' % e)
                except:
                    print('Error at line <%s>' % cmd)
                    traceback.print_exc();
                    sys.exit(1);

    finally:
        if has_termios:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            

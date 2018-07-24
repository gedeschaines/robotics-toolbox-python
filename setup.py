import os
from setuptools import setup  #, find_packages

#with open(os.path.join(os.path.dirname(__file__), 'README.md')) as readme:
#    README = readme.read()

# allow setup.py to be run from any path
os.chdir(os.path.normpath(os.path.join(os.path.abspath(__file__), os.pardir)))

setup(
    name = 'robotics-toolbox-python-update1',
    # Important:    v- keep a space here so getvers.sh script can extract version number.
    version = '1.0'  ,
    packages = ['robot'],
    data_files = [('docs',  ['docs/show_video.html']),
                  ('docs/images', ['docs/images/Puma_560.jpg',
                                   'docs/images/Puma_560.mp4']),
                  ('demo', ['demo/README',
                            'demo/_robot.py',
                            'demo/animation.py',
                            'demo/fdyn.py',
                            'demo/fkine.py',
                            'demo/idyn.py',
                            'demo/ikine.py',
                            'demo/jacobian.py',
                            'demo/my_robot.py'
                            'demo/trajectory.py',
                            'demo/traj.py',
                            'demo/transform.py']),
                  ('test', ['test/README',
                            'test/_robot.py',
                            'test/test',
                            'test/arm.py',
                            'test/dynamics.py',
                            'test/jacobian.py',
                            'test/kinematics.py',
                            'test/kinematics_test.py'
                            'test/kine_fl2d.py',
                            'test/kine_fl3d.py',
                            'test/kine_phanx.py',
                            'test/kine_p560m.py',
                            'test/kine_p560s.py',
                            'test/kine_stanf.py',
                            'test/manip.py',
                            'test/quaternion.py',
                            'test/test_rbplot2d.py',
                            'test/test_rbplot3d.py',
                            'test/test-transform.py',
                            'test/test-wtrans.py',
                            'test/trajectory.py',
                            'test/transform.py']),
                  ('test/compare', ['test/compare/compare.py',
                                    'test/compare/genpath.m',
                                    'test/compare/path.dat',
                                    'test/compare/puma560.dat'
                                    'test/compare/puma560m.dat'])],

    author = 'Peter I. Corke',
    author_email = '',
    maintainer = 'Gary Deschaines',
    maintainer_email = '',

    url = 'http://petercorke.com/wordpress/toolboxes/support-developer',
    download_url = 'https://github.com/gedeschaines/robotics-toolbox-python/tree/update1',

    install_requires = ['numpy', 'matplotlib'],

    include_package_data = True,
    license = 'LGPLv3',
    description = 'Robotics Toolbox for Python - Update 1.',
    #long_description = README,
)

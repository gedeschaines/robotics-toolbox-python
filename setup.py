import os
from setuptools import setup  #, find_packages

#with open(os.path.join(os.path.dirname(__file__), 'README.md')) as readme:
#    README = readme.read()

# allow setup.py to be run from any path
os.chdir(os.path.normpath(os.path.join(os.path.abspath(__file__), os.pardir)))

setup(
    name = 'robotics-toolbox-python-update1',
    version = '1.0',
    packages = ['robot'],

    author = 'Peter I. Corke',
    author_email = '',
    maintainer = 'Gary Deschaines',
    maintainer_email = '',

    url = 'http://petercorke.com/wordpress/toolboxes/support-developer',
    download_url = 'https://github.com/gedeschaines/robotics-toolbox-python/tree/update1',

    install_requires = ['numpy', 'matplotlib'],

    include_package_data = True,
    license = 'LGPLv3',
    description = "Robotics Toolbox for Python - Update 1.",
    #long_description = README,
)

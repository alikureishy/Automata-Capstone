# ! DO NOT MANUALLY INVOKE THIS setup.py SCRIPT. USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['shared_utils'],
    package_dir={'': 'include'}
)
setup(**setup_args)
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rosnode_killer'],
    package_dir={'': 'src'},
    scripts=['scripts/rosnode_killer'],
    requires=['genmsg', 'genpy', 'roslib', 'rospkg']
)

setup(**d)

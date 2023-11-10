"""Setupdatei für ROS."""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['localisation'],
    package_dir={'': 'src'}
)

setup(**d)

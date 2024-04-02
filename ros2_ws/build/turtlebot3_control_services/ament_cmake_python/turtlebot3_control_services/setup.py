from setuptools import find_packages
from setuptools import setup

setup(
    name='turtlebot3_control_services',
    version='0.0.0',
    packages=find_packages(
        include=('turtlebot3_control_services', 'turtlebot3_control_services.*')),
)

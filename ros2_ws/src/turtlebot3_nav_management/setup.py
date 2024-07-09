from setuptools import find_packages, setup
import os, glob

package_name = 'turtlebot3_nav_management'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch directory
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
        # Include config directory
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marvin',
    maintainer_email='kohnen.marvin@googlemail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_navigation_using_nav2 = turtlebot3_nav_management.autonomous_navigation_using_nav2:main',
            'custom_navigation = turtlebot3_nav_management.custom_navigation:main',
            'teleop_navigation_logger = turtlebot3_nav_management.teleop_navigation:main',
        ],
    },
)

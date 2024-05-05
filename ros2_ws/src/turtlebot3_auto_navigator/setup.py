from setuptools import find_packages, setup
import os

package_name = 'turtlebot3_auto_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'auto_navigator = turtlebot3_auto_navigator.auto_navigator:main',
            ],
    },
)

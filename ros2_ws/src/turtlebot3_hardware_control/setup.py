from setuptools import setup, find_packages

package_name = 'turtlebot3_hardware_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # Automatically find all packages
    data_files=[
        ('share/' + package_name, ['package.xml']),  # Include the package XML for ROS2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name])
    ],
    install_requires=['setuptools'],  # Ensure setuptools is included as a requirement
    zip_safe=True,  # Typically set to True unless you have a specific reason to use False
    maintainer='marvin',
    maintainer_email='marvin@todo.todo',  # Update with your actual email
    description='A ROS2 package for controlling TurtleBot3 hardware',
    license='TODO: License declaration',  # Update with your actual license
    tests_require=['pytest'],  # Dependencies for running tests
    entry_points={  # Define entry points for the package
        'console_scripts': [
            'hardware_control = turtlebot3_hardware_control.hardware_control:main',  # Maps the command to the function
        ],
    },
)

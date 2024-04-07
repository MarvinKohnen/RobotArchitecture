from setuptools import find_packages, setup

package_name = 'turtlebot3_map_change_detection'

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
            'map_change_detector = map_change_detection.map_change_detector:main'
        ],
    },
)

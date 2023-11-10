from setuptools import setup

package_name = 'nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rama',
    maintainer_email='rama@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'kf_depthCamera = nodes.kf_depthCamera',
        'test_node = nodes.test_node:main',
        'Handgesture_node = nodes.hand_gesture_recognition:main',
        'obstacle_avoidance_node = nodes.obstacle_avoidance:main',
        ],
    },
)

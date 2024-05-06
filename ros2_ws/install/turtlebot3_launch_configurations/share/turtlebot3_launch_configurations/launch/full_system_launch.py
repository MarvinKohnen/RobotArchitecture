from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_hardware_control',
            executable='hardware_control',
            name='hardware_control',
            output='screen'
        ),
        Node(
            package='turtlebot3_obstacle_avoidance',
            executable='obstacle_avoidance',
            name='obstacle_avoidance',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot3_slam_management'), 
                'launch', 
                'slam_launch.py'
            ])),
            launch_arguments={'use_sim_time': 'True'}.items(),
        ),
        Node(
            package='turtlebot3_random_explorer',
            executable='random_explorer',
            name='random_explorer',
            output='screen'
        ),
        Node(
            package='turtlebot3_map_change_detection',
            executable='map_change_detector',
            name='map_change_detector',
            output='screen'
        )
    ])

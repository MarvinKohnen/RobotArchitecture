from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('turtlebot3_slam_management')
    config_dir = os.path.join(pkg_share, 'config')
    slam_params_file = os.path.join(config_dir, 'slam_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',  # This default can be overridden at launch
            description='Use simulation time'),
            
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
    ])

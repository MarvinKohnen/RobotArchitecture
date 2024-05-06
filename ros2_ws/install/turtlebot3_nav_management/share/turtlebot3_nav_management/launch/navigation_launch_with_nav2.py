from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    package_dir = get_package_share_directory('turtlebot3_nav_management')
    config_dir = os.path.join(package_dir, 'config')
    nav2_config_file = os.path.join(config_dir, 'navigation_params.yaml')

    return LaunchDescription([
    
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',  # This default can be overridden at launch
            description='Use simulation time'),
            
        # Map Server Node
        Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[nav2_config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # AMCL Node for Localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            output='screen',
            parameters=[nav2_config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Planner Server Node
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[nav2_config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Controller Server Node
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
         # Recovery Server Node 
        Node(
            package='nav2_behaviors',  #RENAMED TO BEHAVIORS BUT I CANT FIX IT
            executable='behavior_server',
            output='screen',
            parameters=[nav2_config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
       
        # Behavior Tree Navigator Node
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[nav2_config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        # Lifecycle Manager Node
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'autostart': True},
                        {'node_names': ['map_server', 
                                        'amcl',
                                        'planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator']}],
        ),
    ])


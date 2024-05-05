from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare global parameters
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock if true')
    
    # Declare the map_path launch argument
    declare_map_path_arg = DeclareLaunchArgument(
        'map_path', default_value='~/RobotArchitecture/ros2_ws/src/Maps/',
        description='Full path to the map file (in yaml format).')

    random_goal_navigator_node = Node(
        package='turtlebot3_nav_management',
        executable='random_goal_navigator',
        name='random_goal_navigator',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'map_path': LaunchConfiguration('map_path')}]
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_map_path_arg,
        random_goal_navigator_node,
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_nav_management',
            executable='autonomous_navigation_using_nav2',
            name='autonomous_navigation',
            output='screen',
            parameters=[
                {'start_x': -2.0},
                {'start_y': -0.5},
                {'goal_x': 1.5},
                {'goal_y': 1.5}
            ]
        )
    ])

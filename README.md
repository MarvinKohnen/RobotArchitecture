# RobotArchitecture
Bachelorthesis for my Computer Science Bachelor. Supervised by Prof. Dr. Malte Schilling at the WWU Münster. Wintersemester 2023/24 


# Requirements
Ubuntu 22.04 LTS

ROS2 Humble

Gazebo 11.10.2

Python 3.10.12


# Install

Some of the following information is deducted from the following Tutorial:

https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/



### Global

Required ros2 humble packages:

`sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-cartographer ros-humble-cartographer-ros ros-humble-navigation2 ros-humble-na2-bringup`

Required turtlebot3 packages:

`sudo apt install ros-humble-turtlebot3*` 

`sudo apt-get install ros-humble-tf-transformations`

`sudo apt install ros-humble-dynamixel-sdk`

Required Python packages:

`pip install numpy opencv-python PyYAML`



### In Root of Workspace 

`git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3`

`git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations`


**build via**

`colcon build --symlink-install`

*or (if you experience high resource constraint)*

`colcon build --symlink-install --parallel-workers 1`

# Usage

### Start Simulation

`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`


### Real Robot

#### Bringup

ssh to Turtlebot3

`export TURTLEBOT3_MODEL=burger`

`ros2 launch turtlebot3_bringup robot.launch.py`

see https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup


### Access Layers (preferably in this order)

`ros2 run turtlebot3_hardware_control hardware_control`

`ros2 run turtlebot3_obstacle_avoidance obstacle avoidance`

`ros2 launch turtlebot3_slam_management slam_launch.py`

`ros2 run turtlebot3_random_explorer random_explorer`

`ros2 run turtlebot3_map_change_detection map_change_detector`



*or for a full system launch (not recommended for debugging/ initial use)*

`ros2 launch turtlebot3_launch_configurations full_system_launch.py`



# Miscellaneous

- Swap between Navigational Systems in map_change_detector.py line 62/63
- That being said: **Custom navigation in its current form is not functional!!** 
- TODOs for improvement:
    - Improve Hardware control
        - refining movement functions (currently time based)
        - swap Robot Control Service to Action Server
    - Implement actual Pathfinding in Custom Navigation (by means of an A* algorithm i.e.) 
    - Mapping of Real World presented by Slam Toolbox and /map topic does not match Robots estimated pose in /odom which results in navigational systems failing in real world application

















# Bachlorarbeit - Potential Field

## Launch Befehle zum testen:
verschiedene Welten: <br>
roslaunch turtlebot3_gazebo turtlebot3_world.launch <br>
roslaunch turtlebot3_gazebo turtlebot3_house.launch <br>
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

SLAM: <br>
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

Localisation: <br>
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/bachlorarbeit/localisation/map.yaml

RVIZ: <br>
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

roslaunch controller controller_launch.launch



simples APF testen: <br>
roslaunch map_obstacles avoidance_world.launch 
<br>
roslaunch potential_field test_durchführen.launch obstacle_vel:=0 
<br>
rosservice call /gazebo/reset_world && rosservice call /gazebo/reset_world



Bekannte Fehler:
- Objekte laufen in der Simulation auf unterschiedlichen Geräten signifikant anders
- python pip3 Pakete updaten könnten manche simplen Fehler beheben:
  - pip install -U matplotlib

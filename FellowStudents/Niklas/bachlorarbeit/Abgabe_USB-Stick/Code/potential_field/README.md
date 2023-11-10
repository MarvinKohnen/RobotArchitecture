# Potential Field + Extensions

## Launch Befehle zum testen der Gazebo Simulation:
1. Generate world
```sh
roslaunch map_obstacles avoidance_world.launch
```
2. Let obstacle move with a specific velocity
```sh
roslaunch potential_field test_durchführen.launch obstacle_vel:=0
```
3. Start robot and track it -> adjust used model in launch file
```sh
roslaunch potential_field start_moving_and_recording.launch
```
3. Reset world to test again
```sh
rosservice call /gazebo/reset_world && rosservice call /gazebo/reset_world
```


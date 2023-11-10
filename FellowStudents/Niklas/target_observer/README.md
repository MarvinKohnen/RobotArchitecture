# Target Obeserver

## Beschreibung
Dieses Packet soll ein Ziel mittels Kamera erkennen und  dessen Bewegung verfolgen.
-> Model: realsense d435i

## Hardware

### Installiere (für Hardware):
```
sudo apt-get install ros-noetic-realsense2-camera
sudo apt-get install ros-noetic-realsense2-description
```
(+ ggf. openCV, allerdings sollte dies bereits durch ros-noetic-full geschehen sein)



### Testen (für Hardware):
Pointcloud (Rviz):    
```
roslaunch realsense2_camera demo_pointcloud.launch
```
'Normale Kamera':     
```
rosrun image_view image_view image:=/camera/color/image_raw/
```
Tiefen Kamera:        
```
rosrun image_view image_view image:=/camera/depth/image_rect_raw
```


# Simuation

### Installiere (für Simulation):
```
git clone git@github.com:issaiass/realsense2_description.git
git clone git@github.com:pal-robotics/realsense_gazebo_plugin.git
```



### Testen (für Simulation):
```
roslaunch realsense2_description view_d435_model_rviz_gazebo.launch
```
```
rosrun visual_processing testen.py
```


## Hinweis
Um die RGBD Kamera ausgerichtet nutzen zu wollen empfiehlt es sich den 'rs_aligned_depth.launch' zu nutzen.
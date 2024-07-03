import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/marvin/RobotArchitecture/ros2_ws/install/turtlebot3_teleop'

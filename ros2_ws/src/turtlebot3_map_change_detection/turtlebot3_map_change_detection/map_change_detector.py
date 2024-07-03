import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import os
from datetime import datetime
import subprocess
import threading

class MapChangeDetector(Node):
    def __init__(self):
        super().__init__('map_change_detector')
        self.declare_parameter('map_save_interval', 30)  # in seconds
        self.declare_parameter('map_directory', '~/RobotArchitecture/ros2_ws/src/Maps/')  
        self.map_save_interval = self.get_parameter('map_save_interval').value
        self.map_directory = os.path.expanduser(self.get_parameter('map_directory').value)
        self.map_save_timer = self.create_timer(self.map_save_interval, self.map_save_callback)
        self.last_map_path = ""
        self.declare_parameter('threshold', 10)  # Threshold for changes
        self.lock = threading.Lock()
        self.navigation_started = False  # Flag to track if navigation has been started

    def map_save_callback(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        map_path = os.path.join(self.map_directory, timestamp)
        subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path], check=True)
        self.get_logger().info(f'Saved map at {map_path}')
        if self.last_map_path:
            self.compare_maps(self.last_map_path, map_path + '.pgm')  
        self.last_map_path = map_path + '.pgm'

    def compare_maps(self, old_map_path, new_map_path):
        old_map = cv2.imread(old_map_path, cv2.IMREAD_GRAYSCALE)
        new_map = cv2.imread(new_map_path, cv2.IMREAD_GRAYSCALE)
        if old_map is not None and new_map is not None and old_map.shape == new_map.shape:
            difference = cv2.absdiff(old_map, new_map)
            _, difference = cv2.threshold(difference, 50, 255, cv2.THRESH_BINARY)
            change_percentage = np.sum(difference) / difference.size * 100  # Convert to percentage
            self.get_logger().info(f'Change percentage: {change_percentage}%')
            if change_percentage > self.get_parameter('threshold').value:
                self.get_logger().info('Major changes detected between maps.')
            #bug handling (weiß nicht wo das herkommt)
            elif change_percentage == "0.0":
                pass
            else:
                self.get_logger().info('Map is good enough. Navigation is taking control over Random Explore.')
                self.start_navigation()
        else:
            self.get_logger().error('Error loading maps or map dimensions do not match.')

    def start_navigation(self):
        with self.lock:
            if self.navigation_started:
                self.get_logger().info('Navigation has already been started. Skipping initialization.')
                return

            try:
                nav_command = [
                    "gnome-terminal", "--", "bash", "-c", 
                    f"ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False map:={self.last_map_path}; exec bash"
                ]
                self.nav_process = subprocess.Popen(nav_command)
                self.get_logger().info('Navigation system started.')
                
                """
                auto_nav_command = [
                    "gnome-terminal", "--", "bash", "-c",
                    "ros2 run turtlebot3_auto_navigator auto_navigator; exec bash"
                ]
                self.auto_nav_process = subprocess.Popen(auto_nav_command)
                self.get_logger().info('Auto navigator started.')
                """

                self.navigation_started = True  # Set the flag to indicate navigation has been started

            except Exception as e:
                self.get_logger().error(f'Failed to start a process: {e}')

def main(args=None):
    rclpy.init(args=args)
    map_change_detector = MapChangeDetector()
    try:
        rclpy.spin(map_change_detector)
    except KeyboardInterrupt:
        pass
    finally:
        map_change_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

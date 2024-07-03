import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

class HeatmapGenerator(Node):
    def __init__(self):
        super().__init__('heatmap_generator')
        self.declare_parameter('update_interval', 20)
        self.declare_parameter('nav_map_path', '~/RobotArchitecture/ros2_ws/src/Maps/20240505_165533.pgm')
        self.update_timer = self.create_timer(self.get_parameter('update_interval').value, self.save_heatmap)
        self.nav_map_path = self.get_nav_map_path()
        self.heatmap_path = self.get_heatmap_path()

        self.nav_map = cv2.imread(self.nav_map_path, cv2.IMREAD_GRAYSCALE)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_position = None
        self.heatmap = np.zeros_like(self.nav_map, dtype=np.uint8)

    def get_robot_architecture_path(self):
        # Get the root directory for RobotArchitecture dynamically
        home_directory = os.path.expanduser('~')  # Gets the user's home directory
        return os.path.join(home_directory, 'RobotArchitecture/ros2_ws/src/')

    def get_heatmap_path(self):
        # Ensure the directory for storing the heatmap exists
        heatmap_directory = os.path.join(self.get_robot_architecture_path(), 'Heatmaps')
        if not os.path.exists(heatmap_directory):
            os.makedirs(heatmap_directory)
        return os.path.join(heatmap_directory, 'heatmap.png')

    def get_nav_map_path(self):
        # Retrieve the path of the navigation map from the parameter
        return os.path.expanduser(self.get_parameter('nav_map_path').value)
    
    def odom_callback(self, msg):
        # Update current position based on odometry data
        self.current_position = msg.pose.pose.position
    
        # Map the position to heatmap coordinates
        x, y = self.position_to_heatmap_coords(self.current_position, self.heatmap.shape)
        self.heatmap[y, x] = min(self.heatmap[y, x] + 2, 255)  # Increment heatmap value at robot's position
       

    def save_heatmap(self):
        # Load the navigation map
        if self.nav_map is None:
            self.get_logger().error('Navigation map could not be loaded.')
            return

        # Normalize the heatmap for blending
        heatmap_normalized = cv2.normalize(self.heatmap, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        heatmap_colored = cv2.applyColorMap(heatmap_normalized, cv2.COLORMAP_JET)
        heatmap_resized = cv2.resize(heatmap_colored, (self.nav_map.shape[1], self.nav_map.shape[0]))

        # Blend the heatmap with the navigation map
        nav_map_color = cv2.cvtColor(self.nav_map, cv2.COLOR_GRAY2BGR)
        blended_image = cv2.addWeighted(nav_map_color, 0.4, heatmap_resized, 0.6, 0)

        # Save the blended image
        cv2.imwrite(self.heatmap_path, blended_image)
        self.get_logger().info(f'Blended heatmap updated and saved to {self.heatmap_path}')

    def position_to_heatmap_coords(self, position, shape):
        # Offset and scale factors
        scale_factor_x = 20  
        scale_factor_y = 20 
        offset_x = 2.96 #59.2  
        offset_y = 2.62 #52.4  

        x = int((position.x + offset_x) * scale_factor_x)
        y = int((position.y + offset_y) * scale_factor_y)

        # Clamp coordinates to heatmap dimensions
        x = max(0, min(shape[1] - 1, x))
        y = max(0, min(shape[0] - 1, y))
        self.get_logger().info(f"Converted position ({position.x}, {position.y}) to heatmap coords ({x}, {y}) within bounds ({shape[1]}, {shape[0]})")
        return x, y

def main(args=None):
    rclpy.init(args=args)
    heatmap_generator = HeatmapGenerator()
    try:
        rclpy.spin(heatmap_generator)
    except KeyboardInterrupt:
        pass
    finally:
        heatmap_generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

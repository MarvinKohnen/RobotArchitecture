import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from turtlebot3_control_services.srv import GetLatestMap, GenerateHeatmap
import numpy as np
import cv2
import os
from datetime import datetime
from std_msgs.msg import Bool

class HeatmapGenerator(Node):
    def __init__(self):
        super().__init__('heatmap_generator')
        self.latest_map_service = self.create_client(GetLatestMap, 'get_latest_map')
        while not self.latest_map_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for GetLatestMap service...')

        self.declare_parameter('heatmap_directory', '~/RobotArchitecture/ros2_ws/src/Heatmaps/')
        self.heatmap_directory = os.path.expanduser(self.get_parameter('heatmap_directory').value)
        os.makedirs(self.heatmap_directory, exist_ok=True)

        self.pose_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.robot_positions = []
        self.map_path = ""
        self.map_origin = [-2.95, -2.57]  # Origin from map.yaml
        self.map_resolution = 0.05        # Resolution from map.yaml

        self.heatmap_service = self.create_service(GenerateHeatmap, 'generate_heatmap', self.generate_heatmap_callback)

        self.current_coord_set = None 

        self.recording_subscriber = self.create_subscription(
            Bool,
            'record_movement',
            self.recording_callback,
            10
        )

        self.record_movement = False  # Flag to control recording

    def recording_callback(self, msg):
        self.record_movement = msg.data

    def odom_callback(self, msg):
        if self.record_movement:
            position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.robot_positions.append(position)

    def get_latest_map(self):
        request = GetLatestMap.Request()
        future = self.latest_map_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.map_path = future.result().map_path
            self.get_logger().info(f'Received map path: {self.map_path}')
        else:
            self.get_logger().error('Failed to call GetLatestMap service')

    def generate_heatmap_callback(self, request, response):
        self.get_logger().info(f'Generating heatmap for coordinate set {request.coordinate_set}...')
        
        self.current_coord_set = request.coordinate_set

        self.generate_heatmap()
        response.success = True
        return response

    def generate_heatmap(self):
        if not self.map_path:
            self.get_logger().error('No map path provided.')
            return

        # Load the map
        map_image = cv2.imread(self.map_path, cv2.IMREAD_GRAYSCALE)
        if map_image is None:
            self.get_logger().error(f'Failed to load map image from {self.map_path}')
            return

        self.get_logger().info(f'Map image dimensions: {map_image.shape}, type: {map_image.dtype}, channels: {map_image.ndim}')

        # Create an overlay for the heatmap
        heatmap = np.zeros_like(map_image, dtype=np.float32)

        # Transform robot positions to pixel coordinates considering map origin and resolution
        for pos in self.robot_positions:
            pixel_x = int((pos[0] - self.map_origin[0]) / self.map_resolution)
            pixel_y = int((pos[1] - self.map_origin[1]) / self.map_resolution)
            if 0 <= pixel_x < heatmap.shape[1] and 0 <= pixel_y < heatmap.shape[0]:
                heatmap[pixel_y, pixel_x] += 1
            else:
                self.get_logger().warning(f'Position {pos} is out of bounds for the heatmap.')

        self.get_logger().info(f'Heatmap dimensions: {heatmap.shape}, type: {heatmap.dtype}, channels: {heatmap.ndim}')

        # Normalize the heatmap
        heatmap = cv2.normalize(heatmap, None, 0, 255, cv2.NORM_MINMAX)
        heatmap = heatmap.astype(np.uint8)

        # Apply a color map to the heatmap
        heatmap_colored = cv2.applyColorMap(heatmap, cv2.COLORMAP_HOT)

        self.get_logger().info(f'Heatmap colored dimensions: {heatmap_colored.shape}, type: {heatmap_colored.dtype}, channels: {heatmap_colored.ndim}')

        # Ensure the map image is converted to three channels if necessary
        if map_image.ndim == 2:
            map_image = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)
        
        # Ensure the map image and heatmap have the same dimensions
        if map_image.shape != heatmap_colored.shape:
            self.get_logger().error('Map image and heatmap dimensions do not match')
            return

        # Combine the heatmap with the original map
        combined = cv2.addWeighted(map_image, 0.6, heatmap_colored, 0.4, 0)

        # Save the combined heatmap image with a timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        heatmap_path = os.path.join(self.heatmap_directory, f'heatmap_set_{self.current_coord_set}_{timestamp}.png')
        self.get_logger().info(f'Saving heatmap at {heatmap_path}')
        cv2.imwrite(heatmap_path, combined)
        self.get_logger().info(f'Saved heatmap at {heatmap_path}')

def main(args=None):
    rclpy.init(args=args)
    heatmap_generator = HeatmapGenerator()
    heatmap_generator.get_latest_map()
    try:
        rclpy.spin(heatmap_generator)
    except KeyboardInterrupt:
        pass
    finally:
        heatmap_generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_srvs.srv import Empty as EmptyService
# Import the SaveMap service
from slam_toolbox.srv import SaveMap


class MapChangeDetector(Node):
    def __init__(self):
        super().__init__('map_change_detector')
        self.declare_parameter('save_interval', 30)  # Interval in seconds between map saves
        self.save_map_service_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.timer = self.create_timer(self.get_parameter('save_interval').value, self.save_map_callback)
        self.previous_map_path = None

    def save_map_callback(self):
        # periodic call to save the map and initiate comparison
        if not self.save_map_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Save map service not available.')
            return
        
        request = SaveMap.Request() 
        future = self.save_map_service_client.call_async(request)
        future.add_done_callback(self.on_map_saved)

    def on_map_saved(self, future):
        # Handle the response from the map saving service, perform comparison with previous map
        # For demonstration, we just log that the map was saved. You'll need to implement comparison logic.
        try:
            response = future.result()
            self.get_logger().info('Map saved successfully.')
            # Implement map comparison logic here
        except Exception as e:
            self.get_logger().error('Failed to save map: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    map_change_detector = MapChangeDetector()
    rclpy.spin(map_change_detector)
    map_change_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

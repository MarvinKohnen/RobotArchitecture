import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from turtlebot3_control_services.srv import RobotControl  

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.robot_control_client = self.create_client(RobotControl, 'robot_control')
        while not self.robot_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Robot control service not available, waiting again...')
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.obstacle_distance = 0.5 
        self.is_turning = False

    def scan_callback(self, msg):
        is_obstacle_in_front = any(
            distance < self.obstacle_distance for distance in msg.ranges[:30] + msg.ranges[-30:])
        if is_obstacle_in_front and not self.is_turning:
            self.is_turning = True
            self.pause_publisher.publish(Bool(data=True))
            self.get_logger().info('Obstacle detected, starting to turn.')
            self.send_command_to_hardware("turn_left", 0.5, 10)  
        elif not is_obstacle_in_front and self.is_turning:
            self.is_turning = False
            self.get_logger().info('Path clear, stopping turn.')
            self.send_command_to_hardware("full_stop", 0.0, 10)
            self.pause_publisher.publish(Bool(data=False))

    def send_command_to_hardware(self, command, value, priority):
        request = RobotControl.Request()
        request.command = command
        request.value = value
        request.priority = priority
        future = self.robot_control_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Command executed: %s' % future.result().message)
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

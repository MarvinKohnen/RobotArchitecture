import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from turtlebot3_control_services.srv import RobotControl  
import random
import time

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.robot_control_client = self.create_client(RobotControl, 'robot_control')
        while not self.robot_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Robot control service not available, waiting again...')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                        depth=1)
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_policy)
        self.obstacle_distance = 0.3
        self.is_turning = False

    def scan_callback(self, msg):
    
        is_obstacle_in_front = any(
            distance < self.obstacle_distance for distance in msg.ranges[:30] + msg.ranges[-30:])
        if is_obstacle_in_front and not self.is_turning:
            self.is_turning = True
            self.get_logger().info('Obstacle detected, stop movement and starting to turn.')
            self.send_command_to_hardware("full_stop", 0.0, 10) 
            self.get_logger().info('Hopefully stopped, now turn.')
            
            direction = random.randint(0, 1)
            # Map integer to direction
            if direction == 0:
                turn_direction = "turn_left"
            else:
                turn_direction = "turn_right"
            self.send_command_to_hardware(turn_direction, 0.5, 10) 
        elif not is_obstacle_in_front and self.is_turning:
            self.is_turning = False
            self.get_logger().info('Path clear, resetting priority.')
            self.send_command_to_hardware("reset_priority", 0.0, 11)

    def send_command_to_hardware(self, command, value, priority):
        request = RobotControl.Request()
        request.command = command
        request.value = value
        request.priority = priority
        future = self.robot_control_client.call_async(request)
        future.add_done_callback(self.handle_service_response)
    
    #needed to be placed in separate function in order to handly multiple asynchronous operations
    def handle_service_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Command executed: {response.message}')
            else:
                self.get_logger().info(f'Command failed: {response.message}')
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % e)
            
def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    #multithreading
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(obstacle_avoidance)

    try:
        executor.spin()
    finally:
        obstacle_avoidance.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

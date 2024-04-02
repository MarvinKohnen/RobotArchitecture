import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Twist

from turtlebot3_control_services.srv import RobotControl  # Update with actual package and service names


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.emergency_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pause_publisher = self.create_publisher(Bool, '/stop_signal', 10)
        self.obstacle_distance = 0.5  # meters
        self.is_turning = False

    def scan_callback(self, msg):
        is_obstacle_in_front = any(
            distance < self.obstacle_distance for distance in msg.ranges[:30] + msg.ranges[-30:])
        
        if is_obstacle_in_front and not self.is_turning:
            self.is_turning = True
            self.pause_publisher.publish(Bool(data=True))
            turn_msg = Twist()
            turn_msg.angular.z = 0.5  
            self.emergency_publisher.publish(turn_msg)
            self.get_logger().info('Obstacle detected, starting to turn.')
        elif not is_obstacle_in_front and self.is_turning:
            # Stop turning
            self.is_turning = False
            stop_msg = Twist() 
            self.emergency_publisher.publish(stop_msg)
            self.pause_publisher.publish(Bool(data=False))  # Signal to resume forward movement
            self.get_logger().info('Path clear, stopping turn.')

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleMover(Node):
    def __init__(self):
        super().__init__('simple_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.move_duration = 5  # Move for 5 seconds
        self.start_time = self.get_clock().now().nanoseconds / 1e9  # Get start time in seconds

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time

        if elapsed_time < self.move_duration:
            self.get_logger().info('Moving forward.')
            msg = Twist()
            msg.linear.x = 0.1  # Move forward
            self.publisher_.publish(msg)
        else:
            self.get_logger().info('Stopping.')
            msg = Twist()  # Stop
            self.publisher_.publish(msg)
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    simple_mover = SimpleMover()

    try:
        rclpy.spin(simple_mover)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

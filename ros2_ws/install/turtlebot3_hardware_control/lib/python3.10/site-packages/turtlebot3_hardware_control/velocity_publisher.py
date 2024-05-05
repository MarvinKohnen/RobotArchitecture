import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import random

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pause_subscription = self.create_subscription(Bool, '/stop_signal', self.pause_callback, 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback) 
        self.is_paused = False

    def pause_callback(self, msg: Bool):
        self.is_paused = msg.data
        if self.is_paused:
            self.get_logger().info("Paused due to obstacle. Awaiting clearance.")
        else:
            self.get_logger().info("Obstacle cleared. Resuming movement.")

    def timer_callback(self):
        if not self.is_paused:
            msg = Twist()
            # Favor forward movement but occasionally turn to explore
            if random.random() > 0.1:
                msg.linear.x = 0.2
                self.get_logger().info('Moving forward.')
            else:
                msg.angular.z = random.choice([-1.0, 1.0])
                self.get_logger().info('Turning randomly.')
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
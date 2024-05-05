import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import random

class AutoNavigator(Node):
    def __init__(self):
        super().__init__('auto_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(10, self.send_random_goal)  # Send a goal every 10 seconds

    def send_random_goal(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Navigation action server not available, waiting...')
            return
        
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = random.uniform(-5, 5)  # Randomize these based on your map's bounds
        goal.pose.pose.position.y = random.uniform(-5, 5)
        goal.pose.pose.orientation.w = 1.0  # Facing straight, typically

        self.get_logger().info(f'Sending new goal: {goal.pose.pose.position.x}, {goal.pose.pose.position.y}')
        self.client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    navigator = AutoNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

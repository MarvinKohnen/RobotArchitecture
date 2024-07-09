import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, GoalResponse
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import random

class AutoNavigator(Node):
    def __init__(self):
        super().__init__('auto_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.active_goal = False  # Flag to track if there is an active goal

    def send_random_goal(self):
        if self.active_goal:
            self.get_logger().info('A goal is still active. Waiting for it to complete...')
            return

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Navigation action server not available, waiting...')
            return
        
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = random.uniform(-5, 5)  
        goal.pose.pose.position.y = random.uniform(-5, 5)
        goal.pose.pose.orientation.w = 1.0  # Facing straight mostly

        self.get_logger().info(f'Sending new goal: {goal.pose.pose.position.x}, {goal.pose.pose.position.y}')
        send_future = self.client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.active_goal = False
            return

        self.get_logger().info('Goal accepted :)')
        self.active_goal = True
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal reached: {result}')
        self.active_goal = False  # Reset the active goal flag when the goal is reached or canceled

def main(args=None):
    rclpy.init(args=args)
    navigator = AutoNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

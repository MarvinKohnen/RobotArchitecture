import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from turtlebot3_control_services.srv import RobotControl
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion
import math

class CustomNavigation(Node):
    def __init__(self):
        super().__init__('custom_navigation')
        self.robot_control_client = self.create_client(RobotControl, 'robot_control')
        while not self.robot_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Robot control service not available, waiting again...')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher to shutdown_random_explorer topic
        self.shutdown_publisher = self.create_publisher(Bool, 'shutdown_random_explorer', 10)

        # Parameters
        self.start_position = {'x': -1.429, 'y': -0.59}
        self.goal_position = {'x': 1.5, 'y': 1.45}
        self.obstacle_detected = False
        self.turn_completed = False

        # Subscriber to obstacle_detected topic
        self.obstacle_subscription = self.create_subscription(
            Bool,
            'obstacle_detected',
            self.obstacle_callback,
            10)

        # Initialize the navigation process
        self.start_navigation()

    def start_navigation(self):
        self.send_command_to_hardware("full_stop", 0.0, 10)
        self.send_command_to_hardware("reset_priority", 0.0, 11)
        self.shutdown_random_explorer()
        self.navigate_to_goal(self.start_position)

    def shutdown_random_explorer(self):
        self.get_logger().info('Shutting down random explorer.')
        msg = Bool()
        msg.data = True
        self.shutdown_publisher.publish(msg)

    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))
            translation = trans.transform.translation
            rotation = trans.transform.rotation
            return (translation.x, translation.y, rotation)
        except Exception as e:
            self.get_logger().error(f'Transform error: {str(e)}')
            return None

    def navigate_to_goal(self, goal):
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().error('Failed to get current pose')
            return

        current_position = (current_pose[0], current_pose[1])
        current_orientation = current_pose[2]
        
        # Calculate the direction to the goal
        angle_to_goal = math.atan2(goal['y'] - current_position[1], goal['x'] - current_position[0])
        _, _, current_yaw = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])

        # Turn towards the goal
        self.target_yaw = angle_to_goal
        self.turn_to_angle(angle_to_goal - current_yaw)

    def turn_to_angle(self, angle):
        # Normalize angle
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi

        # Determine the turn direction and speed
        angular_speed = 0.3
        if angle < 0:
            angular_speed = -angular_speed

        # Reset turn_completed flag
        self.turn_completed = False

        # Send turn command to hardware
        self.send_command_to_hardware("turn", angle, 5)

        # Create a timer to check if the turn is complete
        self.create_timer(0.1, self.check_turn_completed)

    def check_turn_completed(self):
        if not self.turn_completed:
            # Check if the turn is completed by comparing current yaw with target yaw
            current_pose = self.get_current_pose()
            if current_pose is not None:
                _, _, current_yaw = euler_from_quaternion([current_pose[2].x, current_pose[2].y, current_pose[2].z, current_pose[2].w])
                if abs(current_yaw - self.target_yaw) < 0.1:  # Allow some tolerance
                    self.turn_completed = True
                    self.drive_to_goal(self.goal_position)

    def drive_to_goal(self, goal):
        if self.obstacle_detected:
            self.get_logger().info('Obstacle detected, waiting until path is clear...')
            self.create_timer(0.5, self.check_obstacle_cleared)
        else:
            self.send_command_to_hardware("move_forward", 0.3, 5)

    def check_obstacle_cleared(self):
        if not self.obstacle_detected:
            self.get_logger().info('Obstacle cleared, moving forward to navigate around obstacle...')
            self.send_command_to_hardware("move_forward", 0.3, 5)
            self.create_timer(2.0, self.resume_navigation_after_avoidance)  # Move forward for 2 seconds before resuming navigation

    def resume_navigation_after_avoidance(self):
        self.get_logger().info('Resuming navigation to goal...')
        self.navigate_to_goal(self.goal_position)

    def is_goal_reached(self, current_position, goal_position, tolerance=0.3):
        return math.sqrt((current_position[0] - goal_position['x']) ** 2 + (current_position[1] - goal_position['y']) ** 2) < tolerance

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data

    def send_command_to_hardware(self, command, value, priority):
        request = RobotControl.Request()
        request.command = command
        request.value = value
        request.priority = priority
        future = self.robot_control_client.call_async(request)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Command executed: {response.message}')
            else:
                self.get_logger().info(f'Command failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    custom_navigation = CustomNavigation()

    try:
        rclpy.spin(custom_navigation)
    finally:
        custom_navigation.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

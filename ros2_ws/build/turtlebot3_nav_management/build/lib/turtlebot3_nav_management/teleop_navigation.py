import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from turtlebot3_control_services.srv import RobotControl
import math
import time
import csv

class TeleopNavigationLogger(Node):
    def __init__(self):
        super().__init__('teleop_navigation_logger')
        self.robot_control_client = self.create_client(RobotControl, 'robot_control')
        while not self.robot_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Robot control service not available, waiting again...')

        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
    
        self.declare_parameter('start_x', -1.429)
        self.declare_parameter('start_y', -0.59)
        self.declare_parameter('goal_x', 1.5)
        self.declare_parameter('goal_y', 1.45)

        start_x = self.get_parameter('start_x').get_parameter_value().double_value
        start_y = self.get_parameter('start_y').get_parameter_value().double_value
        goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        goal_y = self.get_parameter('goal_y').get_parameter_value().double_value

        self.coordinates = [{
            'start': {'x': start_x, 'y': start_y},
            'goal': {'x': goal_x, 'y': goal_y}
        }]

        self.current_pose = None
        self.goal_tolerance = 0.3  # Tolerance to consider the robot has reached the goal

        self.run_count = 3
        self.current_run = 0
        self.current_coord_set = 0

        self.initial_pose_publisher = self.create_publisher(PoseStamped, 'initialpose', 10)
        self.shutdown_publisher = self.create_publisher(Bool, 'shutdown_random_explorer', 10)
        self.recording_publisher = self.create_publisher(Bool, 'record_movement', 10)
        self.stop_heatmap_publisher = self.create_publisher(Bool, 'stop_heatmap', 10)  # Initialize stop heatmap publisher

        self.pose_update_count = 0  # Counter for pose updates

        # Initialize variables for distance and time tracking
        self.total_distance = 0.0
        self.previous_pose = None
        self.start_time = None

        # CSV file setup
        self.csv_file = 'navigation_log.csv'
        self.csv_header = ['Coordinate Set', 'Distance Traveled', 'Time Used (s)']
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.csv_header)

        # Publish shutdown signal to the random explorer
        self.shutdown_random_explorer()

        # Start heatmap generator
        self.start_heatmap_generator()

        # Reset distance and start time for the new coordinate set
        self.total_distance = 0.0
        self.previous_pose = None
        self.start_time = time.time()

    def shutdown_random_explorer(self):
        self.get_logger().info('Sending shutdown signal to random explorer...')
        self.shutdown_publisher.publish(Bool(data=True))

    def start_random_explorer(self):
        self.get_logger().info('Sending start signal to random explorer...')
        self.shutdown_publisher.publish(Bool(data=False))

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
            self.get_logger().error(f'Service call failed: {e}')

    def log_navigation_data(self):
        """Log distance traveled and time used for the current coordinate set."""
        time_used = time.time() - self.start_time
        self.get_logger().info(f'Logging data for coordinate set {self.current_coord_set}: distance={self.total_distance}, time={time_used}')
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([self.current_coord_set, self.total_distance, time_used])

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = (position.x, position.y, yaw)

        # Update the total distance traveled
        if self.previous_pose is not None:
            prev_x, prev_y, _ = self.previous_pose
            distance_increment = math.sqrt((position.x - prev_x) ** 2 + (position.y - prev_y) ** 2)
            self.total_distance += distance_increment
        self.previous_pose = self.current_pose

        # Check if the goal has been reached
        self.check_goal_reached()

    def check_goal_reached(self):
        if self.current_pose is None:
            return

        goal = self.coordinates[self.current_coord_set]['goal']
        current_x, current_y, _ = self.current_pose

        distance = math.sqrt((goal['x'] - current_x) ** 2 + (goal['y'] - current_y) ** 2)
        if distance <= self.goal_tolerance:
            self.get_logger().info(f'Reached goal at ({goal["x"]}, {goal["y"]})')
            self.log_navigation_data()  # Log data when a goal is reached
            self.current_run += 1
            if self.current_run >= self.run_count:
                self.current_run = 0
                self.current_coord_set += 1
                if self.current_coord_set < len(self.coordinates):
                    self.start_heatmap_generator()
                    # Reset distance and start time for the new coordinate set
                    self.total_distance = 0.0
                    self.previous_pose = None
                    self.start_time = time.time()
                else:
                    self.get_logger().info('Completed all coordinate sets.')
                    self.start_random_explorer()  # Start random exploration again
                    self.shutdown()

    def start_heatmap_generator(self):
        self.get_logger().info(f'Starting new terminal for heatmap generation for coordinate set {self.current_coord_set}...')
        heatmap_command = [
            "gnome-terminal", "--", "bash", "-c", 
            f"ros2 run turtlebot3_analysis heatmap_generator --ros-args -p coordinate_set:={self.current_coord_set}; exec bash"
        ]
        self.heatmap_process = subprocess.Popen(heatmap_command)

    def shutdown(self):
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    teleop_navigation_logger = TeleopNavigationLogger()

    try:
        rclpy.spin(teleop_navigation_logger)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_navigation_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

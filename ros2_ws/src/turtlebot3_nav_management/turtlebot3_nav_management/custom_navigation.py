import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from turtlebot3_control_services.srv import RobotControl
from tf_transformations import euler_from_quaternion
import math
import csv
import time

class CustomNavigation(Node):
    def __init__(self):
        super().__init__('custom_navigation')
        self.robot_control_client = self.create_client(RobotControl, 'robot_control')
        while not self.robot_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for robot control service...')
        
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.coordinates = [
            {'start': {'x': -1.429, 'y': -0.59}, 'goal': {'x': 1.5, 'y': 1.45}},
            {'start': {'x': 1.7, 'y': 0.06}, 'goal': {'x': -1.851, 'y': 0.06}},
            {'start': {'x': -1.70, 'y': -1.16}, 'goal': {'x': 1.2, 'y': 0.49}},
            {'start': {'x': 1.65, 'y': -1.05}, 'goal': {'x': -0.42, 'y': 1.85}}
        ]

        self.current_pose = None
        self.goal_tolerance = 0.3
        self.run_count = 10
        self.current_run = 0
        self.current_coord_set = 0
        self.nav_state = 'init'
        self.pose_update_count = 0
        self.goal_in_progress = False
        self.total_distance = 0.0
        self.previous_pose = None
        self.start_time = None

        self.csv_file = 'custom_navigation_log.csv'
        self.csv_header = ['Coordinate Set', 'Distance Traveled', 'Time Used (s)']
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.csv_header)

        self.timer = self.create_timer(2.0, self.timer_callback)
        self.shutdown_publisher = self.create_publisher(Bool, 'shutdown_random_explorer', 10)

    def timer_callback(self):
        self.get_logger().info(f'Timer callback with state: {self.nav_state}')
        if self.nav_state == 'init':
            self.move_to_initial_position()
        elif self.nav_state == 'wait_for_initial_pose':
            self.check_initial_pose_reached()
        elif self.nav_state == 'moving_to_goal':
            self.check_goal_reached()
        elif self.nav_state == 'moving_to_start':
            self.check_goal_reached()

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

    def move_to_initial_position(self):
        coord = self.coordinates[self.current_coord_set]['start']
        self.get_logger().info(f'Moving to initial position: ({coord["x"]}, {coord["y"]})')
        self.nav_state = 'wait_for_initial_pose'
        self.navigate_to_pose(coord['x'], coord['y'])

        self.total_distance = 0.0
        self.previous_pose = None
        self.start_time = time.time()

    def check_initial_pose_reached(self):
        coord = self.coordinates[self.current_coord_set]['start']
        if self.current_pose is None:
            return

        current_x, current_y, _ = self.current_pose
        distance = math.sqrt((coord['x'] - current_x) ** 2 + (coord['y'] - current_y) ** 2)
        if distance <= self.goal_tolerance:
            self.get_logger().info(f'Initial pose reached at ({coord["x"]}, {coord["y"]})')
            self.goal_in_progress = False
            self.move_to_goal_position()

    def move_to_goal_position(self):
        coord = self.coordinates[self.current_coord_set]['goal']
        self.get_logger().info(f'Moving to goal position: ({coord["x"]}, {coord["y"]})')
        self.nav_state = 'moving_to_goal'
        self.navigate_to_pose(coord['x'], coord['y'])

    def move_to_start_position(self):
        coord = self.coordinates[self.current_coord_set]['start']
        self.get_logger().info(f'Moving to start position: ({coord["x"]}, {coord["y"]})')
        self.nav_state = 'moving_to_start'
        self.navigate_to_pose(coord['x'], coord['y'])

    def navigate_to_pose(self, x, y):
        if self.goal_in_progress:
            self.get_logger().info('A goal is already in progress. Ignoring new goal request.')
            return
        
        self.goal_in_progress = True
        self.get_logger().info(f'Navigating to pose ({x}, {y})')
        if self.current_pose is None:
            self.get_logger().error('Current pose is not available.')
            return

        current_x, current_y, current_yaw = self.current_pose
        goal_angle = math.atan2(y - current_y, x - current_x)
        angle_diff = goal_angle - current_yaw

        self.get_logger().info(f'Turning by angle: {angle_diff}')
        self.send_command_to_hardware('turn', angle_diff, 10)

        distance = math.sqrt((x - current_x) ** 2 + (y - current_y) ** 2)
        self.get_logger().info(f'Moving forward by distance: {distance}')
        self.send_command_to_hardware('move_forward', distance, 10)

    def check_goal_reached(self):
        if self.current_pose is None:
            return

        goal = self.coordinates[self.current_coord_set]['goal'] if self.nav_state == 'moving_to_goal' else self.coordinates[self.current_coord_set]['start']
        current_x, current_y, _ = self.current_pose

        distance = math.sqrt((goal['x'] - current_x) ** 2 + (goal['y'] - current_y) ** 2)
        self.get_logger().info(f'Current distance to goal: {distance}')
        if distance <= self.goal_tolerance:
            self.get_logger().info(f'Reached goal at ({goal["x"]}, {goal["y"]})')
            self.goal_in_progress = False
            self.log_navigation_data()
            if self.nav_state == 'moving_to_goal':
                self.current_run += 1
                self.get_logger().info(f'Current run: {self.current_run}')

                if self.current_run < self.run_count:
                    self.move_to_start_position()
                else:
                    self.current_run = 0
                    self.current_coord_set += 1
                    if self.current_coord_set < len(self.coordinates):
                        self.move_to_initial_position()
                    else:
                        self.get_logger().info('Completed all coordinate sets.')
                        self.shutdown()
            elif self.nav_state == 'moving_to_start':
                self.nav_state = 'moving_to_goal'
                self.move_to_goal_position()

    def log_navigation_data(self):
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

        if self.previous_pose is not None:
            prev_x, prev_y, _ = self.previous_pose
            distance_increment = math.sqrt((position.x - prev_x) ** 2 + (position.y - prev_y) ** 2)
            self.total_distance += distance_increment
        self.previous_pose = self.current_pose

    def shutdown(self):
        self.get_logger().info('Shutting down custom navigation.')
        self.shutdown_publisher.publish(Bool(data=True))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    custom_navigation = CustomNavigation()

    try:
        rclpy.spin(custom_navigation)
    except KeyboardInterrupt:
        pass
    finally:
        custom_navigation.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

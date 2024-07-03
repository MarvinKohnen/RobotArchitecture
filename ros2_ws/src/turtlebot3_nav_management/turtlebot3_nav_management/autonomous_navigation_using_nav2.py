import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from turtlebot3_control_services.srv import RobotControl, GenerateHeatmap
import math

class AutonomousNavigation(Node):
    def __init__(self):
        super().__init__('autonomous_navigation')
        self.robot_control_client = self.create_client(RobotControl, 'robot_control')
        while not self.robot_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Robot control service not available, waiting again...')

        self.heatmap_client = self.create_client(GenerateHeatmap, 'generate_heatmap')
        while not self.heatmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Heatmap generation service not available, waiting again...')

        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.get_logger().info('Stopping Robot')
        self.send_command_to_hardware("full_stop", 0.0, 10)

        self.coordinates = [
            {'start': {'x': -1.429, 'y': -0.59}, 'goal': {'x': 1.5, 'y': 1.45}},
            {'start': {'x': -1.434, 'y': 1.64}, 'goal': {'x': 1.75, 'y': 0.06}},
            {'start': {'x': 1.1, 'y': -0.466}, 'goal': {'x': -0.42, 'y': 1.85}},
            {'start': {'x': -2.25, 'y': 0.12}, 'goal': {'x': 0.73, 'y': -1.908}}
        ]

        self.current_pose = None
        self.goal_tolerance = 0.1  # Tolerance to consider the robot has reached the goal

        self.run_count = 25
        self.current_run = 0
        self.current_coord_set = 0
        self.nav_state = 'init'
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.initial_pose_publisher = self.create_publisher(PoseStamped, 'initialpose', 10)
        self.shutdown_publisher = self.create_publisher(Bool, 'shutdown_random_explorer', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.pose_update_count = 0  # Counter for pose updates

    def timer_callback(self):
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
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = coord['x']
        initial_pose.pose.position.y = coord['y']
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        initial_pose.pose.orientation.x = q[0]
        initial_pose.pose.orientation.y = q[1]
        initial_pose.pose.orientation.z = q[2]
        initial_pose.pose.orientation.w = q[3]
        self.initial_pose_publisher.publish(initial_pose)
        self.get_logger().info(f'Moving to initial position: ({coord["x"]}, {coord["y"]})')
        self.nav_state = 'wait_for_initial_pose'
        self.navigate_to_pose(coord['x'], coord['y'])

    def check_initial_pose_reached(self):
        coord = self.coordinates[self.current_coord_set]['start']
        if self.current_pose is None:
            return

        current_x, current_y, _ = self.current_pose
        distance = math.sqrt((coord['x'] - current_x) ** 2 + (coord['y'] - current_y) ** 2)
        if distance <= self.goal_tolerance:
            self.get_logger().info(f'Initial pose reached at ({coord["x"]}, {coord["y"]})')
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
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0
        goal_msg.pose = goal_pose
        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Goal action completed')
            self.check_goal_reached()
        else:
            self.get_logger().info('Goal action failed')

    def check_goal_reached(self):
        if self.current_pose is None:
            return

        goal = self.coordinates[self.current_coord_set]['goal'] if self.nav_state == 'moving_to_goal' else self.coordinates[self.current_coord_set]['start']
        current_x, current_y, _ = self.current_pose

        distance = math.sqrt((goal['x'] - current_x) ** 2 + (goal['y'] - current_y) ** 2)
        if distance <= self.goal_tolerance:
            self.get_logger().info(f'Reached goal at ({goal["x"]}, {goal["y"]})')
            if self.nav_state == 'moving_to_goal':
                self.current_run += 1
                self.get_logger().info(f'Current run: {self.current_run}')
                if self.current_run >= self.run_count:
                    self.current_run = 0
                    self.call_heatmap_service()
                else:
                    self.move_to_start_position()
            elif self.nav_state == 'moving_to_start':
                self.move_to_goal_position()

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = (position.x, position.y, yaw)
        
        self.pose_update_count += 1
        if self.pose_update_count % 50 == 0:  # Log every 50th update
            self.get_logger().info(f'Updated current pose: {self.current_pose}')

    def call_heatmap_service(self):
        self.get_logger().info('Calling heatmap generation service...')
        request = GenerateHeatmap.Request()
        request.coordinate_set = self.current_coord_set
        future = self.heatmap_client.call_async(request)
        future.add_done_callback(self.heatmap_service_callback)

    def heatmap_service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Heatmap generated successfully.')
            else:
                self.get_logger().error('Failed to generate heatmap.')
        except Exception as e:
            self.get_logger().error(f'Heatmap service call failed: {e}')

        self.current_coord_set += 1
        if self.current_coord_set < len(self.coordinates):
            self.move_to_initial_position()
        else:
            self.get_logger().info('Completed all coordinate sets.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    autonomous_navigation = AutonomousNavigation()

    try:
        rclpy.spin(autonomous_navigation)
    except KeyboardInterrupt:
        pass
    finally:
        autonomous_navigation.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

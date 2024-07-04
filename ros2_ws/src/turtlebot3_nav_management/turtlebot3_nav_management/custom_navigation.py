import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from turtlebot3_control_services.srv import RobotControl
import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse
import math

class CustomNavigation(Node):
    def __init__(self):
        super().__init__('custom_navigation')
        self.robot_control_client = self.create_client(RobotControl, 'robot_control')
        while not self.robot_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Robot control service not available, waiting again...')

        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.obstacle_subscriber = self.create_subscription(Bool, 'obstacle_detected', self.obstacle_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.current_pose = None
        self.goal_tolerance = 0.3  # Tolerance to consider the robot has reached the goal

        self.run_count = 10
        self.current_run = 0
        self.current_coord_set = 0
        self.nav_state = 'init'

        self.shutdown_publisher = self.create_publisher(Bool, 'shutdown_random_explorer', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.obstacle_detected = False  # Flag to indicate obstacle detection

        self.full_stop_robot()
        self.coordinates = [
            {'start': {'x': -1.429, 'y': -0.59}, 'goal': {'x': 1.5, 'y': 1.45}},
            {'start': {'x': 1.7, 'y': 0.06}, 'goal': {'x': -1.851, 'y': 0.06}},
            {'start': {'x': -1.70, 'y': -1.16}, 'goal': {'x': 1.2, 'y': 0.49}},
            {'start': {'x': 1.65, 'y': -1.05}, 'goal': {'x': -0.42, 'y': 1.85}}
        ]

    def full_stop_robot(self):
        self.get_logger().info('Stopping Robot')
        self.send_command_to_hardware("full_stop", 0.0, 3)
        self.shutdown_random_explorer()
        self.reset_priority()  # Reset priority after full stop

    def shutdown_random_explorer(self):
        self.get_logger().info('Shutting down Random Explorer')
        shutdown_msg = Bool()
        shutdown_msg.data = True
        self.shutdown_publisher.publish(shutdown_msg)

    def timer_callback(self):
        if self.nav_state == 'init':
            self.move_to_initial_position()
        elif self.nav_state == 'moving_to_goal' or self.nav_state == 'moving_to_start':
            if self.obstacle_detected:
                self.handle_obstacle()
            else:
                self.monitor_progress()

    def send_command_to_hardware(self, command, value, priority):
        self.get_logger().info(f'Sending command to hardware: {command}, value: {value}, priority: {priority}')
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

    def reset_priority(self):
        self.get_logger().info('Resetting priority')
        self.send_command_to_hardware('reset_priority', 0.0, 5)

    def move_to_initial_position(self):
        self.nav_state = 'moving_to_start'
        self.get_logger().info(f'Moving to initial position: {self.coordinates[self.current_coord_set]["start"]}')
        self.move_to_position(self.coordinates[self.current_coord_set]['start'])

    def move_to_goal_position(self):
        self.nav_state = 'moving_to_goal'
        self.get_logger().info(f'Moving to goal position: {self.coordinates[self.current_coord_set]["goal"]}')
        self.move_to_position(self.coordinates[self.current_coord_set]['goal'])

    def move_to_position(self, coord):
        if self.current_pose is None:
            self.get_logger().info('Current pose is None, cannot move to position.')
            return

        transformed_pose = self.transform_pose_to_map_frame(self.current_pose)

        current_x, current_y, current_yaw = transformed_pose.position.x, transformed_pose.position.y, self.yaw_from_orientation(transformed_pose.orientation)
        target_x, target_y = coord['x'], coord['y']
        self.get_logger().info(f'Current pose: ({current_x}, {current_y}, {current_yaw})')
        self.get_logger().info(f'Target position: ({target_x}, {target_y})')

        angle_to_goal = math.atan2(target_y - current_y, target_x - current_x)
        turn_angle = angle_to_goal - current_yaw

        self.get_logger().info(f'Turning towards goal: turn_angle={turn_angle}')
        self.send_command_to_hardware('turn', turn_angle, 1)

        self.get_logger().info(f'Moving towards goal: distance={self.calculate_distance((current_x, current_y), (target_x, target_y))}')
        self.send_command_to_hardware('move_forward', 0.2, 1)  # Move forward at a constant speed

    def monitor_progress(self):
        if self.current_pose is None:
            return

        transformed_pose = self.transform_pose_to_map_frame(self.current_pose)

        coord = self.coordinates[self.current_coord_set]['goal'] if self.nav_state == 'moving_to_goal' else self.coordinates[self.current_coord_set]['start']
        current_x, current_y, current_yaw = transformed_pose.position.x, transformed_pose.position.y, self.yaw_from_orientation(transformed_pose.orientation)
        target_x, target_y = coord['x'], coord['y']

        angle_to_goal = math.atan2(target_y - current_y, target_x - current_x)
        turn_angle = angle_to_goal - current_yaw

        distance_to_goal = self.calculate_distance((current_x, current_y), (target_x, target_y))

        self.get_logger().info(f'Monitoring progress: Current position: ({current_x}, {current_y}), Goal position: ({target_x}, {target_y}), Distance: {distance_to_goal}, Angle: {turn_angle}')

        if distance_to_goal <= self.goal_tolerance:
            self.get_logger().info(f'Reached goal at ({target_x}, {target_y})')
            self.send_command_to_hardware('full_stop', 0.0, 10)
            self.reset_priority()  # Reset priority after reaching goal
            self.goal_reached()
            return  # Exit the function to prevent further commands

        if abs(turn_angle) > 0.1:  # Adjust heading if deviation is significant
            self.get_logger().info(f'Adjusting heading: turn_angle={turn_angle}')
            self.send_command_to_hardware('turn', turn_angle, 2)

            self.reset_priority()  # Reset priority after adjusting heading

        self.send_command_to_hardware('move_forward', 0.2, 1)  # Continue moving towards the goal

    def goal_reached(self):
        if self.nav_state == 'moving_to_goal':
            self.current_run += 1
            self.get_logger().info(f'Current run: {self.current_run}')
            if self.current_run < self.run_count:
                self.move_to_start_position()
            else:
                self.current_run = 0
                self.current_coord_set += 1
                if self.current_coord_set < len(self.coordinates):
                    self.nav_state = 'init'
                    self.move_to_initial_position()
                else:
                    self.get_logger().info('Completed all coordinate sets.')
                    self.shutdown()
        elif self.nav_state == 'moving_to_start':
            self.nav_state = 'moving_to_goal'
            self.move_to_goal_position()

    def move_to_start_position(self):
        self.nav_state = 'moving_to_start'
        self.get_logger().info(f'Moving to start position: {self.coordinates[self.current_coord_set]["start"]}')
        self.move_to_position(self.coordinates[self.current_coord_set]['start'])

    def handle_obstacle(self):
        self.get_logger().info('Handling obstacle: Moving forward to clear the obstacle.')
        self.send_command_to_hardware('move_forward', 0.4, 2)
        coord = self.coordinates[self.current_coord_set]['goal'] if self.nav_state == 'moving_to_goal' else self.coordinates[self.current_coord_set]['start']
        self.move_to_position(coord)
        self.obstacle_detected = False  # Reset obstacle detected flag

        self.reset_priority()  # Reset priority after handling obstacle

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data
        self.get_logger().info(f'Obstacle detected: {self.obstacle_detected}')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        #self.get_logger().info(f'Updated current pose: {self.current_pose}')

    def transform_pose_to_map_frame(self, pose):
        try:
            # Transform from odom to map
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))
            return self.do_transform_pose(pose, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Transform error: {str(e)}')
            return pose

    def do_transform_pose(self, pose, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Translate
        pose.position.x += translation.x
        pose.position.y += translation.y
        pose.position.z += translation.z

        # Rotate
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        q_t = [rotation.x, rotation.y, rotation.z, rotation.w]
        q_new = quaternion_multiply(q_t, q)
        q_new = quaternion_multiply(q_new, quaternion_inverse(q_t))

        pose.orientation.x = q_new[0]
        pose.orientation.y = q_new[1]
        pose.orientation.z = q_new[2]
        pose.orientation.w = q_new[3]

        return pose

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def yaw_from_orientation(self, orientation):
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw

    def shutdown(self):
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

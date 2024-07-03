import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Bool
from turtlebot3_control_services.srv import RobotControl
import time

class AutonomousNavigation(Node):
    def __init__(self):
        super().__init__('autonomous_navigation')
        self.robot_control_client = self.create_client(RobotControl, 'robot_control')
        while not self.robot_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Robot control service not available, waiting again...')

        self.get_logger().info('StoppingRobot')
        self.send_command_to_hardware("full_stop", 0.0, 10)
        
        self.declare_parameter('start_x', -1.429)
        self.declare_parameter('start_y', -0.59)
        self.declare_parameter('goal_x', 1.5)
        self.declare_parameter('goal_y', 1.45)
        self.declare_parameter('initial_orientation', [0.0, 0.0, 0.0])  # Default initial orientation (yaw, pitch, roll)

        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.initial_orientation = self.get_parameter('initial_orientation').value

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.initial_pose_publisher = self.create_publisher(PoseStamped, 'initialpose', 10)
        self.shutdown_publisher = self.create_publisher(Bool, 'shutdown_random_explorer', 10)
        self.timer = self.create_timer(2.0, self.set_initial_pose)
        self.initial_pose_set = False
        self.reached_initial_position = False

        

    def send_command_to_hardware(self, command, value, priority):
        request = RobotControl.Request()
        request.command = command
        request.value = value
        request.priority = priority
        future = self.robot_control_client.call_async(request)
        future.add_done_callback(self.handle_service_response)

    #needed to be placed in separate function in order to handly multiple asynchronous operations
    def handle_service_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Command executed: {response.message}')
            else:
                self.get_logger().info(f'Command failed: {response.message}')
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % e)


    def set_initial_pose(self):
        if self.initial_pose_set:
            return

        # Send shutdown signal to random_explore node
        self.shutdown_publisher.publish(Bool(data=True))
        self.get_logger().info('Shutdown signal sent to random_explorer')

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.start_x
        initial_pose.pose.position.y = self.start_y

        # Convert Euler angles to quaternion for orientation
        q = quaternion_from_euler(self.initial_orientation[0], self.initial_orientation[1], self.initial_orientation[2])
        initial_pose.pose.orientation.x = q[0]
        initial_pose.pose.orientation.y = q[1]
        initial_pose.pose.orientation.z = q[2]
        initial_pose.pose.orientation.w = q[3]

        self.initial_pose_publisher.publish(initial_pose)
        self.get_logger().info('Initial pose published')
        self.initial_pose_set = True

        # Delay to ensure the robot receives the initial pose
        time.sleep(2)
        self.move_to_initial_position()

    def move_to_initial_position(self):
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.start_x
        goal_pose.pose.position.y = self.start_y
        goal_pose.pose.orientation.w = 1.0  # Facing forward

        goal_msg.pose = goal_pose

        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.initial_position_response_callback)

    def initial_position_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Initial position goal rejected :(')
            return

        self.get_logger().info('Initial position goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.initial_position_result_callback)

    def initial_position_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Reached initial position :)')
            self.reached_initial_position = True
            self.move_to_goal_position()
        else:
            self.get_logger().info('Failed to reach initial position :(')

    def move_to_goal_position(self):
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.goal_x
        goal_pose.pose.position.y = self.goal_y
        goal_pose.pose.orientation.w = 1.0  # Facing forward

        goal_msg.pose = goal_pose

        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Goal reached :)')
        else:
            self.get_logger().info('Goal failed :(')

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

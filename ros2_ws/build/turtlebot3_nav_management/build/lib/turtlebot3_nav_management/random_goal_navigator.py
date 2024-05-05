import rclpy
from rclpy.node import Node
from turtlebot3_control_services.srv import RobotControl
import numpy as np
import random
import astar
import cv2
import os
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs import Twist, Pose
from math import atan2, sqrt, pow, pi
from tf_transformations import euler_from_quaternion

class RandomGoalNavigator(Node):
    def __init__(self):
        super().__init__('random_goal_navigator')
        yaml_path = self.get_parameter('map_path').get_parameter_value().string_value
        self.get_logger().info(f'Using map at: {yaml_path}')

        #for hardware control
        self.robot_control_client = self.create_client(RobotControl, 'robot_control')
        while not self.robot_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for robot control service...')

    
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom',
            self.odom_callback,
            10)

        # Placeholder for the current pose
        self.current_pose = Pose() 
        self.odom_received = False

        # Load map data
        self.map_image, self.resolution, self.origin = self.load_map(yaml_path)
        # Convert map image to occupancy grid
        self.occupancy_grid = self.map_image_to_occupancy_grid(self.map_image)


    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.odom_received = True

    def load_map(self, yaml_path):
        # Load map metadata from the YAML file
        with open(yaml_path, 'r') as yaml_file:
            map_metadata = yaml.safe_load(yaml_file)
        
        # Construct the full path to the .pgm file
        map_dir = os.path.dirname(yaml_path)
        pgm_path = os.path.join(map_dir, map_metadata['image'])
        
        # Load the .pgm file using OpenCV
        map_image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        
        # Check if the image was loaded successfully
        if map_image is None:
            raise FileNotFoundError(f"Could not load map image from {pgm_path}")

        # Extract resolution and origin from the metadata
        resolution = map_metadata['resolution']
        origin = map_metadata['origin']
        
        return map_image, resolution, origin

    def map_image_to_occupancy_grid(self, map_image):
        # Example conversion logic; adjust threshold as needed
        occupancy_grid = np.where(map_image < 128, 1, 0)  # Occupied if < 128, else free
        return occupancy_grid


    def navigate_to_random_goal(self):
        # Choose a random goal in the occupancy grid
        free_spaces = np.argwhere(self.occupancy_grid == 0)
        goal = random.choice(free_spaces)

        # MAYBE TODO: mit tf2 ros pose transformieren je nach map
        if self.odom_received:
            # Use A* to find a path from start to goal
            path = astar.astar(self.occupancy_grid, self.current_pose, tuple(goal))

            if path:
                self.get_logger().info(f'Path found: {path}')
                self.follow_path(path)
            else:
                self.get_logger().info('No path found to the random goal.')

    def follow_path(self, path):
        # Wait for the first odometry message
        while not self.odom_received:
            self.get_logger().info("Waiting for odometry data...")
            rclpy.spin_once(self)

        for i in range(1, len(path)):
            # Current target waypoint
            next = path[i]

            # Continuous feedback loop until the target is reached
            while not self.at_target(next):
                current = (self.current_pose.position.x, self.current_pose.position.y)
                angle_to_target = atan2(next[1] - current[1], next[0] - current[0])
                distance_to_target = sqrt(pow(next[1] - current[1], 2) + pow(next[0] - current[0], 2))

                # Calculate required turn
                current_orientation = self.get_yaw_from_quaternion(self.current_pose.orientation)  # Corrected call
                turn_angle = self.normalize_angle(angle_to_target - current_orientation)

                # Send turn command
                self.send_command_to_hardware('turn', turn_angle, 5)
                # Move forward slightly
                self.send_command_to_hardware('move_forward', min(distance_to_target, 0.1), 5)
                rclpy.spin_once(self)

            self.get_logger().info(f"Reached waypoint {next}")

    def at_target(self, target):
        current = (self.current_pose.position.x, self.current_pose.position.y)
        distance = sqrt(pow(target[1] - current[1], 2) + pow(target[0] - current[0], 2))
        return distance < 0.1  # Consider close enough if within 10 cm of the target
            
    
    def get_yaw_from_quaternion(self, quat):
        # Convert quaternion (geometry_msgs/Quaternion) to yaw
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        euler = euler_from_quaternion(quaternion)
        return euler[2]  # yaw

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle
    

    def send_command_to_hardware(self, command, value, priority):
        request = RobotControl.Request()
        request.command = command
        request.value = value   
        request.priority = priority
        future = self.robot_control_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)  # Wait for the command to complete

        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Command executed: {response.message}')
            else:
                self.get_logger().info(f'Command failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    custom_navigator = RandomGoalNavigator()
    rclpy.spin(custom_navigator)
    custom_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

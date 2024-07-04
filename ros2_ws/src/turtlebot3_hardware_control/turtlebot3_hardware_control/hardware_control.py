import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlebot3_control_services.srv import RobotControl
import threading

class HardwareControl(Node):
    def __init__(self):
        super().__init__('hardware_control')
        self.service = self.create_service(RobotControl, 'robot_control', self.robot_control_callback)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.current_priority = 0  # Initialize with the lowest priority

        self.lock = threading.Lock()

    def robot_control_callback(self, request, response):
        self.get_logger().info(f"received command: {request}")
        self.get_logger().info(f"current Priority: {self.current_priority}")
        if request.priority >= self.current_priority:
            if request.command == "move_forward":
                self.drive_forward(request.value, request.priority)
            elif request.command == "turn_left":
                self.turn_left(request.value, request.priority)
            elif request.command == "turn_right":
                self.turn_right(request.value, request.priority)
            elif request.command == "full_stop":
                self.full_stop(request.priority)
            elif request.command == "turn":
                self.turn_by_angle(request.value, request.priority)  
            elif request.command == "reset_priority":
                self.current_priority = 0
                self.get_logger().info('Priority reset.')
            response.success = True
            response.message = "Command executed correctly."
        else:
            response.success = False
            response.message = "Command ignored due to lower priority."
    
        
        return response

    def execute_command(self, msg: Twist, priority: int):
        """Execute a command"""
        self.publisher_.publish(msg)
        self.current_priority = priority  # Update the current priority
        self.get_logger().info(f'Executing command with priority {priority}')

    def drive_forward(self, speed: float, priority: int):
        """Drive the robot forward at the specified speed with a given priority."""
        msg = Twist()
        msg.linear.x = speed
        self.get_logger().info(f'Executing drive_forward with speed {msg.linear.x} and priority {priority}')
        self.execute_command(msg, priority)

    def full_stop(self, priority: int):
        """Stop the robot with a given priority."""
        msg = Twist()  # Zero velocity to full stop
        self.get_logger().info(f'Executing full_stop, priority: {priority}')
        self.execute_command(msg, priority)

    def turn_left(self, angular_speed: float, priority: int):
        """Turn the robot left with a given priority."""
        msg = Twist()
        msg.angular.z = angular_speed
        self.get_logger().info(f'Executing turn_left with speed {msg.angular.z} and priority {priority}')
        self.execute_command(msg, priority)

    def turn_right(self, angular_speed: float, priority: int):
        """Turn the robot right with a given priority. Note: negative angular speed."""
        msg = Twist()
        msg.angular.z = -angular_speed
        self.get_logger().info(f'Executing turn_right with speed {msg.angular.z} and priority {priority}')
        self.execute_command(msg, priority)

    def turn_by_angle(self, angle: float, priority: int):
        """Turn the robot by a specific angle (in radians) with a given priority."""
        angular_speed = 0.3  # Set a default angular speed
        if angle < 0:
            angular_speed = -angular_speed

        duration = abs(angle / angular_speed)  # Calculate the duration to turn the specified angle

        def turn():
            msg = Twist()
            msg.angular.z = angular_speed
            self.execute_command(msg, priority)
            self.get_logger().info(f'Turning with speed {angular_speed} for duration {duration} seconds')
            rclpy.spin_once(self, timeout_sec=duration)
            self.full_stop(10)
            self.reset_priority()
        
        threading.Thread(target=turn).start()

    def stop_robot(self):
        """Failsafe for real world testing"""
        msg = Twist()  # Zero velocity to full stop
        self.publisher_.publish(msg)
        self.get_logger().info('Failsafe: Full stop executed.')

def main(args=None):
    print("Starting the node")
    rclpy.init(args=args)
    hardware_control = HardwareControl()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(hardware_control)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Keyboard Interrupt Received. shutting down.")
        hardware_control.stop_robot()
    finally:
        if rclpy.ok():  # Only shutdown if rclpy has not already been shut down
            hardware_control.destroy_node()
            rclpy.shutdown()
        print("END Node lifecycle.")

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlebot3_control_services.srv import RobotControl  # Update with actual package and service names

class HardwareControl(Node):
    def __init__(self):
        super().__init__('hardware_control')
        self.service = self.create_service(RobotControl, 'robot_control', self.robot_control_callback)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.current_priority = 0  # Initialize with the lowest priority

    def robot_control_callback(self, request, response):
        if request.priority >= self.current_priority:
            if request.command == "move_forward":
                self.drive_forward(request.value, request.priority)
            elif request.command == "turn_left":
                self.turn_left(request.value, request.priority)
            elif request.command == "turn_right":
                self.turn_right(request.value, request.priority)
            elif request.command == "full_stop":
                self.full_stop(request.priority)
            response.success = True
            response.message = "Command executed."
        else:
            response.success = False
            response.message = "Lower priority command ignored."
        return response


    def execute_command(self, msg: Twist, priority: int):
        """Execute a command if the priority is higher than the current priority."""
        if priority >= self.current_priority:
            self.publisher_.publish(msg)
            self.current_priority = priority  # Update the current priority
            self.get_logger().info(f'Executing command with priority {priority}')
        else:
            self.get_logger().info(f'Command ignored due to lower priority: {priority}')

    def drive_forward(self, speed: float, priority: int):
        """Drive the robot forward at the specified speed with a given priority."""
        msg = Twist()
        msg.linear.x = speed
        self.execute_command(msg, priority)

    def full_stop(self, priority: int):
        """Stop the robot with a given priority."""
        msg = Twist()  # Zero velocity to full stop
        self.execute_command(msg, priority)

    def turn_left(self, angular_speed: float, priority: int):
        """Turn the robot left with a given priority."""
        msg = Twist()
        msg.angular.z = angular_speed
        self.execute_command(msg, priority)

    def turn_right(self, angular_speed: float, priority: int):
        """Turn the robot right with a given priority. Note: negative angular speed."""
        msg = Twist()
        msg.angular.z = -angular_speed
        self.execute_command(msg, priority)

def main(args=None):
    rclpy.init(args=args)
    hardware_control = HardwareControl()
    rclpy.spin(hardware_control)  # Keep the node alive to listen for incoming requests.

    hardware_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

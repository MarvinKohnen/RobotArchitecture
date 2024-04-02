import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import random
from turtlebot3_control_services.srv import RobotControl  # Adjust with your actual package name

class RandomExplorer(Node):
    def __init__(self):
        super().__init__('random_explorer')
        self.robot_control_client = self.create_client(RobotControl, 'robot_control')
        while not self.robot_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Robot control service not available, waiting again...')
        self.timer_ = self.create_timer(0.5, self.timer_callback)


    def timer_callback(self):
        print("here")
        if random.random() > 0.1:
            self.get_logger().info('Moving forward.')
            self.send_command_to_hardware("move_forward", 0.2, 1)
        else:
            turn_direction = random.choice(["turn_left", "turn_right"])
            self.get_logger().info(f'Turning {turn_direction.replace("_", " ")}.')
            self.send_command_to_hardware(turn_direction, 0.6, 1)  
            

    def send_command_to_hardware(self, command, value, priority):
        request = RobotControl.Request()
        request.command = command
        request.value = value
        request.priority = priority
        future = self.robot_control_client.call_async(request)
        print("now here")
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Command executed: {future.result().message}')
            else:
                self.get_logger().info(f'Command failed: {future.result().message}')
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

def main(args=None):
    rclpy.init(args=args)
    random_explorer = RandomExplorer()
    rclpy.spin(random_explorer)
    random_explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from tf_transformations import quaternion_multiply, quaternion_inverse

class PositionLogger(Node):
    def __init__(self):
        super().__init__('position_logger')
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.log_position)  # Adjust the logging interval as needed
        self.current_pose = None

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def log_position(self):
        if self.current_pose is None:
            return

        try:
            # Transform from odom to map
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))
            transformed_pose = self.do_transform_pose(self.current_pose, transform)
            self.get_logger().info(f"Position: x={transformed_pose.position.x}, y={transformed_pose.position.y}, orientation (yaw)={transformed_pose.orientation.z}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Transform error: {str(e)}')

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

def main(args=None):
    rclpy.init(args=args)
    position_logger = PositionLogger()
    try:
        rclpy.spin(position_logger)
    except KeyboardInterrupt:
        pass
    finally:
        position_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

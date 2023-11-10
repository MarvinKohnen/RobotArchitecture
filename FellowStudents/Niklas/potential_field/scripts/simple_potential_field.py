import numpy as np
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Twist
import tf
from tf.transformations import euler_from_quaternion

CYLINDER_DEFAULT_RADIUS = 0.2

# MAX_LINEAR_VELOCITY = 1
# ANGULAR_VELOCITY = 20
# GOAL = np.array((10, 0))

SCALING_FACTOR_ATTRACTION_FORCE = 1/20
SCALING_FACTOR_REPULSIVE_FORCE = 7/20
MIN_DISTANCE_REPULSIVE_FORCE = 1 #3


class Circle:
    """Klasse um einen Kreis zubeschreiben."""

    x: float
    y: float
    radius: float

    def __init__(self, x: float, y: float, radius: float):
        self.x = x
        self.y = y
        self.radius = radius

    def get_position(self):
        return np.array((self.x, self.y))


    def get_simple_distance_point(self, point: tuple):
        return (
            np.sqrt((self.x - point[0]) ** 2 + (self.y - point[1]) ** 2)
        )

    def get_shell_distance_point(self, point: tuple):
        return (
            np.sqrt((self.x - point[0]) ** 2 + (self.y - point[1]) ** 2) - (self.radius + ROBOT_CIRCLE_SIZE)
        )

class Updater:
    """Klasse welche die internen Repräsentationen der Hindernisse updatet und eine neu Bewegung des Roboters publisht."""
    pub = rospy.Publisher

    def __init__(self) -> None:
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber('gazebo/model_states', ModelStates, self.update_and_move)

    def update_and_move(self, model: ModelStates):
        global OBSTACLES
        OBSTACLES = ()
        current_point = None
        for i in range(len(model.name)):
                if model.name[i].startswith('cylinder_clone'):
                    OBSTACLES = (*OBSTACLES, Circle(model.pose[i].position.x,model.pose[i].position.y,CYLINDER_DEFAULT_RADIUS))
                if model.name[i].startswith('turtlebot'):
                    current_point = np.array((model.pose[i].position.x,model.pose[i].position.y))
                    # current_orientation = np.array((model.pose[i].orientation.x,model.pose[i].orientation.y))

                    quaternion = (model.pose[i].orientation.x, model.pose[i].orientation.y, model.pose[i].orientation.z, model.pose[i].orientation.w)
                    euler = euler_from_quaternion(quaternion)
                    current_orientation = euler[2]


        # for cylinder in OBSTACLES:            # TODO OBSTACLES RELATIV ZU ROBOTER MACHEN
        #     rotations_winkel =
        #     rotations_matrix = np.array([[np.cos(rotations_winkel), -np.sin(rotations_winkel)], [np.sin(rotations_winkel), np.cos(rotations_winkel)]])


        self.calculate_next_moving_command(current_point, current_orientation)

    # def calculate_next_moving_command(self, current_point, current_orientation):
    #     msg = Twist()

    #     # Da Objekte global in Gazebo-Map gelistet werden, muss der APF-Vektor noch fürn Roboter ausgerichtet werden.
    #     non_relativ_force_vektor = potential_force(current_point)

    #     # rotations_winkel = current_orientation
    #     # rotations_winkel = -current_orientation + np.arctan2(*non_relativ_force_vektor[::-1])
    #     rotations_winkel = np.pi/2 - current_orientation

    #     rotations_matrix = np.array([[np.cos(rotations_winkel), -np.sin(rotations_winkel)], [np.sin(rotations_winkel), np.cos(rotations_winkel)]])
    #     force_vector = rotations_matrix @ non_relativ_force_vektor


    #     b = 0.5 #np.linalg.norm(force_vector)
    #     control_commands = np.array([[np.cos(ANGULAR_VELOCITY), np.sin(ANGULAR_VELOCITY)], [-1/b * np.sin(ANGULAR_VELOCITY), 1/b * np.cos(ANGULAR_VELOCITY)]]) @ force_vector

    #     # control_commands = np.array([[np.cos(b), np.sin(b)], [-1/ANGULAR_VELOCITY * np.sin(b), 1/ANGULAR_VELOCITY * np.cos(b)]]) @ force_vector

    #     # speed = 1/(1 + np.exp(-np.linalg.norm(force_vector))) * MAX_SPEED
    #     # msg.linear.x = speed                                                      # speed using Sigmoid

    #     msg.linear.x, msg.angular.z = control_commands   # using https://medium.com/@sarim.mehdi.550/mapping-path-following-for-a-two-wheeled-robot-b8bd55214405
    #     self.pub.publish(msg)

    #     # print('1',msg.linear.x, msg.angular.z)
    #     # print('2',control_commands)
    #     # print('3',non_relativ_force_vektor)
    #     # print('4',force_vector)
    #     # print('5',rotations_matrix)
    #     print('current_orientation: ',current_orientation)

    #     print('current_position: ',current_point)
    #     # print(1,non_relativ_force_vektor)



    def calculate_next_moving_command(self, current_point, current_orientation):
        msg = Twist()

        non_relativ_force_vektor = potential_force(current_point)

        b = np.linalg.norm(non_relativ_force_vektor)
        control_commands = np.array([[np.cos(current_orientation), np.sin(current_orientation)], [-1/b * np.sin(current_orientation), 1/b * np.cos(current_orientation)]]) @ non_relativ_force_vektor

        # speed = 1/(1 + np.exp(-np.linalg.norm(force_vector))) * MAX_SPEED
        # msg.linear.x = speed                                                      # speed using Sigmoid

        msg.linear.x, msg.angular.z = control_commands   # using https://medium.com/@sarim.mehdi.550/mapping-path-following-for-a-two-wheeled-robot-b8bd55214405
        self.pub.publish(msg)


def attraction_field(point):
    # return 1/2 * SCALING_FACTOR_ATTRACTION_FORCE * np.linalg.norm(GOAL - np.array(point))**2
    return 1/2 * SCALING_FACTOR_ATTRACTION_FORCE * np.dot(
        GOAL - np.array(point), GOAL - np.array(point)
    )

def repulsive_field(point):
    rep_field = 0
    for circle in OBSTACLES:
        distance = circle.get_shell_distance_point(point)
        if distance < MIN_DISTANCE_REPULSIVE_FORCE:
            rep_field += 1/2 * SCALING_FACTOR_REPULSIVE_FORCE * (1/distance - 1/MIN_DISTANCE_REPULSIVE_FORCE)**2
    return rep_field


def potential_field(point):
    return attraction_field(point) + repulsive_field(point)


def attraction_force(point):
    return SCALING_FACTOR_ATTRACTION_FORCE * (GOAL - np.array(point))

def repulsive_force(point):
    rep_force = 0
    for circle in OBSTACLES:
        distance = circle.get_shell_distance_point(point)
        if distance < MIN_DISTANCE_REPULSIVE_FORCE:
            rep_force += SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - circle.get_position()) / np.linalg.norm(np.array(point) - circle.get_position())
    return rep_force

def potential_force(point):
    return attraction_force(point) + repulsive_force(point)




def get_ros_params():
    try:
        global GOAL
        if rospy.has_param('/potential_field/goal_x') and rospy.has_param('/potential_field/goal_y'):
            GOAL = np.array((float(rospy.get_param('/potential_field/goal_x')), float(rospy.get_param('/potential_field/goal_y'))))
        else:
            GOAL = np.array((10, 0))
            # GOAL = None
        
        global ROBOT_CIRCLE_SIZE
        if rospy.has_param('/potential_field/robot_circle_size'):
            ROBOT_CIRCLE_SIZE = rospy.get_param('/potential_field/robot_circle_size')
        else:
            ROBOT_CIRCLE_SIZE = 0.1
            
    except rospy.ROSInterruptException:
        rospy.loginfo('-> ERROR - Map_Obstacle.py - goal Parameter')

if __name__ == "__main__":
    get_ros_params()
    rospy.init_node("simple_potential_field_mover", anonymous=False)
    Updater()
    rospy.spin()
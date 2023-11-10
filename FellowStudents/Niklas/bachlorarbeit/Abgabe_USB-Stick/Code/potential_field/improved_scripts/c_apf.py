import numpy as np
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Twist
import tf
from tf.transformations import euler_from_quaternion
import time

################## Robot and update - Parameters ###############################

CYLINDER_DEFAULT_RADIUS = 0.2
ROBOT_MAX_VELOCITY = 1
ROBOT_UPDATE_TIME = .1
ROBOT_CIRCLE_SIZE = 0.1

OBSTACLE_POSITION_UPDATE_TIME = .1
OBSTACLE_MOVEMENT_UPDATE_THRESHOLD = 0.05

################## Classic APF - Parameters ###############################

SCALING_FACTOR_ATTRACTION_FORCE = 1/20
SCALING_FACTOR_REPULSIVE_FORCE = 5 
MIN_DISTANCE_REPULSIVE_FORCE = 3

################## Classic APF - Code ###############################

def attraction_force(point, GOAL):
    return SCALING_FACTOR_ATTRACTION_FORCE * (GOAL - np.array(point))

def repulsive_force(point, OBSTACLES):
    rep_force = 0
    for circle in OBSTACLES:
        distance = circle.get_shell_distance(point)
        if distance < MIN_DISTANCE_REPULSIVE_FORCE:
            rep_force += SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - circle.get_position()) / np.linalg.norm(np.array(point) - circle.get_position())**3
    return rep_force

def potential_force(point, GOAL, OBSTACLES):
    return attraction_force(point, GOAL) + repulsive_force(point, OBSTACLES)

################### Gazebo -> Code ##############################

class Velocity_Circle:
    """Klasse um einen bewegenden Kreis zubeschreiben."""

    name: str
    x: float
    y: float
    radius: float
    last_position: np.array
    last_position_time: float
    current_movement: np.array

    def __init__(self, x: float, y: float, radius: float, name:str):
        self.x = x
        self.y = y
        self.radius = radius
        self.name = name
        self.last_position = None
        self.last_position_time = None
        self.current_movement = np.array((0,0))

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.name == other.name
        else:
            raise TypeError('Velocity_Circle wird mit etwas falschem verglichen!!')

    def __ne__(self, other):
        return not self.__eq__(other)

    def get_position(self):
        return np.array((self.x, self.y))
    
    def update_position(self, point: np.array):
        self.x, self.y = point

        current_time = time.time()
        if self.last_position is None or self.last_position_time is None:
            self.last_position = point
            self.last_position_time = current_time
            return
        
        if current_time - self.last_position_time >= OBSTACLE_POSITION_UPDATE_TIME:
            if np.linalg.norm(point - self.last_position) >= OBSTACLE_MOVEMENT_UPDATE_THRESHOLD:
                self.current_movement = point - self.last_position
            self.last_position = point
            # print(current_time - self.last_position_time, self.last_position)
            self.last_position_time = current_time
    
    def get_movement(self) -> np.array:
        """Gibt die geschätzte Bewegung zurück."""
        return self.current_movement

    def get_simple_distance_point(self, point: tuple):
        return (
            np.sqrt((self.x - point[0]) ** 2 + (self.y - point[1]) ** 2)
        )

    def get_shell_distance_point(self, point: tuple):
        return (
            np.sqrt((self.x - point[0]) ** 2 + (self.y - point[1]) ** 2) - self.radius
        )

    def get_shell_distance(self, point: tuple, shift:np.array = np.array((0,0))):
        global ROBOT_CIRCLE_SIZE
        return (np.sqrt((self.x + shift[0] - point[0]) ** 2 + (self.y + shift[1] - point[1]) ** 2) - (self.radius + ROBOT_CIRCLE_SIZE))



class Updater:
    """Klasse welche die internen Repräsentationen der Hindernisse updatet und eine neu Bewegung des Roboters publisht."""
    pub = rospy.Publisher

    def __init__(self) -> None:
        global OBSTACLES
        OBSTACLES = ()
        self.current_point = None
        self.current_orientation = None
        self.last_movement = 0
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber('gazebo/model_states', ModelStates, self.update_and_move, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(ROBOT_UPDATE_TIME), self.calculate_next_moving_command)


    def update_and_move(self, model: ModelStates):
        global OBSTACLES
        # OBSTACLES = ()
        current_point = None

        name_list = [o.name for o in OBSTACLES]
        for i in range(len(model.name)):

            if model.name[i].startswith('turtlebot'):
                current_point = np.array((model.pose[i].position.x,model.pose[i].position.y))

                quaternion = (model.pose[i].orientation.x, model.pose[i].orientation.y, model.pose[i].orientation.z, model.pose[i].orientation.w)
                euler = euler_from_quaternion(quaternion)
                current_orientation = euler[2]

                self.current_point, self.current_orientation = current_point, current_orientation
        
            if model.name[i].startswith('cylinder_clone'):
                if model.name[i] not in name_list or len(model.name[i]) == 0:
                    OBSTACLES = (*OBSTACLES, Velocity_Circle(model.pose[i].position.x,model.pose[i].position.y,CYLINDER_DEFAULT_RADIUS, model.name[i]))
                else:
                    measured_point = np.array((model.pose[i].position.x,model.pose[i].position.y))
                    OBSTACLES[name_list.index(model.name[i])].update_position(measured_point)

        self.calculate_next_moving_command(current_point, current_orientation)


    def calculate_next_moving_command(self, current_point = None, current_orientation = None):
        msg = Twist()
        # msg.linear.x, msg.angular.z = self._get_controll_commands(self.current_point, self.current_orientation)
        msg.linear.x, msg.angular.z = self._get_controll_commands_with_adjustment(self.current_point, self.current_orientation)
        self.pub.publish(msg)

        # print(msg.linear.x, msg.angular.z)

    def _get_controll_commands(self, current_point, current_orientation):
        # using https://medium.com/@sarim.mehdi.550/mapping-path-following-for-a-two-wheeled-robot-b8bd55214405
        pot_force_vektor = potential_force(current_point, GOAL, OBSTACLES)           # <----------------------------
        pot_force_vektor_norm = np.linalg.norm(pot_force_vektor)
        if ROBOT_MAX_VELOCITY < pot_force_vektor_norm:
            pot_force_vektor = pot_force_vektor / pot_force_vektor_norm * ROBOT_MAX_VELOCITY
        #############

        b = pot_force_vektor_norm
        control_commands = np.array([[np.cos(current_orientation), np.sin(current_orientation)], [-1/b * np.sin(current_orientation), 1/b * np.cos(current_orientation)]]) @ pot_force_vektor
        return control_commands
    
    def _get_controll_commands_with_adjustment(self, current_point, current_orientation):
        # using https://medium.com/@sarim.mehdi.550/mapping-path-following-for-a-two-wheeled-robot-b8bd55214405
        # non_relativ_force_vektor = potential_force(current_point)
        #############
        # apruptes Ändern verhindern:
        a = 0.8
        dt = ROBOT_UPDATE_TIME # ROSPY_RATE/Timer abhängig
        # pot_force_vektor = potential_force(current_point,current_orientation)
        pot_force_vektor = potential_force(current_point, GOAL, OBSTACLES)
        # print(current_orientation, current_point)

        pot_force_vektor_norm = np.linalg.norm(pot_force_vektor)
        if ROBOT_MAX_VELOCITY < pot_force_vektor_norm:
            pot_force_vektor = pot_force_vektor / pot_force_vektor_norm * ROBOT_MAX_VELOCITY
        non_relativ_force_vektor = (1 - a * dt) * self.last_movement + (a * dt) * pot_force_vektor
        self.last_movement = non_relativ_force_vektor
        #############

        b = np.linalg.norm(non_relativ_force_vektor)
        control_commands = np.array([[np.cos(current_orientation), np.sin(current_orientation)], [-1/b * np.sin(current_orientation), 1/b * np.cos(current_orientation)]]) @ non_relativ_force_vektor

        return control_commands



def get_ros_params():
    try:
        global GOAL
        if rospy.has_param('/potential_field/goal_x') and rospy.has_param('/potential_field/goal_y'):
            GOAL = np.array((float(rospy.get_param('/potential_field/goal_x')), float(rospy.get_param('/potential_field/goal_y'))))
        else:
            GOAL = np.array((10, 0))

        global ROBOT_CIRCLE_SIZE
        if rospy.has_param('/potential_field/robot_circle_size'):
            ROBOT_CIRCLE_SIZE = rospy.get_param('/potential_field/robot_circle_size')
        else:
            ROBOT_CIRCLE_SIZE = 0.1
            
    except rospy.ROSInterruptException:
        rospy.loginfo('-> ERROR - Map_Obstacle.py - goal Parameter')



if __name__ == "__main__":
    get_ros_params()
    rospy.init_node("potential_field_mover", anonymous=False)
    ROSPY_RATE = rospy.Rate(1 / ROBOT_UPDATE_TIME) # in Hz 0.1s -> 10

    Updater()
    rospy.spin()
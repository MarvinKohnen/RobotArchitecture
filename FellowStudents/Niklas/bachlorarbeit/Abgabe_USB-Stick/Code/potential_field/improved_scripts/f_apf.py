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

################## Forward APF - Parameters ###############################

SCALING_FACTOR_ATTRACTION_FORCE = 1/20 #.5# 1/20
SCALING_FACTOR_REPULSIVE_FORCE = 5 #13/20 # 10# 13/20 #7/20
MIN_DISTANCE_REPULSIVE_FORCE = 3 #3
REPULSIVE_FORCE_FUTURE_COUNT = 3
OBSTACLE_FUTURE_ADJUSTMENT = 0.8

################## Forward APF - Code ###############################

def potential_force(point, GOAL, OBSTACLES):
    return attraction_force(point, GOAL) + repulsive_force(point, OBSTACLES)

def attraction_force(point, GOAL):
    return SCALING_FACTOR_ATTRACTION_FORCE * (GOAL - np.array(point))

def repulsive_force(point, OBSTACLES):
    rep_force = 0
    for circle in OBSTACLES:
        rep_force += movement_force3(point, circle)
    return rep_force


def movement_force3(point: np.array, obstacle):
    movement = obstacle.get_movement() #/ MOVEMENT_REPULSIVE_FORCE_COUNT

    # Falls das Objekt sich nicht bewegt -> normale Berechnung:
    if movement[0] == 0 and movement[1] == 0 or obstacle.get_shell_distance(point) > np.linalg.norm(movement * REPULSIVE_FORCE_FUTURE_COUNT) + MIN_DISTANCE_REPULSIVE_FORCE:
        distance = obstacle.get_shell_distance(point)
        if distance < MIN_DISTANCE_REPULSIVE_FORCE:
            # return 1/2 * SCALING_FACTOR_REPULSIVE_FORCE * (1/distance - 1/MIN_DISTANCE_REPULSIVE_FORCE)**2
            position_with_shift = obstacle.get_position()
            return SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / np.linalg.norm(np.array(point) - position_with_shift)**3
        else: return 0

    # in 3 Sektoren aufteilen:
    # - hinter dem richtigem Hinderniss
    # - neben den Projezierten Hindernisse
    # - vor dem am weitesten in der Zukunft liegende Hinderniss

    movement_field_vector = 0
    point_in_rectangle = False

    # Rectangle check:  https://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle
    normalized_movement = movement / np.linalg.norm(movement)
    normalized_orthogonal_movement = np.dot(np.array(((0,-1),(1,0))), normalized_movement)
    rectangle_point_1 = obstacle.get_position() - normalized_orthogonal_movement * MIN_DISTANCE_REPULSIVE_FORCE
    # rectangle_point_2 = obstacle.get_position() + movement * REPULSIVE_FORCE_FUTURE_COUNT - normalized_orthogonal_movement * MIN_DISTANCE_REPULSIVE_FORCE
    AM = point - rectangle_point_1
    AB = movement * REPULSIVE_FORCE_FUTURE_COUNT
    AD = normalized_orthogonal_movement * MIN_DISTANCE_REPULSIVE_FORCE * 2
    if 0 < np.dot(AM, AB) < np.dot(AB, AB) and 0 < np.dot(AM, AD) < np.dot(AD, AD):
        # Falls der Punkt nun in diesem rechteck ist -> orthogonal Abstand betrachten:
        # orth_distance = np.dot(-normalized_orthogonal_movement, point) - (obstacle.radius + ROBOT_CIRCLE_SIZE)

        # orth_distance1 = np.dot(normalized_orthogonal_movement, point) - (obstacle.radius + ROBOT_CIRCLE_SIZE)
        # orth_distance2 = np.dot(-normalized_orthogonal_movement, point) - (obstacle.radius + ROBOT_CIRCLE_SIZE)
        # orth_distance = min(abs(orth_distance1), abs(orth_distance2))
        # orth_distance = abs(np.dot(normalized_orthogonal_movement, point)) - (obstacle.radius + ROBOT_CIRCLE_SIZE)
        # orth_distance = np.dot(normalized_orthogonal_movement, point) - (obstacle.radius + ROBOT_CIRCLE_SIZE)

        point_on_middle_line = obstacle.get_position()
        vektor_between_line_and_point = (point - point_on_middle_line) - np.dot(np.dot(point - point_on_middle_line, normalized_movement), normalized_movement)
        distance_adjustment_scaling = (np.linalg.norm(-vektor_between_line_and_point + point - point_on_middle_line) / np.linalg.norm(movement * REPULSIVE_FORCE_FUTURE_COUNT))
        
        orth_distance = np.linalg.norm(vektor_between_line_and_point) + OBSTACLE_FUTURE_ADJUSTMENT *distance_adjustment_scaling - (obstacle.radius + ROBOT_CIRCLE_SIZE)
        
        sign = -1 if np.dot(-vektor_between_line_and_point, normalized_orthogonal_movement) < 0 else 1
        position_with_shift = sign * normalized_orthogonal_movement * orth_distance + point
        # position_with_shift = -vektor_between_line_and_point + point        ## TODO: anpassen damit nahc vorne gedrückt Robo

        movement_field_vector += SCALING_FACTOR_REPULSIVE_FORCE * (1 / orth_distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / orth_distance**3
        point_in_rectangle = True
        # return 10

    distance_to_obstacle = obstacle.get_shell_distance(point)
    distance_to_future_obstacle = obstacle.get_shell_distance(point, shift=movement * REPULSIVE_FORCE_FUTURE_COUNT)
    distance = min(distance_to_obstacle, distance_to_future_obstacle)
    if distance < MIN_DISTANCE_REPULSIVE_FORCE:
        if distance == distance_to_obstacle:
            position_with_shift = obstacle.get_position()
            movement_field_vector += SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
            # movement_field_vector/=2
        else:
            if point_in_rectangle:
                return movement_field_vector
            distance += OBSTACLE_FUTURE_ADJUSTMENT

            position_with_shift = obstacle.get_position() + movement * REPULSIVE_FORCE_FUTURE_COUNT
            if position_with_shift[0] == point[0] and position_with_shift[1] == point[1]:
                position_with_shift = position_with_shift - normalized_movement*distance
            else:
                position_with_shift = position_with_shift + (position_with_shift - point) / np.linalg.norm(position_with_shift - point) * distance
            #
            return SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
        
    return movement_field_vector



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
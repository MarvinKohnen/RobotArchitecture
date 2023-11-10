import numpy as np
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import time

CYLINDER_DEFAULT_RADIUS = 0.2

ROBOT_MAX_VELOCITY = 1

# MAX_LINEAR_VELOCITY = 1
# ANGULAR_VELOCITY = 20
# GOAL = np.array((10, 0))

SCALING_FACTOR_ATTRACTION_FORCE = 1/20
SCALING_FACTOR_REPULSIVE_FORCE = 1 #13/20 #7/20
SCALING_FACTOR_REPULSIVE_ROTATION_FORCE = SCALING_FACTOR_REPULSIVE_FORCE * 1.5
MIN_DISTANCE_REPULSIVE_FORCE = 6 #3

REPULSIVE_FORCE_FUTURE_COUNT = 3

OBSTACLE_POSITION_UPDATE_TIME = 1
OBSTACLE_MOVEMENT_UPDATE_THRESHOLD = 0.05

OBSTACLE_FUTURE_ADJUSTMENT = 0.8

CLOCKWISE_MATRIX = np.array(((0,1),(-1,0)))
COUNTERCLOCKWISE_MATRIX = np.array(((0,-1),(1,0)))


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
        rospy.Subscriber('gazebo/model_states', ModelStates, self.update_and_move)
        self._timer = rospy.Timer(rospy.Duration(0.1), self.calculate_next_moving_command)


    def update_and_move(self, model: ModelStates):
        global OBSTACLES
        # OBSTACLES = ()
        current_point = None

        name_list = [o.name for o in OBSTACLES]
        for i in range(len(model.name)):

            if model.name[i].startswith('turtlebot'):
                current_point = np.array((model.pose[i].position.x,model.pose[i].position.y))

                # current_twist = model.twist[i]
                # print(current_twist)

                # current_orientation = np.array((model.pose[i].orientation.x,model.pose[i].orientation.y))

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

        # self.calculate_next_moving_command(current_point, current_orientation)

    def calculate_next_moving_command(self, current_point = None, current_orientation = None):
        msg = Twist()
        msg.linear.x, msg.angular.z = self._get_controll_commands_1(self.current_point, self.current_orientation)
        self.pub.publish(msg)


    def _get_controll_commands_1(self, current_point, current_orientation):
        # using https://medium.com/@sarim.mehdi.550/mapping-path-following-for-a-two-wheeled-robot-b8bd55214405
        # non_relativ_force_vektor = potential_force(current_point)
        #############
        # apruptes Ändern verhindern:
        a = 0.7
        dt = 0.1 # ROSPY_RATE/Timer abhängig
        # pot_force_vektor = potential_force(current_point,current_orientation)
        pot_force_vektor = potential_force(current_point,current_point)
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



def attraction_force(point):
    return SCALING_FACTOR_ATTRACTION_FORCE * (GOAL - np.array(point))


def repulsive_force(point: np.array, current_orientation_robot: np.array):
    rep_force = 0
    for circle in OBSTACLES:
        rep_force += movement_force2(point, circle, current_orientation_robot)
    return rep_force


def movement_force2(point: np.array, obstacle: Velocity_Circle, current_orientation_robot: np.array):

    # TODO: testen, ob current orientation bei rotation nicht der Vektor zum Ziel ist und nicht aktuell orientierung
    current_orientation_robot = GOAL - current_orientation_robot
    # current_orientation_robot = np.arctan2(current_orientation_robot[1], current_orientation_robot[0])

    movement = obstacle.get_movement() #/ MOVEMENT_REPULSIVE_FORCE_COUNT

    # Falls das Objekt sich nicht bewegt -> normale Berechnung:
    if movement[0] == 0 and movement[1] == 0:# or obstacle.get_shell_distance(point) > np.linalg.norm(movement * REPULSIVE_FORCE_FUTURE_COUNT) + MIN_DISTANCE_REPULSIVE_FORCE:  # um Performanc zusparen ggf
        distance = obstacle.get_shell_distance(point)
        if distance < MIN_DISTANCE_REPULSIVE_FORCE:
            position_with_shift = obstacle.get_position()

            temp = (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
            rep_force_normal = SCALING_FACTOR_REPULSIVE_FORCE * temp
            rep_force_with_rotation = get_rotational_repulsive_force(temp, current_orientation_robot)
            return rep_force_normal + rep_force_with_rotation
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
    rectangle_point_1 = obstacle.get_position() + normalized_orthogonal_movement * MIN_DISTANCE_REPULSIVE_FORCE
    AM = point - rectangle_point_1
    AB = movement * REPULSIVE_FORCE_FUTURE_COUNT
    AD = - normalized_orthogonal_movement * MIN_DISTANCE_REPULSIVE_FORCE * 2
    if 0 < np.dot(AM, AB) < np.dot(AB, AB) and 0 < np.dot(AM, AD) < np.dot(AD, AD):
        # Falls der Punkt nun in diesem rechteck ist -> orthogonal ABstand betrachten:
        # orth_distance = abs(np.dot(normalized_orthogonal_movement, point)) #- (obstacle.radius + ROBOT_CIRCLE_SIZE)
        # position_with_shift = point - (np.dot(normalized_orthogonal_movement, point) ) * normalized_orthogonal_movement

        point_on_middle_line = obstacle.get_position()
        vektor_between_line_and_point = (point - point_on_middle_line) - np.dot(np.dot(point - point_on_middle_line, normalized_movement), normalized_movement)
        distance_adjustment_scaling = (np.linalg.norm(-vektor_between_line_and_point + point - point_on_middle_line) / np.linalg.norm(movement * REPULSIVE_FORCE_FUTURE_COUNT))
        
        orth_distance = np.linalg.norm(vektor_between_line_and_point) + OBSTACLE_FUTURE_ADJUSTMENT *distance_adjustment_scaling - (obstacle.radius + ROBOT_CIRCLE_SIZE)

        sign = -1 if np.dot(-vektor_between_line_and_point, normalized_orthogonal_movement) < 0 else 1
        position_with_shift = sign * normalized_orthogonal_movement * orth_distance + point

        temp = (1 / orth_distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / orth_distance**3
        rep_force_normal = SCALING_FACTOR_REPULSIVE_FORCE * temp
        rep_force_with_rotation = get_rotational_repulsive_force(temp, current_orientation_robot)
        movement_field_vector += rep_force_normal + rep_force_with_rotation
        point_in_rectangle = True
        
    distance_to_obstacle = obstacle.get_shell_distance(point)
    distance_to_future_obstacle = obstacle.get_shell_distance(point, shift=movement * REPULSIVE_FORCE_FUTURE_COUNT)
    distance = min(distance_to_obstacle, distance_to_future_obstacle)
    if distance < MIN_DISTANCE_REPULSIVE_FORCE:
        if distance == distance_to_obstacle:
            position_with_shift = obstacle.get_position()
            #
            temp = (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
            rep_force_normal = SCALING_FACTOR_REPULSIVE_FORCE * temp
            rep_force_with_rotation = get_rotational_repulsive_force(temp, current_orientation_robot)
            movement_field_vector += rep_force_normal + rep_force_with_rotation
            # movement_field_vector/=2
            #
            # movement_field_vector += SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
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
            temp = (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
            rep_force_normal = SCALING_FACTOR_REPULSIVE_FORCE * temp
            rep_force_with_rotation = get_rotational_repulsive_force(temp, current_orientation_robot)
            return rep_force_normal + rep_force_with_rotation
            #
            # return SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
           
    return movement_field_vector


def get_rotational_repulsive_force2(temp_term: np.array, current_direction: np.array, obstacle_position: np.array, obstacle_future_position: np.array):

    # if type(current_direction) == np.ndarray:
    #     print(current_direction, type(current_direction))
    #     return 0
    
    direction_angle = current_direction
    
    # direction_angle = np.arctan2(current_direction[1], current_direction[0])  # ist current _direction überhaupt ein vektor!?!?!? TODO

    obstacle_angle = np.arctan2(obstacle_position[1], obstacle_position[0])   # anscheinend nicht gebraucht
    obstacle_future_angle = np.arctan2(obstacle_future_position[1], obstacle_future_position[0])

    # TODO: testen ob dies so stimmt
    rotation_matrix = CLOCKWISE_MATRIX if direction_angle - obstacle_future_angle >= 0 else COUNTERCLOCKWISE_MATRIX 


    # phi_first = direction_angle - obstacle_angle
    # phi_end = direction_angle - obstacle_future_angle
    # rotation_matrix = None
    # if phi_first <= 0 and phi_end > 0:
    #     rotation_matrix = CLOCKWISE_MATRIX
    # if phi_first > 0 and phi_end <= 0:
    #     rotation_matrix = COUNTERCLOCKWISE_MATRIX
    # if phi_first > 0 and phi_end > 0:
    #     rotation_matrix = CLOCKWISE_MATRIX
    # if phi_first <= 0 and phi_end <= 0:
    #     rotation_matrix = COUNTERCLOCKWISE_MATRIX
        
    # if not np.array_equal(rotation_matrix2, rotation_matrix):
    #     print('hier', obstacle_position, obstacle_future_position, temp_term)



    # rotation_matrix = CLOCKWISE_MATRIX if direction_angle - obstacle_angle >= 0 else COUNTERCLOCKWISE_MATRIX  # berücksichtigt nicht die projektion
    return SCALING_FACTOR_REPULSIVE_ROTATION_FORCE * np.dot(rotation_matrix, temp_term)


def get_rotational_repulsive_force1(temp_term: np.array, current_direction: np.array, obstacle_position: np.array, obstacle_future_position: np.array):

    ortho_direction_right = np.dot(CLOCKWISE_MATRIX, current_direction)
    obstacle_angle = np.arctan2(obstacle_position[1], obstacle_position[0])   # anscheinend nicht gebraucht
    obstacle_future_angle = np.arctan2(obstacle_future_position[1], obstacle_future_position[0])

    phi_first, phi_end = np.dot(ortho_direction_right,obstacle_position),np.dot(ortho_direction_right,obstacle_future_position)

    # TODO: testen ob dies so stimmt
    rotation_matrix = COUNTERCLOCKWISE_MATRIX if phi_end <= 0 else CLOCKWISE_MATRIX 

    # rotation_matrix = None
    # if phi_first <= 0 and phi_end > 0:
    #     rotation_matrix = CLOCKWISE_MATRIX
    # elif phi_first > 0 and phi_end <= 0:
    #     rotation_matrix = COUNTERCLOCKWISE_MATRIX
    # elif phi_first > 0 and phi_end > 0:
    #     rotation_matrix = COUNTERCLOCKWISE_MATRIX
    # elif phi_first <= 0 and phi_end <= 0:
    #     rotation_matrix = CLOCKWISE_MATRIX
        
    # if not np.array_equal(rotation_matrix2, rotation_matrix):
    #     print('hier', obstacle_position, obstacle_future_position, temp_term)

    return SCALING_FACTOR_REPULSIVE_ROTATION_FORCE * np.dot(rotation_matrix, temp_term)


def get_rotational_repulsive_force42(temp_term: np.array, current_direction: np.array, obstacle_position: np.array, obstacle_future_position: np.array):
    ortho_direction_right = np.dot(CLOCKWISE_MATRIX, current_direction)

    phi_first, phi_end = np.dot(ortho_direction_right,obstacle_position),np.dot(ortho_direction_right,obstacle_future_position)

    phi_front_first, phi_back_first = np.dot(current_direction,obstacle_position),np.dot(current_direction,obstacle_future_position)

    # if phi_front_first <= 0 or phi_back_first <= 0: 
    if phi_front_first <= 0: 
    # if phi_back_first <= 0: 
        # print(np.dot(current_direction,obstacle_position))
        return 0

    # TODO: testen ob dies so stimmt
    rotation_matrix = COUNTERCLOCKWISE_MATRIX if phi_end < 0 else CLOCKWISE_MATRIX 

    # rotation_matrix = None
    # if phi_first < 0 and phi_end >= 0:
    #     rotation_matrix = CLOCKWISE_MATRIX
    # elif phi_first >= 0 and phi_end < 0:
    #     rotation_matrix = COUNTERCLOCKWISE_MATRIX
    # elif phi_first >= 0 and phi_end >= 0:
    #     rotation_matrix = CLOCKWISE_MATRIX
    # elif phi_first < 0 and phi_end < 0:
    #     rotation_matrix = COUNTERCLOCKWISE_MATRIX
        
    # if not np.array_equal(rotation_matrix2, rotation_matrix):
    #     print('hier', obstacle_position, obstacle_future_position, temp_term)

    return SCALING_FACTOR_REPULSIVE_ROTATION_FORCE * np.dot(rotation_matrix, temp_term)



def get_rotational_repulsive_force(temp_term: np.array, current_direction: np.array):
    ortho_direction_right = np.dot(CLOCKWISE_MATRIX, current_direction)
    moving_towards_right = np.dot(ortho_direction_right,current_direction) >= 0
    rotation_matrix = COUNTERCLOCKWISE_MATRIX if moving_towards_right < 0 else CLOCKWISE_MATRIX 
    return SCALING_FACTOR_REPULSIVE_ROTATION_FORCE * np.dot(rotation_matrix, temp_term)


def potential_force(point: np.array, current_orientation_robot: np.array):
    # return attraction_force(point) + repulsive_force(point, current_orientation_robot)

    force1 = attraction_force(point) 
    force2 = repulsive_force(point, current_orientation_robot)
    # print('at_force:',force1, 'rep_force: ', force2)
    return force1 + force2


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


#####################################################

from matplotlib import cm
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
import itertools
import matplotlib.colors as mcolors

def show_field(field_func, start_point: tuple, end_point: tuple, increment: float, min_dist_to_obstacles: float = .1):
    x = start_point[0]
    y = start_point[1]
    field = ()
    while x <= end_point[0]:
        y = start_point[1]
        while y <= end_point[1]:
            # point_in_obstacle = False
            # for circle in OBSTACLES:
            #         if circle.get_shell_distance_point((x,y)) <= min_dist_to_obstacles:
            #             point_in_obstacle = True
            # if not point_in_obstacle:
            #     field = (*field, (x, y, field_func((x, y))))

            value = field_func((x, y))
            field = (*field, (x, y, value if value < 10 else 10))
            y += increment
        x += increment
    plotten_3D_2(field)


def plotten_3D_2(data):

    x = [x_koord for (x_koord, y_koord, z_koord) in data]
    y = [y_koord for (x_koord, y_koord, z_koord) in data]
    z = [z_koord for (x_koord, y_koord, z_koord) in data]

    xv = np.linspace(np.min(x), np.max(x), 100)
    yv = np.linspace(np.min(y), np.max(y), 100)
    [X,Y] = np.meshgrid(xv, yv)
    Z = griddata((x,y),z,(X,Y),method='linear')

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(projection="3d")

    surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('force')
    fig.colorbar(surf, shrink=0.6)
    plt.show()


def get_random_moving_obstacles(count: int) -> tuple:
    obstacles = ()
    for i in range(count):
        random_values_pos = np.random.rand(2) * 12 - 6
        random_values_movement = np.random.rand(2) - np.array((1,1))
        random_values_movement = random_values_movement / np.linalg.norm(random_values_movement) * 1

        # random_values_movement = np.array((0,1))

        obstacle_i = Velocity_Circle(*random_values_pos,CYLINDER_DEFAULT_RADIUS, f'obstacle_{i}')
        obstacle_i.current_movement = random_values_movement
        obstacles = (*obstacles, obstacle_i)
        print(random_values_movement)


    test_circle = Velocity_Circle(-1, -1, CYLINDER_DEFAULT_RADIUS, 'test_kreis')
    # test_circle.current_movement = np.array((2,1))
    test_circle.current_movement = np.array((1,1))
    obstacles = (test_circle,)

    return obstacles

def draw_rotational_forward_example():
    global OBSTACLES
    OBSTACLES = get_random_moving_obstacles(4)
    GOAL = np.array((10,0))

    # show_field(repulsive_field,start_point=(-6, -6), end_point=(6, 6), increment=.1)
    # show_field(potential_field,start_point=(-6, -6), end_point=(6, 6), increment=.1)

    show_map(with_path=False, max_vector_norm=ROBOT_MAX_VELOCITY)
    # show_map(with_path=True, path_steps=300, max_vector_norm=5)

    # test_map_multiple_paths(path_count=10, max_vector_norm=ROBOT_MAX_VELOCITY, path_steps=500)


def show_map(with_force = True, with_path=False, path_steps=-1, max_vector_norm=None):
    # fig, ax = plt.subplots()
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot()
    for circle in OBSTACLES:
        ax.add_patch(plt.Circle(circle.get_position(), circle.radius, color="r"))
        plt.quiver(*circle.get_position(), *circle.current_movement*3, color='g', units='xy', scale=1)
    # plt.scatter(*GOAL)

    if with_force:
        x = -6
        y = -6
        increment = .5
        while x < 6:
            y = -6
            while y < 6:
                draw = True
                for circle in OBSTACLES:
                    if circle.get_shell_distance((x,y)) <= 0:
                        draw = False
                
                if draw:
                    # vector = repulsive_force(np.array((x,y)), current_orientation_robot=np.array((x,y))) + np.array((0,0))
                    vector = potential_force(np.array((x,y)), current_orientation_robot=np.array((x,y)))

                    # if max_vector_norm is None or np.linalg.norm(vector) < max_vector_norm:
                    #     if vector[0] != 0 and vector[1] != 0: 

                    #         plt.quiver(x, y, *vector, color='black', units='xy', scale=1)

                    if max_vector_norm is None and max_vector_norm != 0:
                        plt.quiver(x, y, *vector, color='black', units='xy', scale=1)
                    else:
                        pot_force_vektor_norm = np.linalg.norm(vector)
                        if ROBOT_MAX_VELOCITY < pot_force_vektor_norm:
                            vector = vector / pot_force_vektor_norm * ROBOT_MAX_VELOCITY
                        plt.quiver(x, y, *vector, color='black', units='xy', scale=1)

                # plt.quiver(x, y, *potential_force((x,y)), color='b', units='xy', scale=20)
                y += increment
            x += increment

    if with_path:
        current_pos = np.array((-10,1))
        step_counter = 0
        while(np.linalg.norm(current_pos - GOAL) > 0.1 and step_counter < path_steps):
            # pot_force = repulsive_force((x,y), current_orientation_robot=np.array((x,y))) + np.array((0,0))
            pot_force = potential_force(current_pos, current_orientation_robot=np.array((x,y)))
            # plt.quiver(current_pos[0], current_pos[1], *pot_force, color='r', units='xy', scale=1)
            plt.plot(*current_pos, '.', color = 'r')
            current_pos = current_pos + pot_force*0.1
            step_counter += 1


    # plt.xlim(0, 12)
    # plt.ylim(0, 12)
    plt.show()


def test_map_multiple_paths(with_force = True, path_steps=100, max_vector_norm=None, path_count = 1):
    # colors = itertools.cycle(["r", "b", "g"])
    all_colors = mcolors.TABLEAU_COLORS
    colors = itertools.cycle(all_colors)
    # fig, ax = plt.subplots()
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot()
    for circle in OBSTACLES:
        ax.add_patch(plt.Circle(circle.get_position(), circle.radius, color="r"))
        plt.quiver(*circle.get_position(), *circle.current_movement*3, color='g', units='xy', scale=1)
    # plt.scatter(*GOAL)

    if with_force:
        x = -10
        y = -10
        increment = .5
        while x < 10:
            y = -10
            while y < 10:
                draw = True
                for circle in OBSTACLES:
                    if circle.get_shell_distance((x,y)) <= 0:
                        draw = False
                
                if draw:
                    # vector = repulsive_force(np.array((x,y)), current_orientation_robot=np.array((x,y))) + np.array((0,0))
                    vector = potential_force(np.array((x,y)), current_orientation_robot=np.array((x,y)))

                    # if max_vector_norm is None or np.linalg.norm(vector) < max_vector_norm:
                    #     if vector[0] != 0 and vector[1] != 0: 

                    #         plt.quiver(x, y, *vector, color='black', units='xy', scale=1)

                    if max_vector_norm is None and max_vector_norm != 0:
                        plt.quiver(x, y, *vector, color='black', units='xy', scale=1)
                    else:
                        pot_force_vektor_norm = np.linalg.norm(vector)
                        if ROBOT_MAX_VELOCITY < pot_force_vektor_norm:
                            vector = vector / pot_force_vektor_norm * ROBOT_MAX_VELOCITY
                        plt.quiver(x, y, *vector, color='black', units='xy', scale=1)


                # plt.quiver(x, y, *potential_force((x,y)), color='b', units='xy', scale=20)
                y += increment
            x += increment


    path_walked = 0
    while path_walked < path_count:
        current_pos = np.array((-10,np.random.uniform(-10,10)))
        if np.random.uniform(-1,1) > 0:
            # current_pos = np.array((-10,np.random.uniform(-10,10)))
            current_pos = current_pos[::-1]
        # else:
        #     # current_pos = np.array((np.random.uniform(-10,10), -10))
        step_counter = 0
        color_for_this_path = next(colors)
        while(np.linalg.norm(current_pos - GOAL) > 0.1 and step_counter < path_steps):
            # pot_force = repulsive_force((x,y), current_orientation_robot=np.array((x,y))) + np.array((0,0))
            pot_force = potential_force(current_pos, current_orientation_robot=np.array((x,y)))
            # plt.quiver(current_pos[0], current_pos[1], *pot_force, color='r', units='xy', scale=1)
            plt.plot(*current_pos, '.', color = color_for_this_path)

            pot_force_vektor_norm = np.linalg.norm(pot_force)
            if ROBOT_MAX_VELOCITY < pot_force_vektor_norm:
                pot_force = pot_force / pot_force_vektor_norm * ROBOT_MAX_VELOCITY

            current_pos = current_pos + pot_force*0.1
            step_counter += 1
        path_walked += 1

    # plt.xlim(0, 12)
    # plt.ylim(0, 12)
    plt.show()

#####################################################



if __name__ == "__main__":
    get_ros_params()
    rospy.init_node("simple_potential_field_mover", anonymous=False)

    # Updater()
    # rospy.spin()

    ### experimentell:
    draw_rotational_forward_example()
import numpy as np
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Twist
import tf
from tf.transformations import euler_from_quaternion
import time

CYLINDER_DEFAULT_RADIUS = 0.2

ROBOT_MAX_VELOCITY = 1

# MAX_LINEAR_VELOCITY = 1
# ANGULAR_VELOCITY = 20
# GOAL = np.array((10, 0))

SCALING_FACTOR_ATTRACTION_FORCE = 1/20
SCALING_FACTOR_REPULSIVE_FORCE = 13/20 #7/20
MIN_DISTANCE_REPULSIVE_FORCE = 2 #3

REPULSIVE_FORCE_FUTURE_COUNT = 3

OBSTACLE_POSITION_UPDATE_TIME = 1
OBSTACLE_MOVEMENT_UPDATE_THRESHOLD = 0.05



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
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber('gazebo/model_states', ModelStates, self.update_and_move)


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
        
            if model.name[i].startswith('cylinder_clone'):
                if model.name[i] not in name_list or len(model.name[i]) == 0:
                    OBSTACLES = (*OBSTACLES, Velocity_Circle(model.pose[i].position.x,model.pose[i].position.y,CYLINDER_DEFAULT_RADIUS, model.name[i]))
                else:
                    measured_point = np.array((model.pose[i].position.x,model.pose[i].position.y))
                    OBSTACLES[name_list.index(model.name[i])].update_position(measured_point)

        self.calculate_next_moving_command(current_point, current_orientation)

    def calculate_next_moving_command(self, current_point, current_orientation):
        msg = Twist()
        msg.linear.x, msg.angular.z = self._get_controll_commands_1(current_point, current_orientation)
        self.pub.publish(msg)


    def _get_controll_commands_1(self, current_point, current_orientation):
        # using https://medium.com/@sarim.mehdi.550/mapping-path-following-for-a-two-wheeled-robot-b8bd55214405
        non_relativ_force_vektor = potential_force(current_point)

        b = np.linalg.norm(non_relativ_force_vektor)
        control_commands = np.array([[np.cos(current_orientation), np.sin(current_orientation)], [-1/b * np.sin(current_orientation), 1/b * np.cos(current_orientation)]]) @ non_relativ_force_vektor

        

        # control_commands[0] = ROBOT_MAX_VELOCITY * np.sign(control_commands[0]) * 1/(1 + np.exp(-abs(control_commands[0]))) # SIGMOID

        # simple max/min OPeration for maxSpeed:
        if control_commands[0] < 0:
            control_commands[0] = max(control_commands[0], - ROBOT_MAX_VELOCITY)
        else:
            control_commands[0] = min(control_commands[0], ROBOT_MAX_VELOCITY)

        return control_commands
    
    def _get_controll_commands_2(self, current_point, current_orientation):
        # siehe https://www.mdpi.com/2218-6581/12/3/81
        pass



def attraction_field(point):
    # return 1/2 * SCALING_FACTOR_ATTRACTION_FORCE * np.linalg.norm(GOAL - np.array(point))**2
    return 1/2 * SCALING_FACTOR_ATTRACTION_FORCE * np.dot(
        GOAL - np.array(point), GOAL - np.array(point)
    )

def repulsive_field(point):
    rep_field = 0
    for circle in OBSTACLES:
        rep_field += movement_field2(point, circle)
    return rep_field


def movement_field(point: np.array, obstacle: Velocity_Circle):
    add_movement = obstacle.get_movement() #/ MOVEMENT_REPULSIVE_FORCE_COUNT

    rep_field = 0
    for i in range(REPULSIVE_FORCE_FUTURE_COUNT +1 if add_movement[0] != 0 and add_movement[1] != 0 else 1):
        shift = add_movement * i
        position_with_shift = obstacle.get_position() + shift
        distance = obstacle.get_shell_distance(point, shift=shift)
        if distance < MIN_DISTANCE_REPULSIVE_FORCE:
            rep_field += 1/2 * SCALING_FACTOR_REPULSIVE_FORCE * (1/distance - 1/MIN_DISTANCE_REPULSIVE_FORCE)**2
    # print(obstacle.name, rep_field)
    return rep_field

def movement_field2(point: np.array, obstacle: Velocity_Circle):
    movement = obstacle.get_movement() #/ MOVEMENT_REPULSIVE_FORCE_COUNT

    # in 3 Sektoren aufteilen:
    # - hinter dem richtigem Hinderniss
    # - neben den Projezierten Hindernisse
    # - vor dem am weitesten in der Zukunft liegende Hinderniss


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
        orth_distance = np.linalg.norm(vektor_between_line_and_point) - (obstacle.radius + ROBOT_CIRCLE_SIZE)
        # position_with_shift = -vektor_between_line_and_point + point

        return 1/2 * SCALING_FACTOR_REPULSIVE_FORCE * (1/orth_distance - 1/MIN_DISTANCE_REPULSIVE_FORCE)**2
        # return 10
    
    distance_to_real_obstacle = np.linalg.norm(obstacle.get_position() - point) 
    distance_to_future_obstacle = np.linalg.norm(obstacle.get_position() + movement * REPULSIVE_FORCE_FUTURE_COUNT - point)

    distance = min(distance_to_real_obstacle, distance_to_future_obstacle) - (obstacle.radius + ROBOT_CIRCLE_SIZE)
    if distance < MIN_DISTANCE_REPULSIVE_FORCE:
        # if distance != distance_to_real_obstacle:
        #     distance -= (obstacle.radius + ROBOT_CIRCLE_SIZE)
        return 1/2 * SCALING_FACTOR_REPULSIVE_FORCE * (1/distance - 1/MIN_DISTANCE_REPULSIVE_FORCE)**2

    return 0


def potential_field(point):
    return attraction_field(point) + repulsive_field(point)


def attraction_force(point):
    return SCALING_FACTOR_ATTRACTION_FORCE * (GOAL - np.array(point))

    # force = SCALING_FACTOR_ATTRACTION_FORCE * (GOAL - np.array(point))
    # print(force)
    # return force


def repulsive_force(point):
    rep_force = 0
    for circle in OBSTACLES:
        rep_force += movement_force2(point, circle)

    # print(rep_force)
    return rep_force

def movement_force(point: np.array, obstacle: Velocity_Circle):
    add_movement = obstacle.get_movement() #/ MOVEMENT_REPULSIVE_FORCE_COUNT

    rep_force = 0
    for i in range(REPULSIVE_FORCE_FUTURE_COUNT +1 if add_movement[0] != 0 and add_movement[1] != 0 else 1):
        shift = add_movement * i
        position_with_shift = obstacle.get_position() + shift
        distance = obstacle.get_shell_distance(point, shift=shift)
        if distance < MIN_DISTANCE_REPULSIVE_FORCE:
            rep_force += SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / np.linalg.norm(np.array(point) - position_with_shift)**3
    # print(obstacle.name, rep_force)
    return rep_force

def movement_force2(point: np.array, obstacle: Velocity_Circle):
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


    # Rectangle check:  https://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle
    normalized_movement = movement / np.linalg.norm(movement)
    normalized_orthogonal_movement = np.dot(np.array(((0,-1),(1,0))), normalized_movement)
    rectangle_point_1 = obstacle.get_position() + normalized_orthogonal_movement * MIN_DISTANCE_REPULSIVE_FORCE
    # rectangle_point_2 = obstacle.get_position() + movement * REPULSIVE_FORCE_FUTURE_COUNT - normalized_orthogonal_movement * MIN_DISTANCE_REPULSIVE_FORCE
    AM = point - rectangle_point_1
    AB = movement * REPULSIVE_FORCE_FUTURE_COUNT
    AD = - normalized_orthogonal_movement * MIN_DISTANCE_REPULSIVE_FORCE * 2
    if 0 < np.dot(AM, AB) < np.dot(AB, AB) and 0 < np.dot(AM, AD) < np.dot(AD, AD):
        # Falls der Punkt nun in diesem rechteck ist -> orthogonal ABstand betrachten:
        # orth_distance = abs(np.dot(normalized_orthogonal_movement, point)) #- (obstacle.radius + ROBOT_CIRCLE_SIZE)
        # position_with_shift = point - (np.dot(normalized_orthogonal_movement, point) ) * normalized_orthogonal_movement

        point_on_middle_line = obstacle.get_position()
        vektor_between_line_and_point = (point - point_on_middle_line) - np.dot(np.dot(point - point_on_middle_line, normalized_movement), normalized_movement)
        orth_distance = np.linalg.norm(vektor_between_line_and_point) - (obstacle.radius + ROBOT_CIRCLE_SIZE)
        position_with_shift = -vektor_between_line_and_point + point

        # print('hier',position_with_shift)
        # plt.scatter(*point, color='r')
        # plt.scatter(*position_with_shift, color='g')
        #
        return SCALING_FACTOR_REPULSIVE_FORCE * (1 / orth_distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / np.linalg.norm(np.array(point) - position_with_shift)**3
    
    distance_to_obstacle = np.linalg.norm(obstacle.get_position() - point)
    distance_to_future_obstacle = np.linalg.norm(obstacle.get_position() + movement * REPULSIVE_FORCE_FUTURE_COUNT - point)

    distance = min(distance_to_obstacle, distance_to_future_obstacle)
    if distance < MIN_DISTANCE_REPULSIVE_FORCE:
        # position_with_shift = obstacle.get_position() + (0 if distance == distance_to_obstacle else movement * REPULSIVE_FORCE_FUTURE_COUNT)
        if distance == distance_to_obstacle:
            position_with_shift = obstacle.get_position()
        else:
            position_with_shift = obstacle.get_position() + movement * REPULSIVE_FORCE_FUTURE_COUNT
            distance -= (obstacle.radius + ROBOT_CIRCLE_SIZE)
        return SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / np.linalg.norm(np.array(point) - position_with_shift)**3
    return 0

def repulsive_force_OLD(point):
    rep_force = 0
    for circle in OBSTACLES:
        distance = circle.get_shell_distance_point(point)
        if distance < MIN_DISTANCE_REPULSIVE_FORCE:
            rep_force += SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - circle.get_position()) / np.linalg.norm(np.array(point) - circle.get_position())
    return rep_force


def potential_force(point):
    return attraction_force(point) + repulsive_force(point)

    # force1 = attraction_force(point) 
    # force2 = repulsive_force(point)
    # print(force1, force2)
    # return force1 + force2


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

plt.rc('font', size=15) 

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

        obstacle_i = Velocity_Circle(*random_values_pos,CYLINDER_DEFAULT_RADIUS, f'obsatcle_{i}')
        obstacle_i.current_movement = random_values_movement
        obstacles = (*obstacles, obstacle_i)
        print(random_values_movement)


    test_circle = Velocity_Circle(-1,-1, CYLINDER_DEFAULT_RADIUS, 'test_kreis')
    # test_circle.current_movement = np.array((2,1))
    test_circle.current_movement = np.array((1,1))
    obstacles = (test_circle,)

    return obstacles

def draw_forward_example():
    global OBSTACLES
    OBSTACLES = get_random_moving_obstacles(4)
    GOAL = np.array((10,0))
    # show_field(repulsive_field,start_point=(-6, -6), end_point=(6, 6), increment=.1)
    show_field(potential_field,start_point=(-6, -6), end_point=(6, 6), increment=.1)
    show_map(with_path=False, path_steps=30)


def show_map(with_force = True, with_path=False, path_steps=-1):
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
                    if circle.get_shell_distance_point((x,y)) <= 0:
                        draw = False
                
                if draw:
                    vector = potential_force((x,y))
                    # plt.quiver(x, y, *potential_force((x,y)), color='b', units='xy', scale=1)
                    ROBOT_MAX_VELOCITY = 1

                    pot_force_vektor_norm = np.linalg.norm(vector)
                    if ROBOT_MAX_VELOCITY < pot_force_vektor_norm:
                        vector = vector / pot_force_vektor_norm * ROBOT_MAX_VELOCITY
                    plt.quiver(x, y, *vector, color='b', units='xy', scale=1)

                # plt.quiver(x, y, *potential_force((x,y)), color='b', units='xy', scale=20)
                y += increment
            x += increment

    if with_path:
        current_pos = np.array((-5,1))
        step_counter = 0
        while(np.linalg.norm(current_pos - GOAL) > 0.1 and step_counter < path_steps):
            pot_force = potential_force(current_pos)
            plt.quiver(current_pos[0], current_pos[1], *pot_force, color='r', units='xy', scale=1)
            current_pos = current_pos + pot_force
            step_counter += 1


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
    draw_forward_example()
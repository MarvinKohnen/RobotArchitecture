import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
from matplotlib.animation import FuncAnimation
import time
from sympy import Segment

import DifferentModels.classic_apf as capf
import DifferentModels.forward_apf as fapf
import DifferentModels.rotational_fapf as rfapf
import DifferentModels.improved_apf as iapf

import pickle
import sys

# plt.rc('font', size=15) 

#############################################################
class Robot:
    pos: np.array
    old_pos: np.array
    collisions: int
    walked_path: list
    time_steps: int
    used_model: str
    current_movement: np.array
    finished: bool

    def __init__(self, used_model: str = "undef") -> None:
        self.pos = ROBO_POS.copy() #np.array(ROBO_POS)
        self.old_pos = None
        self.collisions = 0
        self.walked_path = [np.array(self.pos),]
        self.time_steps = 0
        self.used_model = used_model
        self.current_movement = np.array((0,0))
        self.finished = False

    def goal_reached(self):
        return np.linalg.norm(self.pos - GOAL) < GOAL_RADIUS

    def move(self):
        if self.goal_reached():
            self.finished = True
            return

        pot_force_vektor = None
        if self.used_model == "classic_apf":
            pot_force_vektor = capf.potential_force(self.pos, GOAL, OBSTACLES)
        elif self.used_model == "forward_apf":
            pot_force_vektor = fapf.potential_force(self.pos, GOAL, OBSTACLES)
        elif self.used_model == "rotational_fapf":
            pot_force_vektor = rfapf.potential_force(self.pos, GOAL, OBSTACLES)
        elif self.used_model == "improved_apf":
            pot_force_vektor = iapf.potential_force(self.pos, GOAL, self.current_movement, OBSTACLES)
        else:
            raise RuntimeError('model for robot movement undefined!')
        
        pot_force_vektor_norm = np.linalg.norm(pot_force_vektor)
        if ROBOT_MAX_VELOCITY < pot_force_vektor_norm:
            pot_force_vektor = pot_force_vektor / pot_force_vektor_norm * ROBOT_MAX_VELOCITY

        # Regulierung/ Gewichtung:
        weight = 1 #0.7
        pot_force_vektor = pot_force_vektor * weight + self.current_movement * (1 - weight)

        self.current_movement = pot_force_vektor
        self.old_pos = self.pos.copy()
        self.pos += pot_force_vektor
        self.walked_path.append(self.pos.copy())
        self.time_steps += 1

#############################################################

class Velocity_Circle:
    """Klasse um einen bewegenden Kreis zubeschreiben."""
    name: str
    x: float
    y: float
    radius: float
    current_movement: np.array

    def __init__(self, x: float, y: float, radius: float, name:str):
        self.x = x
        self.y = y
        self.radius = radius
        self.name = name
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
    
    def check_collision_for_one_robot(self, obstacle_point1, obstacle_point2, distance_travelled_so_far, robot: Robot):
        # https://gamedev.stackexchange.com/questions/97337/detect-if-two-objects-are-going-to-collide
        global collision_points
        endpoint_check = (np.linalg.norm(robot.old_pos - obstacle_point1) < self.radius + ROBOT_CIRCLE_SIZE,
                np.linalg.norm(robot.old_pos - obstacle_point2) < self.radius + ROBOT_CIRCLE_SIZE,
                np.linalg.norm(robot.pos - obstacle_point1) < self.radius + ROBOT_CIRCLE_SIZE,
                np.linalg.norm(robot.pos - obstacle_point2) < self.radius + ROBOT_CIRCLE_SIZE)
        if any(endpoint_check):
            robot.collisions += 1
            if any(endpoint_check[0:1]):
                collision_points[robot.used_model].append(robot.old_pos.copy())
            else:
                collision_points[robot.used_model].append(robot.pos.copy())
            return
        movement_norm = np.linalg.norm(self.current_movement)
        distance_between_points = np.linalg.norm(obstacle_point1 - obstacle_point2)
        time_start = distance_travelled_so_far/movement_norm
        time_end = distance_between_points / movement_norm

        xa0, ya0 = obstacle_point1      # OBSTACLE_POS Old
        xat, yat = self.current_movement # OBSTACLE_MOVEMENT
        
        xb0, yb0 = robot.old_pos + robot.current_movement/np.linalg.norm(robot.current_movement) * time_start
        xbt,ybt = robot.current_movement

        mintime = -(xa0*xat - xat*xb0 - (xa0 - xb0)*xbt + ya0*yat - yat*yb0 - (ya0 - yb0)*ybt) /(xat**2 - 2*xat*xbt + xbt**2 + yat**2 - 2*yat*ybt + ybt**2)
        mindist = np.sqrt((mintime*xat - mintime*xbt + xa0 - xb0)**2 + (mintime*yat - mintime*ybt + ya0 - yb0)**2)

        if mindist < self.radius + ROBOT_CIRCLE_SIZE and 0 <= mintime <= time_end:  # collision detected
            robot.collisions += 1
            collision_points[robot.used_model].append(np.array((xb0, yb0)) + robot.current_movement * mintime)
    
    def check_collision(self, obstacle_point1, obstacle_point2, distance_travelled_so_far):
        for robot in ROBOTS:
            self.check_collision_for_one_robot(obstacle_point1, obstacle_point2, distance_travelled_so_far, robot)

    def move(self):
        """Bewegt das Hinderniss genau eine Bewegung weiter, Border werden berücksichtigt."""
        estimated_pos = x,y = self.get_position() + self.get_movement()

        distance_to_travel = np.linalg.norm(self.get_position() - estimated_pos)
        distance_traveled = 0
        last_checked_line = -1
        count = 0
        while x > CENTRAL_POSITION_OBSTACLES[0] + BOX_WIDTH or x < CENTRAL_POSITION_OBSTACLES[0] - BOX_WIDTH or y > CENTRAL_POSITION_OBSTACLES[1] + BOX_HEIGHT or y < CENTRAL_POSITION_OBSTACLES[1] - BOX_HEIGHT:
            line_between_positions = Segment(self.get_position(), estimated_pos)

            if (p := line_between_positions.intersection(line_bottom)):
                if last_checked_line != 1:
                    p = np.array(p[0]).astype(np.float32)
                    self.check_collision(self.get_position(), p, distance_traveled)
                    point_diff_to_border = np.linalg.norm(p - self.get_position())
                    distance_to_travel -= point_diff_to_border
                    distance_traveled += point_diff_to_border
                    self.current_movement[1] *= -1
                    estimated_pos = x,y = p + self.get_movement()/np.linalg.norm(self.get_movement()) * distance_to_travel
                    self.x, self.y = p
                    line_between_positions = Segment(self.get_position(), estimated_pos)
                    last_checked_line = 1
                    continue

            if (p := line_between_positions.intersection(line_left)):
                if last_checked_line != 2:
                    p = np.array(p[0]).astype(np.float32)
                    self.check_collision(self.get_position(), p, distance_traveled)
                    point_diff_to_border = np.linalg.norm(p - self.get_position())
                    distance_to_travel -= point_diff_to_border
                    distance_traveled += point_diff_to_border
                    self.current_movement[0] *= -1
                    estimated_pos = x,y = p + self.get_movement()/np.linalg.norm(self.get_movement()) * distance_to_travel
                    self.x, self.y = p   
                    line_between_positions = Segment(self.get_position(), estimated_pos)
                    last_checked_line = 2
                    continue

            if (p := line_between_positions.intersection(line_right)):
                if last_checked_line != 3:
                    p = np.array(p[0]).astype(np.float32)
                    self.check_collision(self.get_position(), p, distance_traveled)
                    point_diff_to_border = np.linalg.norm(p - self.get_position())
                    distance_to_travel -= point_diff_to_border
                    distance_traveled += point_diff_to_border
                    self.current_movement[0] *= -1
                    estimated_pos = x,y = p + self.get_movement()/np.linalg.norm(self.get_movement()) * distance_to_travel
                    self.x, self.y = p
                    line_between_positions = Segment(self.get_position(), estimated_pos)
                    last_checked_line = 3
                    continue

            if (p := line_between_positions.intersection(line_top)):
                if last_checked_line != 4:
                    p = np.array(p[0]).astype(np.float32)
                    self.check_collision(self.get_position(), p, distance_traveled)
                    point_diff_to_border = np.linalg.norm(p - self.get_position())
                    distance_to_travel -= point_diff_to_border
                    distance_traveled += point_diff_to_border
                    self.current_movement[1] *= -1
                    estimated_pos = x,y = p + self.get_movement()/np.linalg.norm(self.get_movement()) * distance_to_travel
                    self.x, self.y = p
                    line_between_positions = Segment(self.get_position(), estimated_pos)
                    last_checked_line = 4
                    continue

            
            count += 1
            if count > 10:
                print(f'------------{count}--------------')
                print(f'pos: {self.get_position()}')
                print(f'mov: {self.get_movement()}')
                print(f'est: {estimated_pos}')
                print(f'dtt: {distance_to_travel}')
                print('-----------------------------------')
                if count > 20:
                    exit()

        self.check_collision(self.get_position(), np.array((x,y)), distance_traveled)
        self.x, self.y = x, y
    
    def get_movement(self) -> np.array:
        """Gibt die geschätzte Bewegung zurück."""
        return self.current_movement
    
    def set_movement(self, new_movement: np.array):
        """Gibt die geschätzte Bewegung zurück."""
        self.current_movement = new_movement

    def get_shell_distance(self, point: tuple, shift:np.array = np.array((0,0))):
        global ROBOT_CIRCLE_SIZE
        return (np.sqrt((self.x + shift[0] - point[0]) ** 2 + (self.y + shift[1] - point[1]) ** 2) - (self.radius + ROBOT_CIRCLE_SIZE))
    
#############################################################

class ModelResults:
    used_model: str     # str on of 'classic_apf', 'forward_apf', 'rotational_fapf'
    collisions: list    # list of ints
    time_steps: list    # list of ints
    walked_paths: list  # list of points
    collision_points: list  # list of points

    def __init__(self ,model) -> None:
        self.used_model = model
        self.collisions: list = []
        self.time_steps: list = []
        self.walked_paths: list = []
        self.collision_points: list = []

    def add_collision(self, collision_count: int):
        self.collisions.append(collision_count)

    def add_time_steps(self, time_steps: int):
        self.time_steps.append(time_steps)

    def add_walked_path(self, path_points: list):
        self.walked_paths.append(path_points)

    def add_collision_points(self, collision_points: list):
        self.collision_points.append(collision_points)

class Results:
    capf_results: ModelResults
    fapf_results: ModelResults
    rfapf_results: ModelResults
    iapf_results: ModelResults
    obstacle_velocity: float
    robot_max_velocity: float
    obstacle_count: int
    test_count: int

    def __init__(self) -> None:
        self.capf_results = ModelResults('classic_apf')
        self.fapf_results = ModelResults('forward_apf')
        self.rfapf_results = ModelResults('rotational_fapf')
        self.iapf_results = ModelResults('improved_fapf')

        self.obstacle_velocity: float = None
        self.robot_max_velocity: float = None
        self.obstacle_count: int = None
        self.test_count: int = None

    def _add_results_to_model_results(self, result_object: ModelResults, robot: Robot):
        result_object.add_collision(robot.collisions)
        result_object.add_time_steps(robot.time_steps)
        result_object.add_walked_path(robot.walked_path)
        result_object.add_collision_points(collision_points[robot.used_model])

    def add_robot_results(self, robot: Robot):
        if not robot.finished:
            raise RuntimeError('Robot hasnt reached goal yet')
        if robot.used_model == 'classic_apf':
            self._add_results_to_model_results(self.capf_results, robot)
        elif robot.used_model == 'forward_apf':
            self._add_results_to_model_results(self.fapf_results, robot)
        elif robot.used_model == 'rotational_fapf':
            self._add_results_to_model_results(self.rfapf_results, robot)
        elif robot.used_model == 'improved_apf':
            self._add_results_to_model_results(self.iapf_results, robot)
        else:
            raise RuntimeError('Used Model for this robot not registered in Results!')
        
    def print_settings(self):
        return f'oCount{self.obstacle_count}_oVel{self.obstacle_velocity}_tCount{self.test_count}'
        
    def save(self, obstacle_count = None, test_count = None):
        file_name = f'Result_Objects/oCount{obstacle_count}_oVel{OBSTACLE_VELOCITY}_tCount{test_count}_{time.time_ns()}'
        self.obstacle_count = obstacle_count
        self.obstacle_velocity = OBSTACLE_VELOCITY
        self.robot_max_velocity = ROBOT_MAX_VELOCITY
        self.test_count = test_count
        file = open(file_name, 'wb')
        pickle.dump(obj=self, file=file)


#############################################################

def get_random_moving_obstacles(count: int) -> tuple:
    obstacles = ()
    for i in range(count):
        random_values_pos = np.random.rand(2) * 12 - 6
        random_values_movement = np.random.rand(2) - np.array((1,1))
        random_values_movement = random_values_movement / np.linalg.norm(random_values_movement) * OBSTACLE_VELOCITY

        # random_values_movement = np.array((0,1))

        obstacle_i = Velocity_Circle(*random_values_pos,CYLINDER_DEFAULT_RADIUS, f'obsatcle_{i}')
        obstacle_i.current_movement = random_values_movement
        obstacles = (*obstacles, obstacle_i)
        # print(random_values_movement)


    # test_circle = Velocity_Circle(4,4, CYLINDER_DEFAULT_RADIUS, 'test_kreis')
    # test_circle.current_movement = np.array((1,1)) * 10
    # test_circle = Velocity_Circle(-6,6, CYLINDER_DEFAULT_RADIUS, 'test_kreis')
    # test_circle.current_movement = np.array((1.00359855, 2.82715227)) * 1
    # obstacles = (test_circle,)

    return obstacles


robot_colors = {"classic_apf": 'cyan', "rotational_fapf": 'lime' ,"forward_apf": 'black', 'improved_apf': 'magenta'}  # Farbschema für die verschiedenen Modelle

def show_map_animation(update_intervall = 1000):
    # global  fig, ax, xdata, ydata, ln
    fig, ax = plt.subplots(figsize=(10, 10))
    ln, = ax.plot([], [], 'ro')

    def init():
        ax.set_xlim(-10 ,10)
        ax.set_ylim(-10, 10)
        return ln,

    def update(frame):
        if all([robot.finished for robot in ROBOTS]):
            ani.event_source.stop()
            plt.close()

        ax.cla()
        ax.set_xlim(-10 ,10)
        ax.set_ylim(-10, 10)

        update_all()

        for circle in OBSTACLES:
            ax.add_patch(plt.Circle(circle.get_position(), circle.radius, color="r"))

        for robot in ROBOTS:
            ax.add_patch(plt.Circle(robot.pos, ROBOT_CIRCLE_SIZE, color=robot_colors[robot.used_model], label = f'{robot.used_model}'))

        ax.add_patch(plt.Circle(GOAL, GOAL_RADIUS, color="yellow"))
        ax.add_patch(plt.Rectangle(CENTRAL_POSITION_OBSTACLES-BOX_WIDTH, BOX_WIDTH*2, BOX_HEIGHT*2, fill = False, color="blue"))

        plt.legend()
        
        return ln,

    ani = FuncAnimation(fig, update, init_func=init, interval = update_intervall, cache_frame_data=False)
    plt.show()



def plot_walked_path():
    # plt.figure()
    plt.title("Walked path")
    
    for robot in ROBOTS:
        x_points = ()
        y_points = ()
        for point in robot.walked_path:
            x_points = (*x_points, point[0])
            y_points = (*y_points, point[1])
            
        plt.plot(x_points, y_points, color = robot_colors[robot.used_model], label = f'{robot.used_model}, C:{robot.collisions}, t:{robot.time_steps}')
    
    plt.grid()
    plt.legend()
    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.show()


def print_obstacle_pos():
    for circle in OBSTACLES:
        print(circle.get_position())
        print(circle.get_movement())

#############################################################

def update_all():
    """Updatet die Positionen aller Figuren"""
    # move_robot()
    for robot in ROBOTS:
        robot.move()

    for circle in OBSTACLES:
        circle.move()


def simulate():
    count = 0
    try:
        while not all([robot.finished for robot in ROBOTS]):
            update_all()
            count += 1
    except KeyboardInterrupt:
        print('!!FINISH!!', count)


def list_robot_results():
    global ALL_RESULTS
    for robot in ROBOTS:
        ALL_RESULTS.add_robot_results(robot)


def init_lines_for_obstacle_movement():
    """Initialisiert die Line, damit die Bewegungsberechnungen der Objekte vereinfacht wird"""
    global line_left, line_bottom, line_right, line_top
    line_right = Segment(CENTRAL_POSITION_OBSTACLES + np.array((BOX_WIDTH, -BOX_HEIGHT)), CENTRAL_POSITION_OBSTACLES + np.array((BOX_WIDTH, BOX_HEIGHT)))
    line_left = Segment(CENTRAL_POSITION_OBSTACLES + np.array((-BOX_WIDTH, -BOX_HEIGHT)), CENTRAL_POSITION_OBSTACLES + np.array((-BOX_WIDTH, BOX_HEIGHT)))
    line_top = Segment(CENTRAL_POSITION_OBSTACLES + np.array((BOX_WIDTH, BOX_HEIGHT)), CENTRAL_POSITION_OBSTACLES + np.array((-BOX_WIDTH, BOX_HEIGHT)))
    line_bottom = Segment(CENTRAL_POSITION_OBSTACLES + np.array((BOX_WIDTH, -BOX_HEIGHT)), CENTRAL_POSITION_OBSTACLES + np.array((-BOX_WIDTH, -BOX_HEIGHT)))

def init_parameters():
    ############## SIMULATION PARAMETERS ##############
    # Enviroment settings:
    global CENTRAL_POSITION_OBSTACLES, BOX_WIDTH, BOX_HEIGHT, CYLINDER_DEFAULT_RADIUS, OBSTACLE_FUTURE_ADJUSTMENT, GOAL, GOAL_RADIUS
    CENTRAL_POSITION_OBSTACLES = np.array((0,0))
    BOX_WIDTH = 6   # its the radius, not the actual width or height
    BOX_HEIGHT = 6
    CYLINDER_DEFAULT_RADIUS = 0.2
    OBSTACLE_FUTURE_ADJUSTMENT = 0.8    # responsible for decreasing projection force 
    # OBSTACLE_VELOCITY = 3 #.03
    GOAL = np.array((10, 0.))
    GOAL_RADIUS = 1

    # ROBOT:
    global ROBO_POS, ROBOT_CIRCLE_SIZE, ROBOT_MAX_VELOCITY
    ROBO_POS = np.array((-10,0.))    # START-POSITION
    ROBOT_CIRCLE_SIZE = 0.1

    global ROBOTS
    # ROBOTS = (Robot("classic_apf"), Robot("forward_apf"), Robot("rotational_fapf"), Robot("improved_apf"))
    # ROBOTS = (Robot("classic_apf"),)
    # ROBOTS = (Robot("forward_apf"),)
    # ROBOTS = (Robot("rotational_fapf"),)
    ROBOTS = (Robot("improved_apf"),)


    global collision_points
    collision_points = {'classic_apf': [], 'forward_apf': [], 'rotational_fapf': [], 'improved_apf': []}


def run_test(obstacle_count = 5, test_count = 1, with_animation = False, show_routes = False, save_results = False):
    global OBSTACLES, ALL_RESULTS

    ALL_RESULTS = Results()

    init_parameters()
    init_lines_for_obstacle_movement()

    for i in range(test_count):
        init_parameters()
        OBSTACLES = get_random_moving_obstacles(obstacle_count)
        if with_animation:
            show_map_animation(update_intervall=100)
        else:
            simulate()
        if show_routes:    
            plot_walked_path()
        if save_results:
            list_robot_results()

        print(f'Run {i} completed!')

    # Ergebnisse ggf serialisieren        
    if save_results:
        ALL_RESULTS.save(obstacle_count, test_count)


if __name__ == '__main__':

    if len(sys.argv) <= 1:
        print('Soll mit Standardeinstellungen fortgefahren werden?')
        eingabe = input('Y/n: ')
        if eingabe == '' or eingabe.lower() == 'y':
            test_count = 10
            obstacle_count = 5
            OBSTACLE_VELOCITY = 1#2
            ROBOT_MAX_VELOCITY = .5 #1# .01
            with_animation = True
            show_routes = True
            save_results = False 
        else: 
            print('Beenden...')
            exit()
    else:
        try:
            test_count = int(sys.argv[1])
            obstacle_count = int(sys.argv[2])
            OBSTACLE_VELOCITY = float(sys.argv[3])  
            ROBOT_MAX_VELOCITY = float(sys.argv[4])  
            with_animation = False
            show_routes = False
            save_results = True 
            if test_count<=0:
                raise ValueError('test_count ist ungültig!')
            if ROBOT_MAX_VELOCITY<=0:
                raise ValueError('ROBOT_MAX_VELOCITY ist ungültig!')
        except:
            print('Fehlerhafte Eingabe:')
            print('1. Argument: test_count')
            print('2. Argument: obstacle_count')
            print('3. Argument: OBSTACLE_VELOCITY')
            print('4. Argument: ROBOT_MAX_VELOCITY')
            exit()

    run_test(obstacle_count, test_count, with_animation = with_animation, show_routes = show_routes, save_results = save_results)
    # run_test(obstacle_count, test_count, with_animation = True, show_routes = True, save_results = False)
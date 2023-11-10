from potential_field import Circle, plotten_3D_2

import numpy as np
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits import mplot3d
from scipy.interpolate import griddata

class Velocity_Circle(Circle):
    """Klasse um einen Kreis zubeschreiben, welcher ebenfalls eine räumliche Bewegung hat."""
    velocity: np.array

    def __init__(self, x: float, y: float, radius: float, velocity: tuple = (0,0)):
        Circle.__init__(self, x, y, radius)
        self.velocity = np.array(velocity)

    def get_velocity(self):
        """Gibt die aktuelle Geschwindigkeit des Kreises zurück."""
        return self.velocity
    
    def change_velocity(self, new_velocity):
        """Ändert die Geschwindigkeit des Kreises."""
        self.velocity = new_velocity

def orthogonal_zerlegung_a_längs_b(a:np.array, b:np.array):
    return a - np.dot(b,a)/np.dot(b,b) * b


def potential_field(point, velocity):
    return attraction_field(point) + repulsive_field(point, velocity)  

def attraction_field(point):
    return 1/2 * SCALING_FACTOR_ATTRACTION_FORCE * np.dot(
        GOAL - np.array(point), GOAL - np.array(point)
    )


def potential_force(point, velocity):
    return attraction_force(point) + repulsive_force(point, velocity)

def attraction_force(point):
    return SCALING_FACTOR_ATTRACTION_FORCE * (GOAL - np.array(point))


def repulsive_field(point, velocity):
    rep_field = 0
    for circle in OBSTACLES:
        distance = circle.get_shell_distance_point(point)
        e_ao = orthogonal_zerlegung_a_längs_b(np.array(velocity) - circle.get_velocity(), np.array(point) - circle.get_position())
        v_ao = np.dot((np.array(velocity) - circle.get_velocity()), e_ao)
        # v_ao = np.dot((np.array(velocity) - circle.get_velocity()), (np.array(point) - circle.get_position()))
        if distance < MIN_DISTANCE_REPULSIVE_FORCE and v_ao >= 0:
            distance_to_GOAL = np.linalg.norm(np.array(point) - GOAL)
            U_X = 1/2 * SCALING_FACTOR_REPULSIVE_FORCE * (1/distance - 1/MIN_DISTANCE_REPULSIVE_FORCE)**2 * distance_to_GOAL**REPULSIVE_FORCE_N
            U_V = SCALING_FACTOR_REPULSIVE_FORCE_VELOCITY * v_ao / distance
            rep_field += U_X + U_V
    return rep_field

def repulsive_force(point, velocity):
    rep_force = 0
    for circle in OBSTACLES:
        distance = circle.get_shell_distance_point(point)
        e_ao = orthogonal_zerlegung_a_längs_b(np.array(velocity) - circle.get_velocity(), np.array(point) - circle.get_position())
        v_ao = np.dot((np.array(velocity) - circle.get_velocity()), e_ao)
        # v_ao = np.dot((np.array(velocity) - circle.get_velocity()), (np.array(point) - circle.get_position()))

        if distance < MIN_DISTANCE_REPULSIVE_FORCE and v_ao >= 0:
            distance_to_GOAL = np.linalg.norm(np.array(point) - GOAL)
            F_X_1 = SCALING_FACTOR_REPULSIVE_FORCE * (1/distance - 1/MIN_DISTANCE_REPULSIVE_FORCE) * distance_to_GOAL**REPULSIVE_FORCE_N / distance**2
            F_X_2 = REPULSIVE_FORCE_N / 2 * SCALING_FACTOR_REPULSIVE_FORCE * (1/distance - 1/MIN_DISTANCE_REPULSIVE_FORCE)**2 * distance_to_GOAL**(REPULSIVE_FORCE_N-1)
            F_X = F_X_1 + F_X_2
            F_V = - SCALING_FACTOR_REPULSIVE_FORCE_VELOCITY * v_ao / distance
            rep_force += F_X + F_V
    return rep_force


def show_field(field_func, start_point: tuple, end_point: tuple, increment: float, min_dist_to_obstacles: float = .1):
    x = start_point[0]
    y = start_point[1]
    field = ()
    while x <= end_point[0]:
        y = start_point[1]
        while y <= end_point[1]:
            point_in_obstacle = False
            for circle in OBSTACLES:
                    if circle.get_shell_distance_point((x,y)) <= min_dist_to_obstacles:
                        point_in_obstacle = True
            if not point_in_obstacle:
                field = (*field, (x, y, field_func((x, y))))
            y += increment
        x += increment
    plotten_3D_2(field)


def show_map(with_force = True, with_path=False, path_steps=-1):
    # fig, ax = plt.subplots()
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot()
    for circle in OBSTACLES:
        ax.add_patch(plt.Circle(circle.get_position(), circle.radius, color="r"))
    plt.scatter(*GOAL)

    if with_force:
        x = 0
        y = 0
        increment = .5
        while x < 12:
            y = 0
            while y < 12:
                draw = True
                for circle in OBSTACLES:
                    if circle.get_shell_distance_point((x,y)) <= 0:
                        draw = False
                
                if draw:
                    plt.quiver(x, y, *potential_force((x,y), (0,0)), color='b', units='xy', scale=1)

                # plt.quiver(x, y, *potential_force((x,y)), color='b', units='xy', scale=20)
                y += increment
            x += increment

    if with_path:
        current_pos = START
        step_counter = 0
        current_robot_velocity = np.array((0,0))
        while(np.linalg.norm(current_pos - GOAL) > 0.1 and step_counter < path_steps):
            pot_force = potential_force(current_pos, current_robot_velocity)
            current_robot_velocity = pot_force
            plt.quiver(current_pos[0], current_pos[1], *pot_force, color='r', units='xy', scale=1)
            current_pos += pot_force
            step_counter += 1


    plt.xlim(0, 12)
    plt.ylim(0, 12)
    plt.show()



def show_gnron_solution():
    print('show_gnron_solution')
    global OBSTACLES, REPULSIVE_FORCE_N

    OBSTACLES = (*OBSTACLES, Velocity_Circle(9,10,0.5))
    show_map(with_force=False)

    REPULSIVE_FORCE_N = 0
    test = lambda point: potential_field(point, (0,0))
    show_field(test,start_point=(0, 0), end_point=(15, 15), increment=.1)

    REPULSIVE_FORCE_N = 1
    test = lambda point: potential_field(point, (0,0))
    show_field(test,start_point=(0, 0), end_point=(15, 15), increment=.1)

    REPULSIVE_FORCE_N = 2
    test = lambda point: potential_field(point, (0,0))
    show_field(test,start_point=(0, 0), end_point=(15, 15), increment=.1)


def show_dynamic_solution():
    print('show_dynamic_solution')
    show_map(with_force=False, with_path=True, path_steps=10)



if __name__ == "__main__":
    START = np.array((0.,0.))
    GOAL = np.array((10, 10))
    OBSTACLES = (Velocity_Circle(4, 7, 3,3), Velocity_Circle(9, 2.5, 2), Velocity_Circle(10,8,1))

    SCALING_FACTOR_ATTRACTION_FORCE = 1/20
    SCALING_FACTOR_REPULSIVE_FORCE = 3/20
    SCALING_FACTOR_REPULSIVE_FORCE_VELOCITY = 3/20
    MIN_DISTANCE_REPULSIVE_FORCE = 1

    REPULSIVE_FORCE_N = 1

    show_gnron_solution()

    show_dynamic_solution()
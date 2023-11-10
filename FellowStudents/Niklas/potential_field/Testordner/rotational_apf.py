import numpy as np
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits import mplot3d
from scipy.interpolate import griddata

CLOCKWISE_MATRIX = np.array(((0,1),(-1,0)))
COUNTERCLOCKWISE_MATRIX = np.array(((0,-1),(1,0)))


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
            np.sqrt((self.x - point[0]) ** 2 + (self.y - point[1]) ** 2) - self.radius
        )

    def get_shell_distance_circle(self, other_circle):
        return np.linalg.norm(self.get_position - other_circle.get_position()) - (
            self.radius + other_circle.radius
        )


def attraction_force(point):
    return SCALING_FACTOR_ATTRACTION_FORCE * (GOAL - np.array(point))

def repulsive_force(point):
    rep_force = 0
    for circle in OBSTACLES:
        # distance = circle.get_shell_distance_point(point)
        distance = circle.get_simple_distance_point(point)
        if distance < MIN_DISTANCE_REPULSIVE_FORCE:
            temp = (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - circle.get_position()) / np.linalg.norm(np.array(point) - circle.get_position())**3
            rep_force_normal = SCALING_FACTOR_REPULSIVE_FORCE * temp
            rep_force_with_rotation = get_rotational_repulsive_force(temp, CURRENT_TEST_DIRECTION, circle.get_position())
            rep_force += rep_force_normal + rep_force_with_rotation
    return rep_force


def get_rotational_repulsive_force(temp_term: np.array, current_direction: np.array, obstacle_position: np.array):
    direction_angle = np.arctan2(current_direction[1], current_direction[0])
    obstacle_angle = np.arctan2(obstacle_position[1], obstacle_position[0])
    rotation_matrix = CLOCKWISE_MATRIX if direction_angle - obstacle_angle >= 0 else COUNTERCLOCKWISE_MATRIX 
    return SCALING_FACTOR_REPULSIVE_ROTATION_FORCE * np.dot(rotation_matrix, temp_term)

def potential_force(point):
    return attraction_force(point) + repulsive_force(point)


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
                    # plt.quiver(x, y, *potential_force((x,y)), color='b', units='xy', scale=1)


                    if type(temp:=repulsive_force((x,y))) != int:
                        plt.quiver(x, y, *temp, color='b', units='xy', scale=1)

                y += increment
            x += increment

    if with_path:
        current_pos = START
        # while(current_pos[0], current_pos[1] != GOAL[0], GOAL[1]):
        step_counter = 0
        while(np.linalg.norm(current_pos - GOAL) > 0.1 and step_counter < path_steps):
            pot_force = potential_force(current_pos)
            plt.quiver(current_pos[0], current_pos[1], *pot_force, color='r', units='xy', scale=1)
            current_pos += pot_force
            step_counter += 1


    plt.xlim(0, 12)
    plt.ylim(0, 12)
    plt.show()


if __name__ == "__main__":
    START = np.array((4.5,0.))
    GOAL = np.array((15, 15))
    # OBSTACLES = (Circle(4.5, 7.5, 1), Circle(9, 2.5, 0.6), Circle(10,8,1))
    OBSTACLES = (Circle(6, 6, 1.3), )

    SCALING_FACTOR_ATTRACTION_FORCE = 1/20
    SCALING_FACTOR_REPULSIVE_FORCE = 5
    SCALING_FACTOR_REPULSIVE_ROTATION_FORCE = 15

    MIN_DISTANCE_REPULSIVE_FORCE = 5

    CURRENT_TEST_DIRECTION = GOAL

    show_map(with_force=True, with_path=True, path_steps=100)


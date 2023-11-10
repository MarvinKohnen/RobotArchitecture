import numpy as np
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits import mplot3d
from scipy.interpolate import griddata


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


def plotten_3D(data):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(projection="3d")
    ax.set_box_aspect(aspect=(1, 1, 2))
    plt.title("Potential Field")

    for x, y, z in data:
        ax.scatter(x, y, z, c=z, cmap="viridis")

    # ax.plot(data)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("force")
    plt.show()

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


def plotten_3D_surface(func):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection="3d")
    x = y = np.arange(0, 20, .1)

    X, Y = np.meshgrid(x, y)

    zs = []
    for point in zip(np.ravel(X), np.ravel(Y)):
        zs.append(func(point))
    Z = np.array(zs).reshape(X.shape)

    ax.plot_surface(X, Y, Z, cmap="viridis", antialiased=True)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("force")

    plt.show()


def attraction_field(point):
    # return 1/2 * SCALING_FACTOR_ATTRACTION_FORCE * np.linalg.norm(GOAL - np.array(point))**2
    return 1/2 * SCALING_FACTOR_ATTRACTION_FORCE * np.dot(
        GOAL - np.array(point), GOAL - np.array(point)
    )

def repulsive_field(point):
    rep_field = 0
    for circle in OBSTACLES:
        distance = circle.get_shell_distance_point(point)
        # if distance <= 0:
        #     continue
        if distance < MIN_DISTANCE_REPULSIVE_FORCE:
            rep_field += 1/2 * SCALING_FACTOR_REPULSIVE_FORCE * (1/distance - 1/MIN_DISTANCE_REPULSIVE_FORCE)**2
    return rep_field


def potential_field(point):
    return attraction_field(point) + repulsive_field(point)


def attraction_force(point):
    # return 1/2 * SCALING_FACTOR_ATTRACTION_FORCE * np.linalg.norm(GOAL - np.array(point))**2
    # return SCALING_FACTOR_ATTRACTION_FORCE * np.linalg.norm(GOAL - np.array(point))
    return SCALING_FACTOR_ATTRACTION_FORCE * (GOAL - np.array(point))

# def repulsive_force2(point):
#     rep_force = 0
#     for circle in OBSTACLES:
#         distance = circle.get_shell_distance_point(point)
#         if distance < MIN_DISTANCE_REPULSIVE_FORCE:
#             rep_force += SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (1/distance**2) * (np.array(point) - circle.get_position()) / np.linalg.norm(np.array(point) - circle.get_position())
#     return rep_force

def repulsive_force(point):
    rep_force = 0
    for circle in OBSTACLES:
        distance = circle.get_shell_distance_point(point)
        if distance < MIN_DISTANCE_REPULSIVE_FORCE:
            rep_force += SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - circle.get_position()) / np.linalg.norm(np.array(point) - circle.get_position())
    return rep_force

def potential_force(point):
    return attraction_force(point) + repulsive_force(point)

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
                    plt.quiver(x, y, *potential_force((x,y)), color='b', units='xy', scale=1)

                # plt.quiver(x, y, *potential_force((x,y)), color='b', units='xy', scale=20)
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



def probleme_aufzeigen():
    """Methode welche die Probleme des klassischen APF-Algorithmus aufzeigen soll."""
    global START, GOAL, OBSTACLES

    # lokales Minimum
    OBSTACLES = (Circle(5, 7, 1.5),Circle(7, 5, 1.5))
    show_map(with_force=True)
    OBSTACLES = (Circle(5, 5, 2),)
    show_map(with_force=True)
    show_map(with_force=False, with_path=True, path_steps=200)


    # goal nonreachable with obstacles nearby (GNRON)
    OBSTACLES = (Circle(9, 10, 0.5),)
    show_map(with_force=True)
    show_map(with_force=False, with_path=True, path_steps=200)

if __name__ == "__main__":
    START = np.array((0.,0.))
    GOAL = np.array((10, 10))
    OBSTACLES = (Circle(4, 7, 3), Circle(9, 2.5, 2), Circle(10,8,1))

    SCALING_FACTOR_ATTRACTION_FORCE = 1/20
    SCALING_FACTOR_REPULSIVE_FORCE = 3/20
    MIN_DISTANCE_REPULSIVE_FORCE = 1


    ######
    show_map(with_force=False)
    
    plotten_3D_surface(attraction_field)
    plotten_3D_surface(repulsive_field)
    plotten_3D_surface(potential_field)
    show_field(repulsive_field,start_point=(0, 0), end_point=(15, 15), increment=.1)
    show_field(potential_field,start_point=(0, 0), end_point=(15, 15), increment=.1)

    show_map(with_force=True)
    show_map(with_force=False, with_path=True, path_steps=100)


    ######
    probleme_aufzeigen()
import DifferentModels.classic_apf as capf
import DifferentModels.forward_apf as fapf
import DifferentModels.rotational_fapf as rfapf
import DifferentModels.improved_apf as iapf

from simulation import Velocity_Circle, init_parameters
import simulation as sim

#####################################################
import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
import itertools
import matplotlib.colors as mcolors

plt.rc('font', size=15) 

def get_random_moving_obstacles(count: int) -> tuple:
    obstacles = ()
    for i in range(count):
        random_values_pos = np.random.rand(2) * 12 - 6
        random_values_movement = np.random.rand(2) - np.array((1,1))
        random_values_movement = random_values_movement / np.linalg.norm(random_values_movement) * 2

        # random_values_movement = np.array((0,1))

        obstacle_i = Velocity_Circle(*random_values_pos,CYLINDER_DEFAULT_RADIUS, f'obstacle_{i}')
        obstacle_i.current_movement = random_values_movement
        obstacles = (*obstacles, obstacle_i)
        print(random_values_movement)


    # test_circle = Velocity_Circle(-1, -1, CYLINDER_DEFAULT_RADIUS, 'test_kreis')
    # # test_circle.current_movement = np.array((2,1))
    # test_circle.current_movement = np.array((1,1))
    # obstacles = (test_circle,)

    test_circle = Velocity_Circle(1, 1, CYLINDER_DEFAULT_RADIUS, 'test_kreis')
    test_circle.current_movement = np.array((2,1))
    # test_circle.current_movement = np.array((-1,-1))
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

    test_map_multiple_paths(path_count=10, max_vector_norm=ROBOT_MAX_VELOCITY, path_steps=500)


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
                    # vector = repulsive_force(np.array((x,y)), np.array((1,0)), OBSTACLES) + np.array((0,0))
                    # vector = repulsive_force(np.array((x,y)), GOAL - np.array((x,y)), OBSTACLES) + np.array((0,0))

                    vector = potential_force(np.array((x,y)), GOAL, OBSTACLES)

                    # if max_vector_norm is None or np.linalg.norm(vector) < max_vector_norm:
                    #     if vector[0] != 0 and vector[1] != 0: 

                    #         plt.quiver(x, y, *vector, color='black', units='xy', scale=1)

                    if max_vector_norm is None and max_vector_norm != 0:
                        plt.quiver(x, y, *vector, color='blue', units='xy', scale=1)
                    else:
                        pot_force_vektor_norm = np.linalg.norm(vector)
                        if ROBOT_MAX_VELOCITY < pot_force_vektor_norm:
                            vector = vector / pot_force_vektor_norm * ROBOT_MAX_VELOCITY
                        plt.quiver(x, y, *vector, color='blue', units='xy', scale=1)

                # plt.quiver(x, y, *potential_force((x,y)), color='b', units='xy', scale=20)
                y += increment
            x += increment

    if with_path:
        current_pos = np.array((-10,1))
        step_counter = 0
        while(np.linalg.norm(current_pos - GOAL) > 0.1 and step_counter < path_steps):
            pot_force = repulsive_force((x,y), current_orientation_robot=np.array((x,y))) + np.array((0,0))
            # pot_force = potential_force(np.array((x,y)), GOAL, OBSTACLES)
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
                    vector = potential_force(np.array((x,y)), GOAL, OBSTACLES)

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
            pot_force = potential_force(np.array((x,y)), GOAL, OBSTACLES)
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


def draw_Map_on_axs(ax: plt.Axes, potential_force, max_vector_norm: float):
    for circle in OBSTACLES:
        ax.add_patch(plt.Circle(circle.get_position(), circle.radius, color="r"))
        ax.quiver(*circle.get_position(), *circle.current_movement*3, color='g', units='xy', scale=1)
    # plt.scatter(*GOAL)
    ax.add_patch(plt.Circle(GOAL, GOAL_RADIUS, color="yellow"))
    ax.add_patch(plt.Rectangle(CENTRAL_POSITION_OBSTACLES-BOX_WIDTH, BOX_WIDTH*2, BOX_HEIGHT*2, fill = False, color="blue"))

    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)
    ax.set_title(potential_force.__module__)

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
                vector = potential_force(np.array((x,y)), GOAL, OBSTACLES)

                # if max_vector_norm is None or np.linalg.norm(vector) < max_vector_norm:
                #     if vector[0] != 0 and vector[1] != 0: 

                #         plt.quiver(x, y, *vector, color='black', units='xy', scale=1)

                if max_vector_norm is None and max_vector_norm != 0:
                    ax.quiver(x, y, *vector, color='black', units='xy', scale=1)
                else:
                    pot_force_vektor_norm = np.linalg.norm(vector)
                    if ROBOT_MAX_VELOCITY < pot_force_vektor_norm:
                        vector = vector / pot_force_vektor_norm * ROBOT_MAX_VELOCITY
                    ax.quiver(x, y, *vector, color='black', units='xy', scale=1)

            # plt.quiver(x, y, *potential_force((x,y)), color='b', units='xy', scale=20)
            y += increment
        x += increment

def show_map_side_by_side():
    fig, axs = plt.subplots(2,2, figsize=(10, 10))
    draw_Map_on_axs(axs[0,0], capf.potential_force, ROBOT_MAX_VELOCITY)
    draw_Map_on_axs(axs[0,1], fapf.potential_force, ROBOT_MAX_VELOCITY)
    draw_Map_on_axs(axs[1,0], rfapf.potential_force, ROBOT_MAX_VELOCITY)
    plt.show()


def show_map_one_by_one():
    potential_force = capf.potential_force
    show_map(max_vector_norm=ROBOT_MAX_VELOCITY)

    potential_force = fapf.potential_force
    show_map(max_vector_norm=ROBOT_MAX_VELOCITY)

    potential_force = rfapf.potential_force
    show_map(max_vector_norm=ROBOT_MAX_VELOCITY)


if __name__ == '__main__':

    init_parameters()

    CYLINDER_DEFAULT_RADIUS, GOAL, ROBOT_CIRCLE_SIZE = sim.CYLINDER_DEFAULT_RADIUS, sim.GOAL, sim.ROBOT_CIRCLE_SIZE
    ROBOT_MAX_VELOCITY = 1
    

    CENTRAL_POSITION_OBSTACLES = np.array((0,0))
    BOX_WIDTH = 6   # its the radius, not the actual width or height
    BOX_HEIGHT = 6
    GOAL_RADIUS = 1

    OBSTACLES = get_random_moving_obstacles(5)

    #####################################################

    # show_map_one_by_one
    # show_map_side_by_side()


    #####################################################

    # potential_force = fapf.potential_force
    # show_map(max_vector_norm=ROBOT_MAX_VELOCITY)

    potential_force = rfapf.potential_force
    repulsive_force = rfapf.repulsive_force
    show_map(max_vector_norm=ROBOT_MAX_VELOCITY)

    # potential_force = rfapf.potential_force
    # show_map(max_vector_norm=ROBOT_MAX_VELOCITY)


    # repulsive_force = iapf.sum_of_repulsive_forces
    # show_map(max_vector_norm=ROBOT_MAX_VELOCITY)
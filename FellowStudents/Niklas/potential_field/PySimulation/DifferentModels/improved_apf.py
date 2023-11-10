## based on https://www.hindawi.com/journals/mpe/2020/6523158/
# https://www.researchgate.net/publication/340959999_Improved_Artificial_Potential_Field_Method_Applied_for_AUV_Path_Planning
import numpy as np

SCALING_FACTOR_ATTRACTION_FORCE = 1/20
SCALING_FACTOR_REPULSIVE_FORCE = 5
SCALING_FACTOR_VELOCITY_REPULSIVE_FORCE = 1
MIN_DISTANCE_REPULSIVE_FORCE = 3

def potential_force(point, GOAL, last_movement, OBSTACLES):
    ############ testen ##############
    # last_movement = attraction_force(point, GOAL)
    # if np.linalg.norm(last_movement) >= .1:
    #     last_movement = last_movement/np.linalg.norm(last_movement)
    ##################################

    return attraction_force(point, GOAL) + sum_of_repulsive_forces(point, last_movement, OBSTACLES)

def attraction_force(point, GOAL):
    return SCALING_FACTOR_ATTRACTION_FORCE * (GOAL - np.array(point))

def sum_of_repulsive_forces(point, last_movement, OBSTACLES):
    rep_force = 0
    for circle in OBSTACLES:
        rep_force += repulsive_force(point, last_movement, circle)
    return rep_force

def repulsive_force(point, last_movement, circle):
    distance = circle.get_shell_distance(point)
    v_ao = get_componand_of_a_along_b(last_movement - circle.get_movement(), circle.get_position() - point)
    # print(v_ao)
    # e_ao = np.array((1,0))
    # v_ao = get_componand_of_a_along_b(last_movement - circle.get_movement(), e_ao)

    rep_force = 0
    if distance <= MIN_DISTANCE_REPULSIVE_FORCE and v_ao >= 0:
        # rep_force = repulsive_force_position(point, circle, distance) + repulsive_force_velocity(distance, v_ao)
        rep_force = repulsive_force_position(point, circle, distance) + repulsive_force_velocity2(distance, v_ao, circle.get_position() - point)
    return rep_force

def repulsive_force_position(point, circle, distance):
    rep_force = SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - circle.get_position()) / np.linalg.norm(np.array(point) - circle.get_position())**3
    return rep_force

def repulsive_force_velocity(distance: float, v_ao: float):
    return -SCALING_FACTOR_VELOCITY_REPULSIVE_FORCE * v_ao / distance

def repulsive_force_velocity2(distance: float, v_ao: float, vector_to_obstacle: np.array):
    return -SCALING_FACTOR_VELOCITY_REPULSIVE_FORCE * ((vector_to_obstacle / distance) * v_ao) / distance

def get_componand_of_a_along_b(a: np.array, b: np.array):
    return a.dot(b)/ np.linalg.norm(b)



if __name__=='__main__':
    ## zum testen:
    v = np.array((1,2))
    vo = np.array((2,2))
    abstands_vector = np.array((1,0))

    print(get_componand_of_a_along_b(v-vo,abstands_vector))
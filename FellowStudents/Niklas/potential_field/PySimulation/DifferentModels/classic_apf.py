import numpy as np

# SCALING_FACTOR_ATTRACTION_FORCE = 1/20
# SCALING_FACTOR_REPULSIVE_FORCE = 3/20
# MIN_DISTANCE_REPULSIVE_FORCE = 1

SCALING_FACTOR_ATTRACTION_FORCE = 1/20 #.5# 1/20
SCALING_FACTOR_REPULSIVE_FORCE = 5 #13/20 # 10# 13/20 #7/20
MIN_DISTANCE_REPULSIVE_FORCE = 3

#############################################################

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
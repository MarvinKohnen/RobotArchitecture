import numpy as np


SCALING_FACTOR_ATTRACTION_FORCE = 1/20
SCALING_FACTOR_REPULSIVE_FORCE = 5
# SCALING_FACTOR_REPULSIVE_ROTATION_FORCE = SCALING_FACTOR_REPULSIVE_FORCE * 1.5
MIN_DISTANCE_REPULSIVE_FORCE = 3
REPULSIVE_FORCE_FUTURE_COUNT = 3
OBSTACLE_FUTURE_ADJUSTMENT = 0.8

CLOCKWISE_MATRIX = np.array(((0,1),(-1,0)))
# COUNTERCLOCKWISE_MATRIX = np.array(((0,-1),(1,0)))

ROBOT_CIRCLE_SIZE = 0.1

REPULSIVE_FORCE_ROTATION = np.deg2rad(45)   # <---------------

def potential_force(point: np.array, GOAL: np.array, OBSTACLES):
    # return attraction_force(point) + repulsive_force(point, current_orientation_robot)

    force1 = attraction_force(point, GOAL) 
    force2 = repulsive_force(point, OBSTACLES, GOAL)
    # print('at_force:',force1, 'rep_force: ', force2)
    return force1 + force2



def attraction_force(point, GOAL):
    return SCALING_FACTOR_ATTRACTION_FORCE * (GOAL - np.array(point))


def repulsive_force(point: np.array, OBSTACLES, GOAL):
    rep_force = 0
    for circle in OBSTACLES:
        rep_force += movement_force3(point, circle, GOAL)
    return rep_force

# def movement_force2(point: np.array, obstacle, GOAL):
#     current_orientation_robot = GOAL - point

#     movement = obstacle.get_movement() #/ MOVEMENT_REPULSIVE_FORCE_COUNT

#     # Falls das Objekt sich nicht bewegt -> normale Berechnung:
#     if movement[0] == 0 and movement[1] == 0:# or obstacle.get_shell_distance(point) > np.linalg.norm(movement * REPULSIVE_FORCE_FUTURE_COUNT) + MIN_DISTANCE_REPULSIVE_FORCE:  # um Performanc zusparen ggf
#         distance = obstacle.get_shell_distance(point)
#         if distance < MIN_DISTANCE_REPULSIVE_FORCE:
#             position_with_shift = obstacle.get_position()

#             temp = (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
#             rep_force_normal = SCALING_FACTOR_REPULSIVE_FORCE * temp
#             rep_force_with_rotation = get_rotational_repulsive_force(temp, current_orientation_robot)
#             return rep_force_normal + rep_force_with_rotation
#         else: return 0

#     # in 3 Sektoren aufteilen:
#     # - hinter dem richtigem Hinderniss
#     # - neben den Projezierten Hindernisse
#     # - vor dem am weitesten in der Zukunft liegende Hinderniss

#     movement_field_vector = 0
#     point_in_rectangle = False

#     # Rectangle check:  https://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle
#     normalized_movement = movement / np.linalg.norm(movement)
#     normalized_orthogonal_movement = np.dot(np.array(((0,-1),(1,0))), normalized_movement)
#     rectangle_point_1 = obstacle.get_position() + normalized_orthogonal_movement * MIN_DISTANCE_REPULSIVE_FORCE
#     AM = point - rectangle_point_1
#     AB = movement * REPULSIVE_FORCE_FUTURE_COUNT
#     AD = - normalized_orthogonal_movement * MIN_DISTANCE_REPULSIVE_FORCE * 2
#     if 0 < np.dot(AM, AB) < np.dot(AB, AB) and 0 < np.dot(AM, AD) < np.dot(AD, AD):
#         # Falls der Punkt nun in diesem rechteck ist -> orthogonal ABstand betrachten:
#         # orth_distance = abs(np.dot(normalized_orthogonal_movement, point)) #- (obstacle.radius + ROBOT_CIRCLE_SIZE)
#         # position_with_shift = point - (np.dot(normalized_orthogonal_movement, point) ) * normalized_orthogonal_movement

#         point_on_middle_line = obstacle.get_position()
#         vektor_between_line_and_point = (point - point_on_middle_line) - np.dot(np.dot(point - point_on_middle_line, normalized_movement), normalized_movement)
#         distance_adjustment_scaling = (np.linalg.norm(-vektor_between_line_and_point + point - point_on_middle_line) / np.linalg.norm(movement * REPULSIVE_FORCE_FUTURE_COUNT))
        
#         orth_distance = np.linalg.norm(vektor_between_line_and_point) + OBSTACLE_FUTURE_ADJUSTMENT *distance_adjustment_scaling - (obstacle.radius + ROBOT_CIRCLE_SIZE)

#         sign = -1 if np.dot(-vektor_between_line_and_point, normalized_orthogonal_movement) < 0 else 1
#         position_with_shift = sign * normalized_orthogonal_movement * orth_distance + point

#         temp = (1 / orth_distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / orth_distance**3
#         rep_force_normal = SCALING_FACTOR_REPULSIVE_FORCE * temp
#         rep_force_with_rotation = get_rotational_repulsive_force(temp, current_orientation_robot)
#         movement_field_vector += rep_force_normal + rep_force_with_rotation
#         point_in_rectangle = True
        
#     distance_to_obstacle = obstacle.get_shell_distance(point)
#     distance_to_future_obstacle = obstacle.get_shell_distance(point, shift=movement * REPULSIVE_FORCE_FUTURE_COUNT)
#     distance = min(distance_to_obstacle, distance_to_future_obstacle)
#     if distance < MIN_DISTANCE_REPULSIVE_FORCE:
#         if distance == distance_to_obstacle:
#             position_with_shift = obstacle.get_position()
#             #
#             temp = (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
#             rep_force_normal = SCALING_FACTOR_REPULSIVE_FORCE * temp
#             rep_force_with_rotation = get_rotational_repulsive_force(temp, current_orientation_robot)
#             movement_field_vector += rep_force_normal + rep_force_with_rotation
#             # movement_field_vector/=2
#             #
#             # movement_field_vector += SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
#             # movement_field_vector/=2
#         else:
#             if point_in_rectangle:
#                 return movement_field_vector
#             distance += OBSTACLE_FUTURE_ADJUSTMENT

#             position_with_shift = obstacle.get_position() + movement * REPULSIVE_FORCE_FUTURE_COUNT
#             if position_with_shift[0] == point[0] and position_with_shift[1] == point[1]:
#                 position_with_shift = position_with_shift - normalized_movement*distance
#             else:
#                 position_with_shift = position_with_shift + (position_with_shift - point) / np.linalg.norm(position_with_shift - point) * distance
#             #
#             temp = (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
#             rep_force_normal = SCALING_FACTOR_REPULSIVE_FORCE * temp
#             rep_force_with_rotation = get_rotational_repulsive_force(temp, current_orientation_robot)
#             return rep_force_normal + rep_force_with_rotation
#             #
#             # return SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
           
#     return movement_field_vector


# def get_rotational_repulsive_force(temp_term: np.array, current_direction: np.array):
#     ortho_direction_right = np.dot(CLOCKWISE_MATRIX, current_direction)
#     moving_towards_right = np.dot(ortho_direction_right,current_direction) >= 0
#     # rotation_matrix = COUNTERCLOCKWISE_MATRIX if moving_towards_right else CLOCKWISE_MATRIX 
#     rotation_matrix = CLOCKWISE_MATRIX if moving_towards_right else COUNTERCLOCKWISE_MATRIX
#     return SCALING_FACTOR_REPULSIVE_ROTATION_FORCE * np.dot(rotation_matrix, temp_term)


def movement_force3(point: np.array, obstacle, GOAL):
    current_orientation_robot = GOAL - point

    movement = obstacle.get_movement() #/ MOVEMENT_REPULSIVE_FORCE_COUNT

    # Falls das Objekt sich nicht bewegt -> normale Berechnung:
    if movement[0] == 0 and movement[1] == 0:# or obstacle.get_shell_distance(point) > np.linalg.norm(movement * REPULSIVE_FORCE_FUTURE_COUNT) + MIN_DISTANCE_REPULSIVE_FORCE:  # um Performanc zusparen ggf
        distance = obstacle.get_shell_distance(point)
        if distance < MIN_DISTANCE_REPULSIVE_FORCE:
            position_with_shift = obstacle.get_position()

            temp = (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
            rep_force_normal = SCALING_FACTOR_REPULSIVE_FORCE * temp
            # rep_force_with_rotation = get_rotational_repulsive_force(temp, current_orientation_robot)
            return rotate_vector(rep_force_normal, current_orientation_robot, movement)
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
        # rep_force_with_rotation = get_rotational_repulsive_force(temp, current_orientation_robot)
        movement_field_vector += rotate_vector(rep_force_normal, current_orientation_robot, movement)
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
            # rep_force_with_rotation = get_rotational_repulsive_force(temp, current_orientation_robot)
            movement_field_vector += rotate_vector(rep_force_normal, current_orientation_robot, movement)
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
            
            temp = (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
            rep_force_normal = SCALING_FACTOR_REPULSIVE_FORCE * temp
            # rep_force_with_rotation = get_rotational_repulsive_force(temp, current_orientation_robot)
            return rotate_vector(rep_force_normal, current_orientation_robot, movement)
            #
            # return SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
           
    return movement_field_vector

def rotate_vector(vector: np.array, current_direction: np.array, obstacle_movement: np.array):
    a = REPULSIVE_FORCE_ROTATION

    ortho_direction_right = np.dot(CLOCKWISE_MATRIX, current_direction)
    moving_towards_right = np.dot(ortho_direction_right, obstacle_movement) >= 0
    a = -a if moving_towards_right else a 

    return np.array(((np.cos(a), -np.sin(a)),(np.sin(a), np.cos(a)))).dot(vector)
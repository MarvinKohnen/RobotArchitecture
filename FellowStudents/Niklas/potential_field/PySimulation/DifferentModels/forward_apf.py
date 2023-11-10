import numpy as np

SCALING_FACTOR_ATTRACTION_FORCE = 1/20 #.5# 1/20
SCALING_FACTOR_REPULSIVE_FORCE = 5 #13/20 # 10# 13/20 #7/20
MIN_DISTANCE_REPULSIVE_FORCE = 3 #3
REPULSIVE_FORCE_FUTURE_COUNT = 3
OBSTACLE_FUTURE_ADJUSTMENT = 0.8

ROBOT_CIRCLE_SIZE = 0.1

#############################################################

def potential_force(point, GOAL, OBSTACLES):
    return attraction_force(point, GOAL) + repulsive_force(point, OBSTACLES)

def attraction_force(point, GOAL):
    return SCALING_FACTOR_ATTRACTION_FORCE * (GOAL - np.array(point))

def repulsive_force(point, OBSTACLES):
    rep_force = 0
    for circle in OBSTACLES:
        rep_force += movement_force3(point, circle)
    return rep_force


def movement_force3(point: np.array, obstacle):
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

    movement_field_vector = 0
    point_in_rectangle = False

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
        distance_adjustment_scaling = (np.linalg.norm(-vektor_between_line_and_point + point - point_on_middle_line) / np.linalg.norm(movement * REPULSIVE_FORCE_FUTURE_COUNT))
        
        orth_distance = np.linalg.norm(vektor_between_line_and_point) + OBSTACLE_FUTURE_ADJUSTMENT *distance_adjustment_scaling - (obstacle.radius + ROBOT_CIRCLE_SIZE)
        
        sign = -1 if np.dot(-vektor_between_line_and_point, normalized_orthogonal_movement) < 0 else 1
        position_with_shift = sign * normalized_orthogonal_movement * orth_distance + point
        # position_with_shift = -vektor_between_line_and_point + point        ## TODO: anpassen damit nahc vorne gedrückt Robo

        movement_field_vector += SCALING_FACTOR_REPULSIVE_FORCE * (1 / orth_distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / orth_distance**3
        point_in_rectangle = True
        # return 10

    distance_to_obstacle = obstacle.get_shell_distance(point)
    distance_to_future_obstacle = obstacle.get_shell_distance(point, shift=movement * REPULSIVE_FORCE_FUTURE_COUNT)
    distance = min(distance_to_obstacle, distance_to_future_obstacle)
    if distance < MIN_DISTANCE_REPULSIVE_FORCE:
        if distance == distance_to_obstacle:
            position_with_shift = obstacle.get_position()
            movement_field_vector += SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
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
            return SCALING_FACTOR_REPULSIVE_FORCE * (1 / distance - 1 / MIN_DISTANCE_REPULSIVE_FORCE) * (np.array(point) - position_with_shift) / distance**3
        
    return movement_field_vector
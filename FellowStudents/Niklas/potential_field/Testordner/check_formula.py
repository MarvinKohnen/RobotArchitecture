import numpy as np


ROBO_POS = xa0, ya0 = (0,0)
ROBO_MOVEMENT = xat, yat = (1,0)

OBSTACLE_POS = xb0, yb0 = (1,1)
OBSTACLE_MOVEMENT = xbt,ybt = (0,-1)

mintime = -(xa0*xat - xat*xb0 - (xa0 - xb0)*xbt + ya0*yat - yat*yb0 - (ya0 - yb0)*ybt) /(xat**2 - 2*xat*xbt + xbt**2 + yat**2 - 2*yat*ybt + ybt**2)
mindist = np.sqrt((mintime*xat - mintime*xbt + xa0 - xb0)**2 + (mintime*yat - mintime*ybt + ya0 - yb0)**2)

collide = mindist <= 0.2 + 0.1

print(mintime, mindist, collide)





# a = np.array((2,5))

# print(1/np.linalg.norm(a))



    # def check_collision(self, obstacle_point1, obstacle_point2, distance_travelled_so_far):
    # # https://gamedev.stackexchange.com/questions/97337/detect-if-two-objects-are-going-to-collide
    #     global COLLISIONS_DETECTED

    #     endpoint_check = (np.linalg.norm(ROBO_OLD_POS - obstacle_point1) < self.radius + ROBOT_CIRCLE_SIZE,
    #             np.linalg.norm(ROBO_OLD_POS - obstacle_point2) < self.radius + ROBOT_CIRCLE_SIZE,
    #             np.linalg.norm(ROBO_POS - obstacle_point1) < self.radius + ROBOT_CIRCLE_SIZE,
    #             np.linalg.norm(ROBO_POS - obstacle_point2) < self.radius + ROBOT_CIRCLE_SIZE)
    #     if any(endpoint_check):
    #         COLLISIONS_DETECTED += 1
    #         return
    #     movement_norm = np.linalg.norm(self.current_movement)
    #     distance_between_points = np.linalg.norm(obstacle_point1 - obstacle_point2)
    #     time_start = distance_travelled_so_far/movement_norm
    #     time_end = distance_between_points / movement_norm

    #     xa0, ya0 = obstacle_point1      # OBSTACLE_POS Old
    #     xat, yat = self.current_movement # OBSTACLE_MOVEMENT
        
    #     xb0, yb0 = ROBO_OLD_POS + ROBO_CURRENT_MOVEMENT/np.linalg.norm(ROBO_CURRENT_MOVEMENT) * time_start# ROBO_OLD_POS
    #     xbt,ybt = ROBO_CURRENT_MOVEMENT    #ROBO_POS - ROBO_OLD_POS

    #     mintime = -(xa0*xat - xat*xb0 - (xa0 - xb0)*xbt + ya0*yat - yat*yb0 - (ya0 - yb0)*ybt) /(xat**2 - 2*xat*xbt + xbt**2 + yat**2 - 2*yat*ybt + ybt**2)
    #     mindist = np.sqrt((mintime*xat - mintime*xbt + xa0 - xb0)**2 + (mintime*yat - mintime*ybt + ya0 - yb0)**2)

    #     if mindist < self.radius + ROBOT_CIRCLE_SIZE and 0 <= mintime <= time_end:  # collision detected
    #         COLLISIONS_DETECTED += 1




###########################################


l1 = [np.array((1,2)),np.array((3,4))]
l2 = [np.array((5,6)),np.array((7,8))]

# ls = [l1,l2]
ls = []

ls.append(l1)
ls.append(l2)

print(ls[0])









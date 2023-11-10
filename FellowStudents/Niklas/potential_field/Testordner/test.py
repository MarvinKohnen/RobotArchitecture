import numpy as np

####################
# current_orientation = np.pi/2
# b = 1
# pot_force_vektor = np.array((1,0))

# control_commands = np.array([[np.cos(current_orientation), np.sin(current_orientation)], [-1/b * np.sin(current_orientation), 1/b * np.cos(current_orientation)]]) @ pot_force_vektor
        
# print(control_commands)

####################


# def get_componand_of_a_along_b(a: np.array, b: np.array):
#     return a.dot(b)/ np.linalg.norm(b)

# last_movement = np.array((0,1))
# point = np.array((0,0))

# circle_movement = np.array((1,0))
# circle_position = np.array((1,1))

# v_ao = get_componand_of_a_along_b(last_movement - circle_movement, circle_position - point)

# print(v_ao)


####################

ls = []
ls1 = [1,2,3,4,5]
ls2 = [6,7,8,9,0]

ls.append(ls1)
ls.append(ls2)

ls2 = None

print(ls)
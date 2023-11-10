import sys


counter = 0
counter2 = 0


all_lines = open('collisions.txt').readlines()

for i,line in enumerate(all_lines):
    if 'contact {' in line:
        robot_in_there = 'turtlebot3' in all_lines[i+1] != 'turtlebot3' in all_lines[i+2]
        sphere_in_there = 'sphere_clone' in all_lines[i+1] != 'sphere_clone' in all_lines[i+2]
        if robot_in_there and sphere_in_there:
            counter+=1

        if 'turtlebot3' in all_lines[i+1] and 'sphere_clone' in all_lines[i+2] or 'turtlebot3' in all_lines[i+2] and 'sphere_clone' in all_lines[i+1]:
            counter2+=1
            
            print(all_lines[i+1])
            print(all_lines[i+2])
            print('--------------')
    # if 'turtlebot3' in line:
    #     counter+=1


print(counter, counter2)

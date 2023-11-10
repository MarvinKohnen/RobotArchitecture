import subprocess
import time


# command = ['echo', 'this is a test. Does it come out as a single line?']
# command = ['echo',' this is a test. Does it come out as a single line?',]
# command = ["dir"]

command = ['gz','topic', '-e', '/gazebo/default/physics/contacts']
# gz topic -e '/gazebo/default/physics/contacts'

p = subprocess.Popen(command, stdout=subprocess.PIPE, text=True)

collision_counter = 0
STATE_READ = 0

line1 = ''
line2 = ''

while True:
    line = p.stdout.readline()
    current_line = line.strip()
    # print(current_line)
    # time.sleep(1)

    if 'contact {' in current_line:
        STATE_READ = 1
        continue

    if STATE_READ > 0:
        if STATE_READ == 1:
            line1 = current_line
            STATE_READ += 1
        elif STATE_READ == 2:
            line2 = current_line
            STATE_READ = 0
            if ('turtlebot3' in line1 and 'cylinder_clone' in line2) or ('turtlebot3' in line2 and 'cylinder_clone' in line1):
                collision_counter += 1
            line1 = ''
            line2 = ''   
        elif STATE_READ >2:
            print('ERROR')


    # print(line1, line2)

    # if ('turtlebot3' in line1 and 'cylinder_clone' in line2) or ('turtlebot3' in line2 and 'cylinder_clone' in line1):
    #     collision_counter += 1

    # robot_in_there = 'turtlebot3' in line1 != 'turtlebot3' in line2
    # sphere_in_there = 'cylinder_clone' in line1 != 'cylinder_clone' in line2
    # if robot_in_there and sphere_in_there:
    #     collision_counter+=1

    print(collision_counter)
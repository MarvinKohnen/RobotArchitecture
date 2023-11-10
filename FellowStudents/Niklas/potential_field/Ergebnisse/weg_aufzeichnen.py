import matplotlib.pyplot as plt
import numpy as np
import sys

def plot_different_routes(stepper_modulo: int = 1):
    try:
        logged_points = open(f'/home/nik/catkin_ws/src/bachlorarbeit/potential_field/Ergebnisse/{sys.argv[1]}.txt')
        # logged_points = open('/home/nik/catkin_ws/src/bachlorarbeit/potential_field/Ergebnisse/Points.txt')
        # logged_points = open('/home/nik/Documents/TARTW/src/bachlorarbeit/potential_field/Ergebnisse/Points.txt')
    except:
        logged_points = open('/home/nik/catkin_ws/src/bachlorarbeit/potential_field/Ergebnisse/Points.txt')


    

    plt.figure()
    plt.title("Routen")
    x_points = ()
    y_points = ()
    current_color = np.random.random_sample((3,))
    step_counter = -1
    for line in logged_points.readlines():
        if '---' in line:

            # plt.plot(x_points[::1000],y_points[::1000], color=current_color)
            plt.plot(x_points,y_points, color=current_color)

            x_points = ()
            y_points = ()
            current_color = np.random.random_sample((3,))
            step_counter = -1
        else:
            step_counter += 1 
            if step_counter % stepper_modulo != 0:
                continue
            string = line.split()

            x = float(string[0])
            y = float(string[1])
            
            # x = float(string[0][1::])
            # y = float(string[1][:-1:])

            x_points = (*x_points, x)
            y_points = (*y_points, y)
            
    plt.grid()

    # plt.xlim(-4, 10)
    # plt.ylim(-5, 5)
    plt.show()



if __name__ == '__main__':
    plot_different_routes(100)
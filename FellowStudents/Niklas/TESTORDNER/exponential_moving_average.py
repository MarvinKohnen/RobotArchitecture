import numpy as np
import matplotlib.pyplot as plt



def exponential_moving_average(points:tuple, beta: float):
    add_term = 0
    for k,point in enumerate(points[-2::-1]):
        add_term += beta**k * (1-beta) * np.array(point)


    # for k in range(len(points[0:-1])):
    #     add_term += beta**k * (1-beta) * np.array(points[-1 -k+1])

    return beta**(len(points)) * np.array(points[-1]) + add_term






if __name__ == '__main__':
    plt.title('exponential moving average')

    number_of_points = 105
    increment = 0.2#2*np.pi/number_of_points
    func = np.sin
    beta = 0.5

    points = ((0,0),)
    for i in range(number_of_points):
        x = i * increment
        y = func(x)

        predicted_x, predicted_y = exponential_moving_average(points, beta)
        points = (*points, (x,y))

        if i == 20:
            print(x,y)
            print(predicted_x, predicted_y)

        plt.scatter(x,y,color='blue')
        plt.scatter(predicted_x,predicted_y,color='red')



    plt.grid()
    plt.show()

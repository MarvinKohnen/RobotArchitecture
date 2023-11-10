import matplotlib.pyplot as plt
import numpy as np

class Results:
    used_model: str
    obstacle_velocity: float
    collisions: list
    distance_traveled: list
    time: list
    def __init__(self, used_model) -> None:
        self.used_model = used_model
        self.obstacle_velocity = None
        self.collisions = []
        self.distance_traveled = []
        self.time = []
    @property
    def avg_collisions(self):
        return sum(self.collisions) / len(self.collisions)
    @property
    def avg_distance(self):
        return sum(self.distance_traveled) / len(self.distance_traveled)
    @property
    def avg_time(self):
        return sum(self.time) / len(self.time)
    def __str__(self):
        return f'{self.used_model}\nobstacle_vel: {self.obstacle_velocity}\nAvg.Collisions: {self.avg_collisions}'

def get_paths_lists(stepper_modulo: int = 1, relativ_path: str = 'vel_1/c_apf/Points.txt'):
    try:
        print(f'/home/nik/catkin_ws/src/bachlorarbeit/potential_field/Ergebnisse/specific_tests/{relativ_path}')
        logged_points = open(f'/home/nik/catkin_ws/src/bachlorarbeit/potential_field/Ergebnisse/specific_tests/{relativ_path}')
    except:
        print('Error: wrong file name!')
        return

    all_paths = []
    current_path = []
    step_counter = -1
    for line in logged_points.readlines():
        if '---' in line:

            all_paths.append(current_path)
            current_path = []

            step_counter = -1
        else:
            step_counter += 1 
            if step_counter % stepper_modulo != 0:
                continue
            string = line.split()

            x = float(string[0])
            y = float(string[1])
            point = np.array((x,y))
            current_path.append(point)
    return all_paths

def show_coord_histogram(all_paths: list, coord_index: int = 1, title=''):
    fig, axs = plt.subplots(1, 4, sharey=True, tight_layout=True)

    all_coords = []
    for i in range(4):
        paths = all_paths[i]

        coords = []
        for path in paths:
            for point in path:
                coords.append(point[coord_index])
        all_coords.append(coords)

    n_bins = 20
    for i in range(4):
        axs[i].hist(all_coords[i], bins=n_bins)
        axs[i].set_xlim(-7, 7)

    axs[0].set_ylabel('frequency of the points')
    axs[0].set_xlabel('classical apf')
    axs[1].set_xlabel('forward apf')
    axs[2].set_xlabel('rotational forward apf')
    axs[3].set_xlabel('improved apf')

    plt.xlim(-7, 7)
    # plt.ylim(-5, 5)
    # plt.title(title)
    plt.show() 

def get_test_results(model: str, obstacle_velocity: str = 'vel_1'):
    try:
        logged_points = open(f'/home/nik/catkin_ws/src/bachlorarbeit/potential_field/Ergebnisse/specific_tests/{obstacle_velocity}/{model}/ErgebnissePotential_Field_Results.txt')
        # logged_points = open(f'/home/nik/catkin_ws/src/bachlorarbeit/potential_field/Ergebnisse/deprecated/{obstacle_velocity}/{model}/ErgebnissePotential_Field_Results.txt')
    except:
        print('Error: wrong file name!')
        return

    result_obj = Results(model)
    for line in logged_points.readlines():
        result_list = line.split()
        # print(result_list[3], result_list[6], result_list[13], result_list[18])
        result_obj.obstacle_velocity = float(result_list[3])
        result_obj.collisions.append(int(result_list[6]))
        result_obj.distance_traveled.append(float(result_list[13]))
        result_obj.time.append(float(result_list[18]))
    return result_obj


def draw_path(path_list: list, stepper_modulo: int = 100, file_name = ''):
    plt.figure()
    # plt.title("Pfad")
    counter = 0
    for path in path_list:
        x_points = ()
        y_points = ()
        for point in path[::stepper_modulo]:
            x_points = (*x_points, point[0])
            y_points = (*y_points, point[1])

        plt.plot(x_points,y_points, label=f'{counter}')
        counter += 1
        # if counter == 3: break

    # vorne
    # pos = np.array((8,0))
    # vector = np.array((-1, 0))
    # schräg
    # pos = np.array((8/np.sqrt(2),-8/np.sqrt(2)))
    # vector = np.array((-1/np.sqrt(2), 1/np.sqrt(2)))
    # rechts
    pos = np.array((0,-8))
    vector = np.array((0, 1))

    for i in range(20):
        plt.quiver(*pos, *vector, color='blue', units='xy', scale=2)
        pos = pos + vector

    plt.xlabel('x coordinate')
    plt.ylabel('y coordinate')

    # # # vorne
    # # plt.xlim(-7,3)
    # # plt.ylim(-1.5,1.5)

    # # # schräg
    # # plt.xlim(-3,5)
    # # plt.ylim(-2,1)

    # # # rechts
    # # plt.xlim(-3,5)
    # # plt.ylim(-4,4)

    # # Auswertung:
    plt.xlim(-3,3)
    plt.ylim(-3,3)

    # plt.legend()
    plt.show()
    # plt.savefig(f'{file_name}.png')

def scatter_path(path_list: list, stepper_modulo: int = 100):
    plt.figure()
    plt.title("Pfad")
    counter = 0
    for path in path_list:
        x_points = ()
        y_points = ()
        for point in path[::stepper_modulo]:
            x_points = (*x_points, point[0])
            y_points = (*y_points, point[1])
            # plt.scatter()

        plt.scatter(x_points,y_points)
    plt.show()



def auswertungen_anzeigen_simple(obstacle_vel='vorne'):
    r1 = get_test_results(model='c_apf', obstacle_velocity=obstacle_vel)
    r2 = get_test_results(model='f_apf', obstacle_velocity=obstacle_vel)
    r3 = get_test_results(model='rf_apf', obstacle_velocity=obstacle_vel)
    r4 = get_test_results(model='i_apf', obstacle_velocity=obstacle_vel)

    print('Avg. collisions:')
    print(r1.avg_collisions)
    print(r2.avg_collisions)
    print(r3.avg_collisions)
    print(r4.avg_collisions)
    print()
    print('Avg. distance traveled:')
    print(r1.avg_distance)
    print(r2.avg_distance)
    print(r3.avg_distance)
    print(r4.avg_distance)
    print()
    print('Avg. time spend:')
    print(r1.avg_time)
    print(r2.avg_time)
    print(r3.avg_time)
    print(r4.avg_time)
    print()
    print('------------------')

def get_in_bounds_percentages(all_pathes):
    percentage_list = []
    anzahl_liste = []
    for model_pathes in all_pathes:
        n = 0
        in_bounds_counter = 0
        for path in model_pathes:   
            for point in path:
                n += 1
                # if abs(point[0]) <= 5 and abs(point[1]) <= 5:
                #     in_bounds_counter += 1
                if abs(point[1]) <= 5:
                    in_bounds_counter += 1
        percentage_list.append(in_bounds_counter/n)
        anzahl_liste.append(n)
    print(percentage_list)
    print(anzahl_liste)
    return percentage_list

def show_path_heatmap_for_all(all_paths: list, bins=(20,20)):
    fig, axs = plt.subplots(4)
    fig.suptitle(f"Heatmaps")

    # https://matplotlib.org/stable/tutorials/colors/colormaps.html
    # color_map = plt.cm.hot #plt.cm.jet
    # color_map = plt.cm.jet
    # color_map = plt.cm.OrRd
    color_map = plt.cm.gist_heat_r

    h0 = axs[0].hist2d(*get_x_y_points_as_diff_lists(all_paths[0]),bins=bins, cmap=color_map)
    h1 = axs[1].hist2d(*get_x_y_points_as_diff_lists(all_paths[1]),bins=bins, cmap=color_map)
    h2 = axs[2].hist2d(*get_x_y_points_as_diff_lists(all_paths[2]),bins=bins, cmap=color_map)
    h3 = axs[3].hist2d(*get_x_y_points_as_diff_lists(all_paths[3]),bins=bins, cmap=color_map)

    fig.colorbar(h0[3], ax=axs[0])
    fig.colorbar(h1[3], ax=axs[1])
    fig.colorbar(h2[3], ax=axs[2])
    fig.colorbar(h3[3], ax=axs[3])
    
    plt.show()

def show_heatmap(path: list, bins=(15,15), stepper_modulo=100, save_file=False, file_name='default_name'):
    fig, ax = plt.subplots(1)
    # fig.suptitle(f"Heatmap")

    color_map = plt.cm.gist_heat_r
    h0 = ax.hist2d(*get_x_y_points_as_diff_lists(path, stepper_modulo=stepper_modulo),bins=bins, cmap=color_map)
    fig.colorbar(h0[3], ax=ax)
    ax.set_xlabel('x coordinate')
    ax.set_ylabel('y coordinate')

    border = 10
    ax.set_xlim(-border,border)
    ax.set_ylim(-border,border)

    if save_file:
        plt.savefig(f'{file_name}.png')
    else:
        plt.show()


def get_x_y_points_as_diff_lists(point_list: list, stepper_modulo=100) -> (list, list):
    x_points = ()
    y_points = ()

    logged_point_sum = np.array((0.,0.))
    points_logged = 0

    for points in point_list:
        for point in points[::stepper_modulo]:

            ##### potentielle Einschränkungen für die Punkte -> Gebiet #####
            # if abs(point[0]) >= 6 and abs(point[1]) >= 6:
            #     continue
            ################################################################
            points_logged += 1
            logged_point_sum = logged_point_sum + point
            x_points = (*x_points, point[0])
            y_points = (*y_points, point[1])
    print(logged_point_sum/points_logged)
    return x_points, y_points


def heatmaps():
    all_paths_vel_1 = [get_paths_lists(relativ_path='vel_1/c_apf/Points.txt'), get_paths_lists(relativ_path='vel_1/f_apf/Points.txt'), get_paths_lists(relativ_path='vel_1/rf_apf/Points.txt'), get_paths_lists(relativ_path='vel_1/i_apf/Points.txt')]
    # show_path_heatmap_for_all(all_paths_vel_1, bins=(30,30))
    bild_speichern = False
    show_heatmap(all_paths_vel_1[0], save_file=bild_speichern, file_name='c_apf')
    show_heatmap(all_paths_vel_1[1], save_file=bild_speichern, file_name='f_apf')
    show_heatmap(all_paths_vel_1[2], save_file=bild_speichern, file_name='rf_apf')
    show_heatmap(all_paths_vel_1[3], save_file=bild_speichern, file_name='i_apf')

def collision_bars():
    barWidth = 0.18

    # Indizes 0|2|4|5 in results_objects_list:
    bars1 = [1,1,1,1]   # <------------- theoretische Ergebnisse
    bars2 = [3,3,3,3]   # <------------- Gazebo Ergebnisse
    
    # Set position of bar on X axis
    r1 = np.arange(len(bars1)) - barWidth*1.5
    r2 = [x + barWidth for x in r1]
    r3 = [x + barWidth for x in r2]
    r4 = [x + barWidth for x in r3]
    
    # Make the plot
    plt.bar(r1, bars1, width=barWidth, edgecolor='white', label='theoretical results')
    plt.bar(r2, bars2, width=barWidth, edgecolor='white', label='gazebo results')
    
    # Add xticks on the middle of the group bars
    plt.xlabel(f'Different models', fontweight='bold')
    plt.ylabel('Avg. collisions', fontweight='bold')
    plt.xticks([r for r in range(len(bars1))], ['c_apf', 'f_apf', 'rf_apf', 'i_apf'])
    
    # Create legend & Show graphic
    # plt.title(title)
    # plt.legend()
    # plt.colorbar(mcolors.TABLEAU_COLORS)
    plt.show()

def print_path_lengths(path_lists: list):
    for path in path_lists:
        last_point = path[0]
        length = 0
        for point in path:
            length += np.linalg.norm(point - last_point)
            last_point = point
        print(length)

            

if __name__ == '__main__':
    plt.rc('font', size=15) 

    ordner = 'vorne'
    all_paths_vel_1 = [get_paths_lists(relativ_path=f'{ordner}/c_apf/Points.txt'), get_paths_lists(relativ_path=f'{ordner}/f_apf/Points.txt'), get_paths_lists(relativ_path=f'{ordner}/rf_apf/Points.txt'), get_paths_lists(relativ_path=f'{ordner}/i_apf/Points.txt')]
    all_paths = [all_paths_vel_1[0][0], all_paths_vel_1[1][0], all_paths_vel_1[2][0], all_paths_vel_1[3][0]]
    # all_paths = [all_paths_vel_1[2][0], ]
    print_path_lengths(all_paths)
    draw_path(all_paths, file_name='pfad_vorne')

    ordner = 'schräg'
    all_paths_vel_1 = [get_paths_lists(relativ_path=f'{ordner}/c_apf/Points.txt'), get_paths_lists(relativ_path=f'{ordner}/f_apf/Points.txt'), get_paths_lists(relativ_path=f'{ordner}/rf_apf/Points.txt'), get_paths_lists(relativ_path=f'{ordner}/i_apf/Points.txt')]
    all_paths = [all_paths_vel_1[0][0], all_paths_vel_1[1][0], all_paths_vel_1[2][0], all_paths_vel_1[3][0]]
    print_path_lengths(all_paths)
    draw_path(all_paths, file_name='pfad_schräg')

    ordner = 'rechts'
    all_paths_vel_1 = [get_paths_lists(relativ_path=f'{ordner}/c_apf/Points.txt'), get_paths_lists(relativ_path=f'{ordner}/f_apf/Points.txt'), get_paths_lists(relativ_path=f'{ordner}/rf_apf/Points.txt'), get_paths_lists(relativ_path=f'{ordner}/i_apf/Points.txt')]
    all_paths = [all_paths_vel_1[0][0], all_paths_vel_1[1][0], all_paths_vel_1[2][0], all_paths_vel_1[3][0]]
    print_path_lengths(all_paths)
    draw_path(all_paths, file_name='pfad_rechts')

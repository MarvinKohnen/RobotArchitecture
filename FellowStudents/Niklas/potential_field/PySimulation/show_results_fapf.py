import pickle
import os
import matplotlib.pyplot as plt
from simulation import Results, ModelResults
import itertools
import matplotlib.colors as mcolors
import numpy as np
import sys

results_objects_list = []   # liste von listen

plt.rc('font', size=15) 

# robot_colors = {"classic_apf": 'blue', "rotational_fapf": 'green' ,"forward_apf": 'black'}  # Farbschema für die verschiedenen Modelle
robot_colors = {"classic_apf": 'cyan', "rotational_fapf": 'lime' ,"forward_apf": 'black', 'improved_apf': 'magenta'}

def load_all_results():
    """Lädt alle Result-Objecte im Order Result_Objects und speichert diese in der Liste results_objects_list"""
    counter = 0
    direct_list = os.listdir(f'{BASE_PATH}{ADDITIONAL_PATH}')
    direct_list.sort(key=float)

    print(direct_list)

    for direct_name in direct_list:
        lst = []
        for file_name in os.listdir(f'{BASE_PATH}{ADDITIONAL_PATH}{direct_name}/'):
            # if os.path.isdir(file_name):
            #     continue
            if file_name == 'settings.txt': continue
            # print(file_name)
            file = open(f'{BASE_PATH}{ADDITIONAL_PATH}{direct_name}/{file_name}', 'rb')
            lst.append(pickle.load(file))
            counter += 1
            file.close()
        results_objects_list.append(lst)
    print(f'Found {counter} results in {BASE_PATH}{ADDITIONAL_PATH}')
    # print(len(results_objects_list))
    # print(len(results_objects_list[0]))

    for lst in results_objects_list:
        lst.sort(key=lambda result: result.robot_max_velocity)
        lst.sort(key=lambda result: result.obstacle_count)
        lst.sort(key=lambda result: result.obstacle_velocity)
        
    # for result in results_objects_list[1]:
    #     print(result.print_settings())

def get_result_obj_from_file(file_name: str) -> Results:
    file = open(f'{BASE_PATH}{ADDITIONAL_PATH}{file_name}', 'rb')
    results_object = pickle.load(file)
    file.close()
    return results_object

def draw_all_path_from_one_model_in_one_sim(model_results: ModelResults):
    all_colors = mcolors.TABLEAU_COLORS
    colors = itertools.cycle(all_colors)
    # plt.figure()
    # plt.title(f"Walked path - {model_results.used_model}")
    fig, axs = plt.subplots(len(model_results.walked_paths), figsize=(10, 10))
    fig.suptitle(f"Walked path - {model_results.used_model}")

    for i, path in enumerate(model_results.walked_paths):
        x_points = ()
        y_points = ()
        for point in path:
            x_points = (*x_points, point[0])
            y_points = (*y_points, point[1])
        
        # axs[i].set_xlim(-10, 10)
        # axs[i].set_ylim(-10, 10)
        axs[i].grid()
        axs[i].plot(x_points, y_points, color = next(colors), label=i)   #, label = f'{robot.used_model}, C:{robot.collisions}, t:{robot.time_steps}'
    
    # plt.grid()
    # plt.legend()
    # plt.xlim(-10, 10)
    # plt.ylim(-10, 10)
    plt.show()


def draw_path_from_one_test_run(result_obj: Results):
#     all_colors = mcolors.TABLEAU_COLORS
#     colors = itertools.cycle(all_colors)
    # plt.figure()
    # plt.title(f"Walked path - {model_results.used_model}")
    fig, axs = plt.subplots(len(result_obj.capf_results.walked_paths), figsize=(10, 10))
    fig.suptitle(f"Walked path of different models")

    def draw_path_on_axs(axs, model_results: ModelResults):
        for i, path in enumerate(model_results.walked_paths):
            x_points = ()
            y_points = ()
            for point in path:
                x_points = (*x_points, point[0])
                y_points = (*y_points, point[1])
            
            # axs[i].set_xlim(-10, 10)
            # axs[i].set_ylim(-10, 10)
            axs[i].grid()
            axs[i].plot(x_points, y_points, color = robot_colors[model_results.used_model], label=i)

    draw_path_on_axs(axs, result_obj.capf_results)
    draw_path_on_axs(axs, result_obj.fapf_results)
    draw_path_on_axs(axs, result_obj.rfapf_results)
    plt.show()

def get_x_y_points_as_diff_lists(collision_for_run: list) -> list:
    x_points = ()
    y_points = ()
    for points in collision_for_run:
        for point in points:
            x_points = (*x_points, point[0])
            y_points = (*y_points, point[1])
    return x_points, y_points

def get_path_length(path: list):
    length = 0
    last_point = path[0]
    for point in path[1::]:
        length += np.linalg.norm(last_point - point)
        last_point = point
    return length

def get_avg_path_length(results: ModelResults):
    return sum([get_path_length(path) for path in results.walked_paths]) / len(results.walked_paths)

def get_avg_model_collisions(results: ModelResults):
    return sum(results.collisions)/len(results.collisions)

def get_avg_model_time_steps(results: ModelResults):
    return sum(results.time_steps)/len(results.time_steps)

def calc_area_between_two_points(x: tuple, y: tuple):
    return (y[0]-x[0]) * (x[1] + y[1]) / 2

def calc_area_under_lin_function(points: int):
    area = 0
    for i in range(len(points)-1):
        area += calc_area_between_two_points(points[i], points[i+1])
    # print(f'Die Fläche unter der Kurve beträgt: {area}')
    return area

def get_curvature_of_path(path: list):
    i = 2
    curvature = ()
    while i < len(path):
        a = np.linalg.norm(path[i-1] - path[i-2])
        b = np.linalg.norm(path[i] - path[i-1])
        c = np.linalg.norm(path[i] - path[i-2])
        s = (a+b+c)/2
        # print(s*(s-a)*(s-b)*(s-c))
        # print(path[i-2], path[i-1], path[i])
        temp = s*(s-a)*(s-b)*(s-c)
        if temp < 0:
            A = 0
        else:
            A = np.sqrt(temp)    # Calculating area using Herons formula
        c_at_this_point = 4*A/(a*b*c)   # # Menger curvature
        curvature = (*curvature, c_at_this_point)
        i += 1
    return curvature

def get_avg_curvature_integral(results: ModelResults):
    avg_area = 0
    for path in results.walked_paths:
        curv_list = list(enumerate(get_curvature_of_path(path)))
        avg_area += calc_area_under_lin_function(curv_list)
    return avg_area/len(results.walked_paths)

def print_avg_stats(result_obj: Results):
    print('----------------------')
    print('Settings:')
    print('Test runs:', result_obj.test_count)
    print('Obstacle count:', result_obj.obstacle_count)
    print('Obstacle velocity:', result_obj.obstacle_velocity)
    print('Robot max velocity:',result_obj.robot_max_velocity)
    print()
    print('Forward APF:')
    print('Avg. time steps:',get_avg_model_time_steps(result_obj.fapf_results))
    print('Avg. collisions:',get_avg_model_collisions(result_obj.fapf_results))
    print('Avg. path length:', get_avg_path_length(result_obj.fapf_results))
    print('Avg. curvature integral:', get_avg_curvature_integral(result_obj.fapf_results))
    print('----------------------')


def show_curvature_for_one_testrun(result_obj: Results):
    fig, axs = plt.subplots(4, figsize=(10, 10))
    fig.suptitle(f"Curvature")
    all_colors = mcolors.TABLEAU_COLORS
    colors = itertools.cycle(all_colors)

    for path in result_obj.capf_results.walked_paths:
        curv_list = get_curvature_of_path(path)
        axs[0].plot(range(len(curv_list)), curv_list, color=next(colors))

    for path in result_obj.fapf_results.walked_paths:
        curv_list = get_curvature_of_path(path)
        axs[1].plot(range(len(curv_list)), curv_list, color=next(colors))

    for path in result_obj.rfapf_results.walked_paths:
        curv_list = get_curvature_of_path(path)
        axs[2].plot(range(len(curv_list)), curv_list, color=next(colors))

    for path in result_obj.iapf_results.walked_paths:
        curv_list = get_curvature_of_path(path)
        axs[3].plot(range(len(curv_list)), curv_list, color=next(colors))

    plt.show()



def show_collisions_heatmap_for_one_testrun(result_obj: Results, bins=(100,100)):
    fig, axs = plt.subplots(4, figsize=(10, 10))
    fig.suptitle(f"Heatmaps of collisions")

    # https://matplotlib.org/stable/tutorials/colors/colormaps.html
    # color_map = plt.cm.hot #plt.cm.jet
    # color_map = plt.cm.jet
    # color_map = plt.cm.OrRd
    color_map = plt.cm.gist_heat_r

    h0 = axs[0].hist2d(*get_x_y_points_as_diff_lists(result_obj.capf_results.collision_points),bins=bins, cmap=color_map)
    h1 = axs[1].hist2d(*get_x_y_points_as_diff_lists(result_obj.fapf_results.collision_points),bins=bins, cmap=color_map)
    h2 = axs[2].hist2d(*get_x_y_points_as_diff_lists(result_obj.rfapf_results.collision_points),bins=bins, cmap=color_map)
    h3 = axs[2].hist2d(*get_x_y_points_as_diff_lists(result_obj.rfapf_results.collision_points),bins=bins, cmap=color_map)

    fig.colorbar(h0[3], ax=axs[0])
    fig.colorbar(h1[3], ax=axs[1])
    fig.colorbar(h2[3], ax=axs[2])
    fig.colorbar(h3[3], ax=axs[3])
    
    plt.show()


def _get_collision_list(result_list: list, obstacles_speed: float):
    lst = []
    for result_obj in result_list:
        if result_obj.obstacle_velocity == obstacles_speed:
            lst.append(get_avg_model_collisions(result_obj.fapf_results))
            # print(result_obj.obstacle_count)
    return lst


def show_collision_bars_selection(obstacles_speed=.5):
    # https://www.tutorialspoint.com/matplotlib/matplotlib_bar_plot.htm
    # https://python-graph-gallery.com/11-grouped-barplot/
    # set width of bars
    barWidth = 0.2
    
    # set heights of bars
    # bars1 = [12, 30, 1, 8, 22]
    # bars2 = [28, 6, 16, 5, 10]
    # bars3 = [29, 3, 24, 25, 17]
    # bars4 = [29, 3, 24, 25, 17]

    # Indizes 0|2|4|5 in results_objects_list:
    bars1 = _get_collision_list(results_objects_list[0],obstacles_speed)
    bars2 = _get_collision_list(results_objects_list[2],obstacles_speed)
    bars3 = _get_collision_list(results_objects_list[4],obstacles_speed)
    bars4 = _get_collision_list(results_objects_list[5],obstacles_speed)
    
    # Set position of bar on X axis
    r1 = np.arange(len(bars1))
    r2 = [x + barWidth for x in r1]
    r3 = [x + barWidth for x in r2]
    r4 = [x + barWidth for x in r3]
    
    # Make the plot
    plt.bar(r1, bars1, width=barWidth, edgecolor='white', label='0.5')
    plt.bar(r2, bars2, width=barWidth, edgecolor='white', label='3')
    plt.bar(r3, bars3, width=barWidth, edgecolor='white', label='10')
    plt.bar(r4, bars4, width=barWidth, edgecolor='white', label='20')
    
    # Add xticks on the middle of the group bars
    plt.xlabel(f'Avg. collisions with obstacle_speed={obstacles_speed}', fontweight='bold')
    plt.xticks([r + barWidth for r in range(len(bars1))], ['2', '3', '4', '5', '6'])
    
    # Create legend & Show graphic
    plt.legend()
    plt.show()


def show_collision_bars_all(obstacles_speed=.5):
    # https://www.tutorialspoint.com/matplotlib/matplotlib_bar_plot.htm
    # https://python-graph-gallery.com/11-grouped-barplot/
    # set width of bars
    barWidth = 0.1

    # Indizes 0|2|4|5 in results_objects_list:
    bars1 = _get_collision_list(results_objects_list[0],obstacles_speed)
    bars2 = _get_collision_list(results_objects_list[1],obstacles_speed)
    bars3 = _get_collision_list(results_objects_list[2],obstacles_speed)
    bars4 = _get_collision_list(results_objects_list[3],obstacles_speed)
    bars5 = _get_collision_list(results_objects_list[4],obstacles_speed)
    bars6 = _get_collision_list(results_objects_list[5],obstacles_speed)
    
    # Set position of bar on X axis
    r1 = np.arange(len(bars1)) - barWidth*1.5
    r2 = [x + barWidth for x in r1]
    r3 = [x + barWidth for x in r2]
    r4 = [x + barWidth for x in r3]
    r5 = [x + barWidth for x in r4]
    r6 = [x + barWidth for x in r5]
    
    # Make the plot
    plt.bar(r1, bars1, width=barWidth, edgecolor='white', label='0.5')
    plt.bar(r2, bars2, width=barWidth, edgecolor='white', label='1')
    plt.bar(r3, bars3, width=barWidth, edgecolor='white', label='3')
    plt.bar(r4, bars4, width=barWidth, edgecolor='white', label='5')
    plt.bar(r5, bars5, width=barWidth, edgecolor='white', label='10')
    plt.bar(r6, bars6, width=barWidth, edgecolor='white', label='20')
    
    # Add xticks on the middle of the group bars
    plt.xlabel(f'Avg. collisions with obstacle_speed={obstacles_speed}', fontweight='bold')
    plt.xticks([r + barWidth for r in range(len(bars1))], ['2', '3', '4', '5', '6'])
    
    # Create legend & Show graphic
    plt.legend()
    plt.show()


def show_all_data():
    for test in results_objects_list:
        for result_obj in test:
            print_avg_stats(result_obj)
            
            # show_collisions_heatmap_for_one_testrun(result_obj)
            # show_curvature_for_one_testrun(result_obj)


def _get_avg_collisions_per_paramater(obstacles_speed = None):
    avg_collisions = []
    for test in results_objects_list:
        avg_collision_per_parameter = 0
        counter = 0
        for result_obj in test:
            if obstacles_speed is None:
                avg_collision_per_parameter += get_avg_model_collisions(result_obj.fapf_results)
                counter += 1
            elif obstacles_speed == result_obj.obstacle_velocity:
                avg_collision_per_parameter += get_avg_model_collisions(result_obj.fapf_results)
                counter += 1
        avg_collisions.append(avg_collision_per_parameter/counter)
    return avg_collisions

def _get_avg_step_per_paramater(obstacles_speed = None):
    avg_steps = []
    for test in results_objects_list:
        avg_steps_per_parameter = 0
        counter = 0
        for result_obj in test:
            if obstacles_speed is None:
                avg_steps_per_parameter += get_avg_model_time_steps(result_obj.fapf_results)
                counter += 1
            elif obstacles_speed == result_obj.obstacle_velocity:
                avg_steps_per_parameter += get_avg_model_time_steps(result_obj.fapf_results)
                counter += 1
        avg_steps.append(avg_steps_per_parameter/counter)
    return avg_steps

def _get_avg_path_per_paramater(obstacles_speed = None):
    avg_path_length = []
    for test in results_objects_list:
        avg_path_lengt_per_parameter = 0
        counter = 0
        for result_obj in test:
            if obstacles_speed is None:
                avg_path_lengt_per_parameter += get_avg_path_length(result_obj.fapf_results)
                counter += 1
            elif obstacles_speed == result_obj.obstacle_velocity:
                avg_path_lengt_per_parameter += get_avg_path_length(result_obj.fapf_results)
                counter += 1
        avg_path_length.append(avg_path_lengt_per_parameter/counter)
    return avg_path_length


def show_avg_collisions_per_paramater(per_parameter=False):
    if not per_parameter:
        barWidth = 0.3

        # Indizes 0|2|4|5 in results_objects_list:
        bars1 = _get_avg_collisions_per_paramater()
        # print(bars1)
        
        # Set position of bar on X axis
        r1 = np.arange(len(bars1))
        
        # Make the plot
        plt.bar(r1, bars1, width=barWidth, edgecolor='white', label='Avg. collisions')
        
        # Add xticks on the middle of the group bars
        plt.xlabel(f'Different repulsive forces', fontweight='bold')
        # plt.xticks([r + barWidth for r in range(len(bars1))], ['0.5', '1', '1.5', '2'])
        plt.xticks([r for r in range(len(bars1))], ['0', '1', '2', '3', '4'])
        
        # Create legend & Show graphic
        plt.legend()
        plt.show()

    else:
        barWidth = 0.18

        # Indizes 0|2|4|5 in results_objects_list:
        bars1 = _get_avg_collisions_per_paramater(0.5)
        bars2 = _get_avg_collisions_per_paramater(1)
        bars3 = _get_avg_collisions_per_paramater(1.5)
        bars4 = _get_avg_collisions_per_paramater(2)
        
        # Set position of bar on X axis
        r1 = np.arange(len(bars1)) - barWidth*1.5
        r2 = [x + barWidth for x in r1]
        r3 = [x + barWidth for x in r2]
        r4 = [x + barWidth for x in r3]
        
        # Make the plot
        plt.bar(r1, bars1, width=barWidth, edgecolor='white', label='0.5')
        plt.bar(r2, bars2, width=barWidth, edgecolor='white', label='1')
        plt.bar(r3, bars3, width=barWidth, edgecolor='white', label='1.5')
        plt.bar(r4, bars4, width=barWidth, edgecolor='white', label='2')
        
        # Add xticks on the middle of the group bars
        plt.xlabel(f'Different repulsive forces', fontweight='bold')
        # plt.xticks([r + barWidth for r in range(len(bars1))], ['2', '3', '4', '5', '6'])
        plt.xticks([r for r in range(len(bars1))], ['0', '1', '2', '3', '4'])
        
        # Create legend & Show graphic
        plt.legend()
        plt.show()


def show_avg_func_per_parameter(func, per_parameter=False, y_label=''):
    # plt.figure(figsize=(16,9))

    if not per_parameter:
        barWidth = 0.3

        # Indizes 0|2|4|5 in results_objects_list:
        bars1 = func()
        # print(bars1)
        
        # Set position of bar on X axis
        r1 = np.arange(len(bars1))
        
        # Make the plot
        plt.bar(r1, bars1, width=barWidth, edgecolor='white', label='')
        
        # Add xticks on the middle of the group bars
        plt.xlabel(f'Different future counts', fontweight='bold')
        plt.ylabel(y_label, fontweight='bold')
        # plt.xticks([r + barWidth for r in range(len(bars1))], ['0.5', '1', '1.5', '2'])
        # plt.xticks([r for r in range(len(bars1))], ['0', '1', '2', '3', '4'])
        plt.xticks([r for r in range(len(bars1))], ['0', '1', '2', '3', '4', '5'])
        
        # Create legend & Show graphic
        # plt.legend()
        plt.show()
        

    else:
        barWidth = 0.18

        # Indizes 0|2|4|5 in results_objects_list:
        bars1 = func(0.5)
        bars2 = func(1)
        bars3 = func(1.5)
        bars4 = func(2)

        print(bars2)
        print(bars4)
        
        # Set position of bar on X axis
        r1 = np.arange(len(bars1)) - barWidth*1.5
        r2 = [x + barWidth for x in r1]
        r3 = [x + barWidth for x in r2]
        r4 = [x + barWidth for x in r3]
        
        # Make the plot
        plt.bar(r1, bars1, width=barWidth, edgecolor='white', label='0.5')
        plt.bar(r2, bars2, width=barWidth, edgecolor='white', label='1')
        plt.bar(r3, bars3, width=barWidth, edgecolor='white', label='1.5')
        plt.bar(r4, bars4, width=barWidth, edgecolor='white', label='2')
        
        # Add xticks on the middle of the group bars
        plt.xlabel(f'Different future counts', fontweight='bold')
        plt.ylabel(y_label, fontweight='bold')
        # plt.xticks([r for r in range(len(bars1))], ['0', '1', '2', '3', '4'])
        plt.xticks([r for r in range(len(bars1))], ['0', '1', '2', '3', '4', '5'])
        
        # Create legend & Show graphic
        # plt.title(title)
        # plt.legend()
        # plt.colorbar(mcolors.TABLEAU_COLORS)
        plt.show()

def analyze_different_future_counts():
    show_avg_func_per_parameter(func=_get_avg_collisions_per_paramater, per_parameter=False, y_label='Avg. collisions')
    show_avg_func_per_parameter(func=_get_avg_collisions_per_paramater, per_parameter=True, y_label='Avg. collisions')

    show_avg_func_per_parameter(func=_get_avg_step_per_paramater, per_parameter=False, y_label='Avg. calculation steps')
    show_avg_func_per_parameter(func=_get_avg_step_per_paramater, per_parameter=True, y_label='Avg. calculation steps')

    show_avg_func_per_parameter(func=_get_avg_path_per_paramater, per_parameter=False, y_label='Avg. path length')
    show_avg_func_per_parameter(func=_get_avg_path_per_paramater, per_parameter=True, y_label='Avg. path length')


if __name__== '__main__':
    
    if len(sys.argv) <= 1:
        BASE_PATH = 'Result_Objects/forward_apf_results/'
        ADDITIONAL_PATH = 'different_future_counts/'
    else:
        try:
            BASE_PATH = str(sys.argv[1])    
            ADDITIONAL_PATH = ''
        except:
            print('ERROR, falsche Pfadeingabe als String!')

    
    load_all_results()
    # show_all_data()

    analyze_different_future_counts()


    # print(_get_avg_collisions_per_paramater(0.5))
    # print(_get_avg_collisions_per_paramater(1))
    # print(_get_avg_collisions_per_paramater(1.5))
    # print(_get_avg_collisions_per_paramater(2))

    # print(_get_avg_collisions_per_paramater(0.5)[-2::])
    # print(_get_avg_collisions_per_paramater(1)[-2::])
    # print(_get_avg_collisions_per_paramater(1.5)[-2::])
    # print(_get_avg_collisions_per_paramater(2)[-2::])
import pickle
import os
import matplotlib.pyplot as plt
from simulation import Results, ModelResults
import itertools
import matplotlib.colors as mcolors
import numpy as np
import sys

results_objects_list = []

# robot_colors = {"classic_apf": 'blue', "rotational_fapf": 'green' ,"forward_apf": 'black'}  # Farbschema für die verschiedenen Modelle
robot_colors = {"classic_apf": 'cyan', "rotational_fapf": 'lime' ,"forward_apf": 'black', 'improved_apf': 'magenta'}

def load_all_results():
    """Lädt alle Result-Objecte im Order Result_Objects und speichert diese in der Liste results_objects_list"""
    counter = 0
    for file_name in os.listdir(f'{BASE_PATH}{ADDITIONAL_PATH}'):
        # if os.path.isdir(file_name):
        #     continue
        if file_name == 'settings.txt': continue
        # print(file_name)
        file = open(f'{BASE_PATH}{ADDITIONAL_PATH}{file_name}', 'rb')
        results_objects_list.append(pickle.load(file))
        counter += 1
        file.close()
    print(f'Found {counter} results in {BASE_PATH}{ADDITIONAL_PATH}')

    results_objects_list.sort(key=lambda result: result.robot_max_velocity)
    results_objects_list.sort(key=lambda result: result.obstacle_count)
    results_objects_list.sort(key=lambda result: result.obstacle_velocity)
    # for result in results_objects_list:
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
    print('Classic APF:')
    print('Avg. time steps:',get_avg_model_time_steps(result_obj.capf_results))
    print('Avg. collisions:',get_avg_model_collisions(result_obj.capf_results))
    print('Avg. path length:', get_avg_path_length(result_obj.capf_results))
    print('Avg. curvature integral:', get_avg_curvature_integral(result_obj.capf_results))
    print()
    print('Forward APF:')
    print('Avg. time steps:',get_avg_model_time_steps(result_obj.fapf_results))
    print('Avg. collisions:',get_avg_model_collisions(result_obj.fapf_results))
    print('Avg path length:', get_avg_path_length(result_obj.fapf_results))
    print('Avg. curvature integral:', get_avg_curvature_integral(result_obj.fapf_results))
    print() 
    print('Rotational FAPF:')
    print('Avg. time steps:',get_avg_model_time_steps(result_obj.rfapf_results))
    print('Avg. collisions:',get_avg_model_collisions(result_obj.rfapf_results))
    print('Avg path length:', get_avg_path_length(result_obj.rfapf_results))
    print('Avg. curvature integral:', get_avg_curvature_integral(result_obj.rfapf_results))
    print() 
    print('Improved APF:')
    print('Avg. time steps:',get_avg_model_time_steps(result_obj.iapf_results))
    print('Avg. collisions:',get_avg_model_collisions(result_obj.iapf_results))
    print('Avg path length:', get_avg_path_length(result_obj.iapf_results))
    print('Avg. curvature integral:', get_avg_curvature_integral(result_obj.iapf_results))
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


def show_all_data():
    for result_obj in results_objects_list:
        print_avg_stats(result_obj)
        # show_collisions_heatmap_for_one_testrun(result_obj)
        # show_curvature_for_one_testrun(result_obj)


if __name__== '__main__':
    
    if len(sys.argv) <= 1:
        BASE_PATH = 'Result_Objects/'
        ADDITIONAL_PATH = 'experimentieren/' #'oc2-6_ov1-3_tc100_parameter_test/' #'oc2-6_ov1-3_tc100/' #'tests/'
    else:
        try:
            BASE_PATH = str(sys.argv[1])
            ADDITIONAL_PATH = ''
        except:
            print('ERROR, falsche Pfadeingabe als String!')

    
    load_all_results()

    # draw_all_path_from_one_model_in_one_sim(results_objects_list[-1].capf_results)
    # draw_all_path_from_one_model_in_one_sim(results_objects_list[0].fapf_results)

    # draw_path_from_one_test_run(results_objects_list[2])

    # show_collisions_heatmap_for_one_testrun(results_objects_list[2])

    # result_obj = get_result_obj_from_file(file_name='oc5_tc20_1693608724.4769855')
    # draw_path_from_one_test_run(result_obj)
    # show_collisions_heatmap_for_one_testrun(result_obj)

    # show_curvature_for_one_testrun(get_result_obj_from_file(file_name='oc5_tc20_1693608724.4769855'))

    # show_all_data()


    # result_obj = get_result_obj_from_file(file_name='oCount2_oVel1.5_tCount100_1694087732424281489')
    # print('Avg. curvature integral:', get_avg_curvature_integral(result_obj.capf_results))
    # show_curvature_for_one_testrun(result_obj)





    
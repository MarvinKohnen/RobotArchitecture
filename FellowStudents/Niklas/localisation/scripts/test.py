"""Alle Methoden die für das Movement mit dem Controller gebraucht werden."""
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

import cv2
from cv_bridge import CvBridge

import numpy as np

import numpy as np
import matplotlib.pyplot as plt
import time

import itertools

# from localisation.astar import astar
from localisation.astar2 import astar



angle_offset_lidar = np.pi / 2
CLUSTER_MIN_DIFFERENZ = 0.5


# def clustering(scan_points: tuple, min_point_diff: float):
#     all_clusters = ()
#     zähler = 0
#     for point in scan_points:

#         if float('inf') in point or float('-inf') in point:
#             continue
#         zähler +=1
#         for cluster in all_clusters:
#             cluster_found_for_this_point = False
#             for already_selected_point in cluster:
#                 distance_between_those_points = distance(already_selected_point, point)
#                 if distance_between_those_points < min_point_diff:
#                     cluster = (*cluster, point)
#                     cluster_found_for_this_point = True
#                     break
#             if cluster_found_for_this_point:
#                 break
#         else:
#             all_clusters = (*all_clusters, (point,))
#     print('zähler',zähler)
#     return all_clusters


def clustering(scan_points: tuple, min_point_diff: float):
    all_clusters = []
    zähler = 0
    for point in scan_points:
        if float("inf") in point or float("-inf") in point:
            continue
        zähler += 1
        for i in range(len(all_clusters)):
            cluster_found_for_this_point = False
            for already_selected_point in all_clusters[i]:
                distance_between_those_points = distance(already_selected_point, point)
                if distance_between_those_points < min_point_diff:
                    all_clusters[i] = (*all_clusters[i], point)
                    cluster_found_for_this_point = True
                    break
            if cluster_found_for_this_point:
                break
        else:
            all_clusters.append((point,))
            # all_clusters = (*all_clusters, (point,))
    print("zähler", zähler)
    return all_clusters


def distance(p1: tuple, p2: tuple) -> float:
    """Berechnet die Distanz zwischen zwei Punkten."""
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def pot_cluster_merge(all_clusters: list, first_point, last_point):
    if distance(first_point, last_point) < CLUSTER_MIN_DIFFERENZ:
        first_index = None
        last_index = None
        for i in range(len(all_clusters)):
            for point in all_clusters[i]:
                if point[0] == first_point[0] and point[1] == first_point[1]:
                    first_index = i
                if point[0] == last_point[0] and point[1] == last_point[1]:
                    last_index = i
                if first_index is not None and last_index is not None:
                    if first_index == last_index:
                        return all_clusters
                    break
        all_clusters[first_index] = (
            *all_clusters[first_index],
            *all_clusters[last_index],
        )
        all_clusters.pop(last_index)

        # first_index = all_clusters.index((first_point))
        # last_index = all_clusters.index(last_index)
        # all_clusters[first_index] = (*all_clusters[first_index], all_clusters[last_index])
        # all_clusters.pop(last_index)

        print("hallo")
        return all_clusters


def calculate_median_points(cluster):
    x = 0
    y = 0
    for point in cluster:
        x += point[0]
        y += point[1]
    return (x / len(cluster), y / len(cluster))


def simplify_clusters(all_clusters):  # TODO
    simple_clusters = ()
    for cluster in all_clusters:
        if len(cluster) < 4:
            simple_clusters = (*simple_clusters, (calculate_median_points(cluster),))
            continue
        first_point = np.array(cluster[0])
        second_point = np.array(cluster[1])
        simple_cluster_temp = (cluster[0],)
        diff_vektor = second_point - first_point
        for point in cluster[2:]:
            diff_vektor_temp = np.array(point) - first_point
            if (
                np.dot(diff_vektor_temp, diff_vektor)
                / (np.linalg.norm(diff_vektor_temp) * np.linalg.norm(diff_vektor))
                < 0.7
            ):
                simple_cluster_temp = (*simple_cluster_temp, point)
                diff_vektor = diff_vektor_temp
            else:
                first_point = np.array((point[0], point[1]))
        else:
            simple_cluster_temp = (*simple_cluster_temp, first_point)
            simple_clusters = (*simple_clusters, simple_cluster_temp)

    return simple_clusters


def show_LaserScan_with_clusters(data: LaserScan):
    plt.xlim(-data.range_max, data.range_max)
    plt.ylim(-data.range_max, data.range_max)

    laser_scan_points = calculate_laserScanPoints(data)
    all_clusters = clustering(laser_scan_points, CLUSTER_MIN_DIFFERENZ)
    all_clusters = pot_cluster_merge(
        all_clusters, laser_scan_points[0], laser_scan_points[-1]
    )  # TODO

    all_clusters_simple = simplify_clusters(all_clusters)

    print("TESTTEST", all_clusters_simple)
    print("LÄNGE", len(all_clusters_simple))

    # print(len(data.ranges))

    # zähler = 0
    # for point in data.ranges:
    #     if float('inf') == point or float('-inf') == point:
    #         zähler += 1
    # print('zahlen ungleich unendlich',zähler)

    # draw_all_clusters_points(all_clusters)
    draw_all_clusters_simple(all_clusters_simple)


def draw_all_clusters_simple(all_clusters_simple):
    colors = itertools.cycle(["r", "b", "g"])
    i = 0
    for cluster in all_clusters_simple:
        if len(cluster) == 1:
            plt.scatter(
                *berechne_x_y_werte_aus_punkten(cluster),
                color=next(colors),
                label=f"Cluster {i}",
            )
            continue
        plt.plot(
            *berechne_x_y_werte_aus_punkten(cluster),
            color=next(colors),
            label=f"Cluster {i}",
        )
        i += 1

    plt.legend()
    plt.draw()
    plt.pause(0.0001)
    plt.clf()


def draw_all_clusters_points(all_clusters):
    colors = itertools.cycle(["r", "b", "g"])
    i = 0
    for cluster in all_clusters:
        plt.scatter(
            *berechne_x_y_werte_aus_punkten(cluster),
            color=next(colors),
            label=f"Cluster {i}",
        )
        i += 1

    plt.legend()
    plt.draw()
    plt.pause(0.0001)
    plt.clf()


def berechne_x_y_werte_aus_punkten(points):
    x_werte = ()
    y_werte = ()
    print("länge von points", len(points))
    for point in points:
        print(point)
        x_werte = (*x_werte, point[0])
        y_werte = (*y_werte, point[1])
    return x_werte, y_werte


def show_LaserScan(data: LaserScan):
    """Zeigt die aktuell gemessenen LaserScans in einem Plot."""
    # print(data.header)
    # print(data.angle_min, data.angle_max)
    # print('Increment',data.angle_increment)
    # print(np.round(data.angle_max / data.angle_increment))

    laser_scan_points = calculate_laserScanPoints(data)

    # print(len(laser_scan_points[:][0]), len(laser_scan_points[:][1]))
    # print('ausgabe0', len(laser_scan_points))

    # print(laser_scan_points)
    # print('ausgabe1', laser_scan_points[:][0])
    # print('ausgabe2', laser_scan_points[:][:][0])

    # print(laser_scan_points[:][0])
    # print(laser_scan_points[:][1])
    # print(laser_scan_points)

    # x_werte = ()
    # y_werte = ()
    # for point in laser_scan_points:
    #     x_werte = (*x_werte, point[0])
    #     y_werte = (*y_werte, point[1])

    plt.scatter(*berechne_x_y_werte_aus_punkten(laser_scan_points))

    # plt.scatter(laser_scan_points[:][0], laser_scan_points[:][1])
    plt.xlim(-data.range_max, data.range_max)
    plt.ylim(-data.range_max, data.range_max)

    # plt.arrow(0,0,0,1)
    # plt.annotate("", xy=(0, 1), xytext=(0, 0),
    #         arrowprops=dict(arrowstyle="->"))

    # minimaler ScanRadius
    circle1 = plt.Circle((0, 0), data.range_min, color="r")
    fig = plt.gcf()
    ax = fig.gca()
    ax.add_patch(circle1)

    plt.draw()
    plt.pause(0.0001)
    plt.clf()


def calculate_laserScanPoints(data: LaserScan):
    """Berechnet aus den LaserScan Daten die Messpunkte."""
    array_length = int(np.round(data.angle_max / data.angle_increment)) + 1
    if array_length != len(data.ranges):
        rospy.loginfo("ERROR: LaserScan Ausgabe -> test.py")

    laser_points = ()
    current_angle = data.angle_min + angle_offset_lidar
    for i in range(array_length):
        # if data.ranges[i] == float('inf'):
        #     continue
        laser_points = (
            *laser_points,
            (
                data.ranges[i] * np.cos(current_angle),
                data.ranges[i] * np.sin(current_angle),
            ),
        )

        current_angle += data.angle_increment
    return laser_points


def show_map(data: OccupancyGrid):
    """Gibt das Grid als Bild auf dem Bildschirm aus."""
    map = np.reshape(data.data, (data.info.height, data.info.width)).astype("uint8")
    # map = cv2.rotate(map, cv2.ROTATE_90_CLOCKWISE)

    # for i in range(384):
    #     for j in range(384):
    #         x = map[i,j]
    #         if x == -1:
    #             map[i,j] = 255
    #         if x == 100:
    #             map[i,j] = 100
    #         if x == 0:
    #             map[i,j] = 255
    # print(map)
    cv2.imshow(f"Map ({data.info.width}x{data.info.height})", map)
    cv2.waitKey(0)


def get_walkable_map(data: OccupancyGrid, min_distance_in_m_to_obstacles: float = 0.25):
    """Ermittelt ein Bild der Karte, welche befahren werden kann und ein mind. Abstand vonmin_distance_in_m_to_obstacles zu Objekten eingehalten wird."""
    ## Standart wäre:
    #   -1    -> unknown
    #   0     -> free
    #   100   -> obstacle
    unkown_grid_value = 0
    free_grid_value = 0
    obstacle_grid_value = 100

    show_map = np.reshape(data.data, (data.info.height, data.info.width))
    for i in range(len(show_map)):
        for j in range(len(show_map[0])):
            if show_map[i, j] == -1:
                show_map[i, j] = unkown_grid_value
            elif show_map[i, j] == 0:
                show_map[i, j] = free_grid_value
            elif show_map[i, j] == 100:
                show_map[i, j] = obstacle_grid_value

    map = show_map.astype("uint8")

    min_pixel_distance = int(
        np.ceil(min_distance_in_m_to_obstacles / data.info.resolution)
    )

    # kernel = np.ones((5, 5), int)
    kernel = np.ones((min_pixel_distance, min_pixel_distance), int)
    map_with_convolution = cv2.filter2D(map, -1, kernel)

    return map_with_convolution


def show_map_possible_walk(data: OccupancyGrid):
    ## Standart wäre:
    #   -1    -> unknown
    #   0     -> free
    #   100   -> obstacle
    unkown_grid_value = 0
    free_grid_value = 0
    obstacle_grid_value = 100

    MIN_DISTANCE_IN_M = 0.25

    show_map = np.reshape(data.data, (data.info.height, data.info.width))
    for i in range(len(show_map)):
        for j in range(len(show_map[0])):
            if show_map[i, j] == -1:
                show_map[i, j] = unkown_grid_value
            elif show_map[i, j] == 0:
                show_map[i, j] = free_grid_value
            elif show_map[i, j] == 100:
                show_map[i, j] = obstacle_grid_value

    map = show_map.astype("uint8")

    min_pixel_distance = int(np.ceil(MIN_DISTANCE_IN_M / data.info.resolution))

    # kernel = np.ones((5, 5), int)
    kernel = np.ones((min_pixel_distance, min_pixel_distance), int)
    map_with_convolution = cv2.filter2D(map, -1, kernel)

    # cv2.imshow(f"Map ({data.info.width}x{data.info.height})", map_with_convolution)

    test_map = map_with_convolution.copy()
    for i in range(len(test_map)):
        for j in range(len(test_map[0])):
            if test_map[i, j] == 0:
                test_map[i, j] = 300

    possible_walk_map = map_with_convolution.copy()
    for i in range(len(possible_walk_map)):
        for j in range(len(possible_walk_map[0])):
            if possible_walk_map[i, j] != 0:
                possible_walk_map[i, j] = 0
                continue
            if possible_walk_map[i, j] == 0:
                possible_walk_map[i, j] = 100

    # cv2.imshow("bilder zuzsammen",np.hstack((map,map_with_convolution, test_map, possible_walk_map)))

    # cv2.imshow('Occupancy Grid', map)
    # cv2.imshow('Convolution', map_with_convolution)
    # cv2.imshow('Walkable Map', possible_walk_map)
    cv2.imshow(
        "Occupancy Grid - Convolution - Walkable Map",
        np.hstack((map, map_with_convolution, possible_walk_map)),
    )

    cv2.waitKey(0)


def test_path_finding(data: OccupancyGrid):
    # start = (10,10)
    # # end = (30, 30)
    # end = (330,300)

    # schwieriger Weg
    # start = (240,220)
    # end = (240,40)

    # Punkte zum testen
    start = (270,220)
    end = (270,160)
    # start = (170,220)
    # end = (210,160)

    map = get_walkable_map(data)

    if map[start[0]][start[1]] != 0 or 0 != map[end[0]][end[1]]:
        rospy.loginfo('start or end is a wall!')
        return

    time_start = time.time()
    path = astar(map, start, end, allow_diagonal_movement = True)
    time_end = time.time()
    print(f'finished searching -> time: {time_end - time_start}')

    show_map = map.copy()
    for i in range(len(show_map)):
        for j in range(len(show_map[0])):
            if (i,j) in path:
                show_map[i, j] = 255
            elif show_map[i, j] >= 100:
                show_map[i, j] = 50

    for path_point in path:
        map[path_point] = 30

    # cv2.imshow("Karte mit Weg",map)
    cv2.imshow(f"Karten mit Weg - time: {time_end - time_start}",np.hstack((map, show_map)))
    cv2.waitKey(0)



def show_map2(data: OccupancyGrid):  # not working
    show_map = np.reshape(data.data, (data.info.height, data.info.width))
    kernel = np.ones((3, 3), int)
    map_with_convolution = cv2.filter2D(show_map, cv2.CV_64F, kernel)
    cv2.imshow("Occupancy Grid", show_map)
    cv2.waitKey(3)


def initialize():
    """Initialisert diesen Node und erzeugt den dazugehörigen Publisher, welcher die Bewegungsart steuert."""
    rospy.init_node("localisation_test", anonymous=False)

    # rospy.Subscriber("/map", OccupancyGrid, show_map)
    # rospy.Subscriber("/map", OccupancyGrid, show_map2)

    # rospy.Subscriber("/map", OccupancyGrid, show_map_possible_walk)
    rospy.Subscriber("/map", OccupancyGrid, test_path_finding)

    # sub_LaserScan = rospy.Subscriber("/scan", LaserScan, show_LaserScan)
    # sub_LaserScan = rospy.Subscriber("/scan", LaserScan, show_LaserScan_with_clusters)
    # plt.show()

    rospy.spin()




if __name__ == "__main__":
    plt.figure()
    plt.title("Ausgabe")
    initialize()

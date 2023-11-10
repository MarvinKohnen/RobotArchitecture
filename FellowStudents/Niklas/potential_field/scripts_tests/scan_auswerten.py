import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

import cv2
from cv_bridge import CvBridge

import numpy as np

import numpy as np
import matplotlib.pyplot as plt


angle_offset_lidar = np.pi / 2
CLUSTER_MIN_DIFFERENZ = 0.5


def show_LaserScan(data: LaserScan):
    """Zeigt die aktuell gemessenen LaserScans in einem Plot."""

    laser_scan_points = calculate_laserScanPoints(data)

    plt.scatter(*berechne_x_y_werte_aus_punkten(laser_scan_points))

    plt.xlim(-data.range_max, data.range_max)
    plt.ylim(-data.range_max, data.range_max)


    # minimaler ScanRadius
    circle1 = plt.Circle((0, 0), data.range_min, color="r")
    fig = plt.gcf()
    ax = fig.gca()
    ax.add_patch(circle1)

    plt.draw()
    plt.pause(0.000001)
    plt.clf()



def calculate_laserScanPoints(data: LaserScan):
    """Berechnet aus den LaserScan Daten die Messpunkte."""
    array_length = int(np.round(data.angle_max / data.angle_increment)) + 1
    if array_length != len(data.ranges):
        rospy.loginfo("ERROR: LaserScan Ausgabe -> test.py")

    laser_points = ()
    current_angle = data.angle_min + angle_offset_lidar
    for i in range(array_length):
        # if data.ranges[i] == float('inf') or data.ranges[i] == float('-inf'):
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


def berechne_x_y_werte_aus_punkten(points):
    x_werte = ()
    y_werte = ()
    for point in points:
        x_werte = (*x_werte, point[0])
        y_werte = (*y_werte, point[1])
    return x_werte, y_werte


def show_laser_scan_on_main_thread():
    while not rospy.is_shutdown():
        scans = rospy.wait_for_message("/scan", LaserScan)
        show_LaserScan(scans)
        # plt.show()


if __name__ == "__main__":
    rospy.init_node("scan_auswerten_test", anonymous=False)

    # sub_LaserScan = rospy.Subscriber("/scan", LaserScan, show_LaserScan)
    # # plt.show()
    # rospy.spin()

    show_laser_scan_on_main_thread()

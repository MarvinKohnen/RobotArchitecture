import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import numpy as np


ANGLE_OFFSET_LIDAR = np.pi / 2
ROBOT_WIDTH = 0.178
AVOID_PUFFER = .022


def calculate_laserScanPoints(data: LaserScan):
    """Berechnet aus den LaserScan Daten die Messpunkte."""
    array_length = int(np.round(data.angle_max / data.angle_increment)) + 1
    if array_length != len(data.ranges):
        rospy.loginfo("ERROR: LaserScan Ausgabe -> test.py")

    laser_points = ()
    current_angle = data.angle_min + ANGLE_OFFSET_LIDAR
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


def distance(p1: tuple, p2: tuple) -> float:
    """Berechnet die Distanz zwischen zwei Punkten."""
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def going_to_crash(laser_scan_points: tuple):
    """Testet ob für alle Punkte vor dem Roboter ein mindest Abstand eingehalten wird."""
    for point in laser_scan_points:
        if point[1] >= 0 and abs(point[0]) < ROBOT_WIDTH + AVOID_PUFFER:
            return True
    return True

def steer_to_clearest_path(laser_scan_points: tuple, pub: rospy.Publisher):
    finde freiesten weg

    überdenke goingt to crash


    rotations_matrix = np.array([[np.cos(rotations_winkel), -np.sin(rotations_winkel)], [
            np.sin(rotations_winkel), np.cos(rotations_winkel)]])



    msg = Twist()
    msg.linear.x = speed
    pub.publish(msg)

def avoid_and_move(data: LaserScan, speed: float):
    laser_scan_points = calculate_laserScanPoints(data)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    if going_to_crash(laser_scan_points):
        steer_to_clearest_path(laser_scan_points, pub)
    else:
        msg = Twist()
        msg.linear.x = speed
        pub.publish(msg)



def initialize():
    """Initialisert diesen Node und erzeugt den dazugehörigen Publisher, welcher die Bewegungsart steuert."""
    rospy.init_node("random_mover", anonymous=False)


    # sub_LaserScan = rospy.Subscriber("/scan", LaserScan, show_LaserScan)
    sub_LaserScan = rospy.Subscriber(
        "/scan", LaserScan, avoid_and_move)

    rospy.spin()


if __name__ == "__main__":
    initialize()

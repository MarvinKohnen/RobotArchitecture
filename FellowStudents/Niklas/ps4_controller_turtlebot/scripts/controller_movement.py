"""Alle Methoden die für das Movement mit dem Controller gebraucht werden."""
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

import numpy as np
import matplotlib.pyplot as plt


speed_factor = 0.4
rotation_factor = 1.2


def change_moving(data: Joy, pub: rospy.Publisher):
    """Controller EIngaben werden zu Bewegungsbefehlen des Roboters."""
    # print(data)
    global stop_boolean

    if data.buttons[0] == 1.0:
        rospy.loginfo("Pause")
        stop_boolean = not stop_boolean

    if stop_boolean:
        pub.publish(Twist())
        return
    

    # R2 and L2 auf [0,1] normieren:
    l2_value = (data.axes[2] +1) /2
    r2_value = (data.axes[5] +1) /2

    
    if r2_value == l2_value == data.axes[0] == 0:
        pub.publish(Twist())
        return
    

    speed = (l2_value - r2_value) * speed_factor
    rotation = data.axes[0] * rotation_factor

    msg = Twist()
    msg.linear.x = speed
    msg.angular.z = rotation
    pub.publish(msg)


def initialize():
    """Initialisert diesen Node und erzeugt den dazugehörigen Publisher, welcher die Bewegungsart steuert."""
    rospy.init_node("ps4_controller_listener", anonymous=False)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    global stop_boolean
    stop_boolean = False

    rospy.Subscriber("joy", Joy, change_moving, callback_args=pub)
    rospy.spin()


if __name__ == "__main__":
    initialize()

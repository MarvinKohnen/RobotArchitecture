from typing import Any
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import itertools

ANGLE_OFFSET_LIDAR = np.pi / 2
ROBOT_WIDTH = 0.178



class Point:
    x: float
    y: float

    def __init__(self, x, y) -> None:
        self.x, self.y = x,y

    def __call__(self, *args: Any, **kwds: Any) -> Any:
        return np.array((self.x,self.y))
    
    def __sub__(self, other):
        if isinstance(other, self.__class__):
            return np.linalg.norm(np.array((self.x,self.y)) - np.array((other.x,other.y)))
            # return np.linalg.norm(self() - other())
        else:
            raise TypeError('Point wrongly subtracted!!')
        
    def __str__(self) -> str:
        return f'({self.x}, {self.y})' 
        
    
    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.x, self.y == other.x, other.y
        else:
            raise TypeError('Point wrongly compared!!')
        
    def __ne__(self, other):
        return not self.__eq__(other)

class Cluster:
    points: list    # list of Points
    center: Point
    approx_radius: float

    def __init__(self, first_point: Point = None) -> None:
        if first_point is not None:
            self.points = [first_point]
            self.center = first_point
            self.approx_radius = 0
            return

        self.points = []
        self.center = None
        self.approx_radius = None

    def check_point_and_add(self, new_point: Point, min_dist = 0.1):
        """Return None if point is not in range for any other point within the min_dist range of this cluster. Adds it otherwise."""
        if float("inf") in new_point() or float("-inf") in new_point():
            return False
        
        if len(self.points) == 0:
            self.points.append(new_point)
            self.center = new_point
            self.approx_radius = 0
            return True
        
        for point in self.points:
            if point - new_point < min_dist and new_point not in self.points:
                self.points.append(new_point)
                self.center = None
                self.approx_radius = None
                return True
        else:
            return False

    def get_center(self):
        if len(self.points) == 0:
            return None

        if self.center is not None:
            return self.center
        x = 0
        y = 0
        for point in self.points:
            x += point.x
            y += point.y
        self.center = Point(x / len(self.points), y / len(self.points))
        return self.center

    def get_approx_radius(self):
        if self.approx_radius is not None:
            return self.approx_radius
        if self.center is None:
            self.get_center() 
        approx_radius = 0
        for point in self.points:
            approx_radius = max(approx_radius, point - self.center)
        self.approx_radius = approx_radius
        return approx_radius

    def __str__(self) -> str:
        return f'Cluster - count: {len(self.points)} - center: {self.center} - radius: {self.approx_radius}'
        


def distance(p1: tuple, p2: tuple) -> float:
    """Berechnet die Distanz zwischen zwei Punkten."""
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


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
                (data.ranges[i] + data.range_min) * np.cos(current_angle),
                (data.ranges[i] + data.range_min) * np.sin(current_angle),
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


# def clustering2(scan_points: tuple, min_point_diff: float):
#     all_clusters = []
#     zähler = 0
#     for point in scan_points:
#         if float("inf") in point or float("-inf") in point:
#             continue
#         zähler += 1
#         for i in range(len(all_clusters)):
#             cluster_found_for_this_point = False
#             for already_selected_point in all_clusters[i]:
#                 distance_between_those_points = distance(already_selected_point, point)
#                 if distance_between_those_points < min_point_diff:
#                     all_clusters[i] = (*all_clusters[i], point)
#                     cluster_found_for_this_point = True
#                     break
#             if cluster_found_for_this_point:
#                 break
#         else:
#             all_clusters.append((point,))
#             # all_clusters = (*all_clusters, (point,))
#     print("zähler", zähler)
#     return all_clusters


def clustering(scan_points: tuple, min_point_diff: float = .1):
    # print(scan_points)
    print('hier')

    all_clusters = [Cluster(),]

    print(all_clusters[0])
    return all_clusters

    i = len(scan_points) -1
    while all_clusters[0].check_point_and_add(Point(*scan_points[i]), min_point_diff):
        print('moin')
        i -= 1
        if i < 0: return all_clusters

    j = 0
    while j < i:
    # for point in scan_points:
        p = Point(*scan_points[j])
        for cluster in all_clusters:
            if not cluster.check_point_and_add(p, min_point_diff):
                all_clusters.append(Cluster(p))
        j+=1
    
    print(len(all_clusters))
    return all_clusters

def cluster_data(msg: LaserScan): 
    clusters = clustering(calculate_laserScanPoints(msg), .1)
    print('Cluster count: ',len(clusters))
    # vis.draw_estimated_obstacles(clusters)




class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ln, = plt.plot([], [], 'b.')
        self.x_data, self.y_data = [] , []

        self.colors = itertools.cycle(["r", "b", "g"])

    def plot_init(self):
        self.ax.set_xlim(-4, 4)
        self.ax.set_ylim(-4, 4)
        return self.ln
    
    def add_circle(self, x, y, radius):
        circle1 = plt.Circle((x, y), radius, color=next(self.colors))
        self.ax.add_patch(circle1)

    def draw_estimated_obstacles(self, all_clusters):
        for cluster in all_clusters:
            # self.ax.add_patch(plt.Circle(cluster.get_center()(), cluster.get_approx_radius(), color=next(self.colors)))
            center_point = cluster.get_center()
            radius = cluster.get_approx_radius()
            if radius is not None and center_point is not None:
                self.add_circle(*center_point(), radius)

    def scan_callback(self, msg: LaserScan):
        self.x_data, self.y_data = berechne_x_y_werte_aus_punkten(calculate_laserScanPoints(msg))

    def update_plot(self, frame):
        # self.x_data = (*self.x_data, 2.5)
        # self.y_data = (*self.y_data, 2.5)
        self.ln.set_data(self.x_data, self.y_data)
        return self.ln


def test(msg: LaserScan):
    print(msg.range_min,msg.range_max)


def test2(msg: LaserScan):
    circle1 = plt.Circle((0, 0), ROBOT_WIDTH, color="r")
    vis.ax.add_patch(circle1)

def start_subscribers():
    sub_LaserScan1 = rospy.Subscriber("/scan", LaserScan, cluster_data)
    # test2(None)
    print('Subscribtion started...')


def test3(msg: LaserScan):
    vis.scan_callback(msg)
    cluster_data(msg)

def visualize_scan():
    global vis
    vis = Visualiser()
    sub = rospy.Subscriber('/scan', LaserScan, vis.scan_callback)
    # sub = rospy.Subscriber('/scan', LaserScan, test3)
    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
    start_subscribers()
    plt.show(block=True)
    print('Plot has been closed!')


def initialize():
    """Initialisert diesen Node und erzeugt den dazugehörigen Publisher, welcher die Bewegungsart steuert."""
    rospy.init_node("obstacle_detector", anonymous=False)

    
    # visualize_scan()
    start_subscribers()
    rospy.spin()  # falls visualize_scan nicht aufgerufen wird, muss dies genutzt werden!




if __name__ == "__main__":
    initialize()
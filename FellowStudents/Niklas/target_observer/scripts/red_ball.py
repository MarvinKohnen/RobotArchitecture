"""Script welches den Roboter ermöglichen soll einen roten Kreis zu entdecken und diesen auf seiner 2D-Karte abbilden. Hier müssen die tiefen/RGB-Kamera die Auflösung (480, 640) haben, nutze hierfür den roslaunch: roslaunch realsense2_camera rs_aligned_depth.launch."""
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import Int8, Bool
from numpy import sin, cos
from geometry_msgs.msg import Point


class Viewer:
    """Klasse um die Bilder der Kamera nach farbigen Bällen zu analysieren."""

    bridge: CvBridge
    color_Spectrum: tuple
    time_synchronizer: ApproximateTimeSynchronizer
    circle_not_detected_counter: int
    point_publisher: rospy.Publisher

    def __init__(self, collection: tuple, point_publisher: rospy.Publisher) -> None:
        """Initialisiert den Viewer."""
        self.bridge = CvBridge()
        self.color_Spectrum = collection
        self.point_publisher = point_publisher
        self.circle_not_detected_counter = 0
        self._start_Pics_Subscribers()

    def _start_Pics_Subscribers(self) -> None:
        """Liest die Bild-Topic aus, und ruft die Mehoden zur Verabeitung auf."""
        rgb_View = Subscriber("/camera/color/image_raw", Image)
        depth_View = Subscriber("/camera/depth/image_rect_raw", Image)
        self.time_synchronizer = ApproximateTimeSynchronizer(
            [rgb_View, depth_View], queue_size=5, slop=0.1
        )
        self.time_synchronizer.registerCallback(self._procces_Pics)

    def _procces_Pics(self, rgb_Image: Image, depth_Image: Image) -> None:
        """Verabreitet die uebergebenen Bilder."""

        print('RGB ====',rgb_Image.header)
        print('DEPTH ====',depth_Image.header)

        pic_with_filter = self._get_Pics_with_Filters(rgb_Image)
        x, y, circle_count = self._detect_circles_using_Contours(pic_with_filter)
        if circle_count != 1:
            rospy.loginfo(f"Kreis nicht eindeutig erkannt ({circle_count})")
            self.circle_not_detected_counter += 1
            if self.circle_not_detected_counter >= 10:
                self.point_publisher.publish(None)
        else:
            self.circle_not_detected_counter = 0
            depth = self._get_depth_at(depth_Image, x, y)
            target_coords = self.determine_3d_coords_of_taget(x, y, depth)
            self.point_publisher.publish(target_coords)

    def _get_depth_at(self, depth_Image, x: int, y: int):
        """Berechnet die Entfernung zu einem gegebenen Punkt."""
        image = self.bridge.imgmsg_to_cv2(depth_Image, desired_encoding="passthrough")
        return image[y - 1][x - 1]

    def _get_Pics_with_Filters(self, image_message: Image):
        """Erzeugt eine Kopie die nur die Farben des color_Spectrum des Bildes zeigt."""
        image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")

        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        image_mask = None
        for boundaries in self.color_Spectrum:
            image_temp_mask = cv2.inRange(image_hsv, boundaries[0], boundaries[1])
            if image_mask is None:
                image_mask = image_temp_mask
            else:
                image_mask = cv2.bitwise_or(image_mask, image_temp_mask)

        if image_mask is None:
            return image

        # return cv2.bitwise_and(image, image, mask=image_mask)

        temp = cv2.bitwise_and(image, image, mask=image_mask)
        # cv2.imshow("Analysieres Bild", temp)
        # cv2.waitKey(3)
        return temp

    def _detect_circles_using_Contours(self, image_cv):
        """Untersucht das Bild nach Kreisen und gibt die Position des Mittelpunktes des erst besten Kreises zurück."""
        # Load image, grayscale, median blur, Otsus threshold
        gray = cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 11)
        thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

        # Morph open
        # https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=3)

        # Find contours and filter using contour area and aspect ratio
        # https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html
        cnts = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        for c in cnts:
            peri = cv2.arcLength(c, True)  # Umfang
            # implementation of Douglas-Peucker algorithm
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            area = cv2.contourArea(c)  # Fläche
            if len(approx) > 5 and area > 1000 and area < 500000:
                ((x, y), r) = cv2.minEnclosingCircle(c)
                cv2.circle(image_cv, (int(x), int(y)), int(r), (36, 255, 12), 2)

                # cv2.imshow('Kreise entdeckt:', image_cv)
                # cv2.waitKey(3)

                return int(x), int(y), 1

        # cv2.imshow('thresh', thresh)
        # cv2.imshow('opening', opening)
        # cv2.imshow('Kreise entdeckt:', image_cv)
        # cv2.waitKey(3)

        return -1, -1, 0

    def determine_3d_coords_of_taget(pixel_x: float, pixel_y: float, depth: float):
        """Bestimmt anhand der Position des Punktes und dessen Tiefe eine 2D Repräsentation auf der Karte zum Roboter."""
        FOV_HORIZONTAL = 75  # FielofView in Grad der Kamera
        FOV_VERTICAL = 62  # FielofView in Grad der Kamera
        # https://www.intelrealsense.com/wp-content/uploads/2022/11/Intel-RealSense-D400-Series-Datasheet-November-2022.pdf#page=65&zoom=100,184,370

        horizontal_angle_degrees = pixel_x * FOV_HORIZONTAL / 640 - FOV_HORIZONTAL / 2
        vertical_angle_degrees = pixel_y * FOV_VERTICAL / 480 - FOV_VERTICAL / 2

        horizontal_angle = np.deg2rad(horizontal_angle_degrees)
        vertical_angle = np.deg2rad(vertical_angle_degrees)
        # vertical_angle = np.deg2rad(180 - vertical_angle_degrees) #TODO

        # Kugelkoordinaten -> euklidischen Koord
        return (
            depth * sin(horizontal_angle) * cos(vertical_angle),
            depth * sin(horizontal_angle) * sin(vertical_angle),
            depth * cos(horizontal_angle),
        )


if __name__ == "__main__":
    print("loading...")
    rospy.init_node("red_ball", anonymous=False)

    # erster Entwurf des Farbspektrums:
    # lower_red1 = np.array((0, 50, 0))
    # upper_red1 = np.array((10, 255, 255))
    # lower_red2 = np.array((170, 50, 50))
    # upper_red2 = np.array((180, 255, 255))

    # Farbspektrum zum untersuchen:
    lower_red1 = np.array((0, 125, 50))
    upper_red1 = np.array((7, 200, 250))
    lower_red2 = np.array((175, 50, 50))
    upper_red2 = np.array((180, 255, 255))
    color_Filter = ((lower_red1, upper_red1), (lower_red2, upper_red2))

    Viewer(
        collection=color_Filter,
        point_publisher=rospy.Publisher("/target_observer/point", Point, queue_size=1),
    )
    rospy.spin()

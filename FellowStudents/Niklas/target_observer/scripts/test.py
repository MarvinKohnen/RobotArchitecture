import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np


def find_circle_in_depth(depth_Image: Image):
    bridge = CvBridge()
    depth_img_cv2 = bridge.imgmsg_to_cv2(depth_Image, desired_encoding="passthrough")
    depth_rgb = cv2.cvtColor(depth_img_cv2, cv2.COLOR_GRAY2BGR)
    detect_circles_using_Contours(depth_rgb)


def test(depth_Image: Image):
    bridge = CvBridge()
    # image_cv = bridge.imgmsg_to_cv2(depth_Image, desired_encoding="passthrough")
    image_cv = bridge.imgmsg_to_cv2(depth_Image, desired_encoding="16UC1")
    img = cv2.cvtColor(image_cv, cv2.COLOR_GRAY2BGR)
    cv2.imshow('Farbbild:', img)
    cv2.waitKey(3)


def test2(depth_Image: Image):
    bridge = CvBridge()
    gray = bridge.imgmsg_to_cv2(depth_Image, desired_encoding="8UC1")
    image_cv = bridge.imgmsg_to_cv2(depth_Image, desired_encoding="passthrough")

    blur = cv2.medianBlur(gray, 11)
    thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=3)


    cnts = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    for c in cnts:
        peri = cv2.arcLength(c, True)  # Umfang
        # implementation of Douglas-Peucker algorithm
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        area = cv2.contourArea(c)  # Fläche
        if len(approx) > 5 and area > 1000 and area < 500000 :#or len(cnts) == 1:
            ((x, y), r) = cv2.minEnclosingCircle(c)
            # cv2.circle(image_cv, (int(x), int(y)), int(r), (36, 255, 12), 2)
            cv2.circle(image_cv, (int(x), int(y)), int(r), 1000000, 2)

            cv2.imshow('Kreise entdeckt:', image_cv)
            cv2.waitKey(3)
            print(int(x), int(y), 1)
            return int(x), int(y), 1

    # cv2.imshow('thresh', thresh)
    # cv2.imshow('opening', opening)
    # cv2.imshow('Kreise entdeckt:', image_cv)
    # cv2.waitKey(3)
    print('nicht eindeutig', len(cnts))
    # cv2.imshow('Kreise entdeckt:', image_cv)
    # cv2.waitKey(3)
    return -1, -1, 0

def detect_circles_using_Contours(image_cv):
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

                cv2.imshow('Kreise entdeckt:', image_cv)
                cv2.waitKey(3)

                return int(x), int(y), 1

        # cv2.imshow('thresh', thresh)
        # cv2.imshow('opening', opening)
        # cv2.imshow('Kreise entdeckt:', image_cv)
        # cv2.waitKey(3)

        return -1, -1, 0


if __name__ == '__main__':
    rospy.init_node('searchView_aligned_pics', anonymous=False)

    # rospy.Subscriber("/camera/depth/image_raw", Image, find_circle_in_depth)
    rospy.Subscriber("/camera/depth/image_raw", Image, test2)
    
    rospy.spin()
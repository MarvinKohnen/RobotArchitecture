import cv2
from cv_bridge import CvBridge
import numpy as np

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


def get_Pics_with_Filters(image):
    """Erzeugt eine Kopie die nur die Farben des color_Spectrum des Bildes zeigt."""
    # image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")

    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    image_mask = None
    for boundaries in color_Filter:
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

            # cv2.imshow('Kreise entdeckt:', image_cv)
            # cv2.waitKey(3)

            return image_cv
    return image_cv


def test(image_cv):
    pic_only_red = get_Pics_with_Filters(image_cv)
    global previous_frame
    d = cv2.absdiff(previous_frame, pic_only_red)
    previous_frame = pic_only_red

    gray = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, th = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
    dilated = cv2.dilate(th, np.ones((3, 3), np.uint8), iterations=3)
    (c, _) = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # cv2.drawContours(image_cv, c, -1, (0, 255, 0), 2)

    for contour in c:
        if cv2.contourArea(contour) < 1500:
            continue
        (x, y, w, h) = cv2.boundingRect(contour)

        # cv2.circle(image_cv, (int(x), int(y)), int(w), (36, 255, 12), 2)
        cv2.rectangle(image_cv, (x, y), (x + w, y + h), (0, 255, 0), 3)
    return image_cv


def test2(image_cv):
    pic_only_red = get_Pics_with_Filters(image_cv)
    global previous_frame
    d = cv2.absdiff(previous_frame, pic_only_red)
    # d = image_cv

    gray = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, th = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
    dilated = cv2.dilate(th, np.ones((3, 3), np.uint8), iterations=3)
    (cnts, _) = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # cv2.drawContours(image_cv, cnts, -1, (0, 255, 0), 2)
    previous_frame = pic_only_red

    for c in cnts:
        peri = cv2.arcLength(c, True)  # Umfang
        # implementation of Douglas-Peucker algorithm
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        area = cv2.contourArea(c)  # Fläche
        if len(approx) > 4 and area > 1000 and area < 500000:
            ((x, y), r) = cv2.minEnclosingCircle(c)
            cv2.circle(image_cv, (int(x), int(y)), int(r), (36, 255, 12), 2)

            # cv2.imshow('Kreise entdeckt:', image_cv)
            # cv2.waitKey(3)

            # return image_cv
    return image_cv


def test3(image_cv):
    pic_only_red = get_Pics_with_Filters(image_cv)
    global previous_frame
    d = cv2.absdiff(previous_frame, pic_only_red)
    previous_frame = pic_only_red
    # d = pic_only_red

    gray = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, th = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
    dilated = cv2.dilate(th, np.ones((3, 3), np.uint8), iterations=3)
    # (contours, _) = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    (contours, _) = cv2.findContours(
        dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    # cv2.drawContours(image_cv, c, -1, (0, 255, 0), 2)

    for c in contours:
        if cv2.contourArea(c) < 1500:
            continue

        peri = cv2.arcLength(c, True)  # Umfang
        # implementation of Douglas-Peucker algorithm
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        area = cv2.contourArea(c)  # Fläche
        if len(approx) > 5:
            ((x, y), r) = cv2.minEnclosingCircle(c)
            cv2.circle(image_cv, (int(x), int(y)), int(r), (36, 255, 12), 2)
    return image_cv


def get_exactly_one_red_circle(image_cv):
    pic_only_red = get_Pics_with_Filters(image_cv)

    gray = cv2.cvtColor(pic_only_red, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, th = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
    dilated = cv2.dilate(th, np.ones((3, 3), np.uint8), iterations=3)
    # (contours, _) = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    (contours, _) = cv2.findContours(
        dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    circles = []
    for c in contours:
        if cv2.contourArea(c) < 1500:
            continue

        peri = cv2.arcLength(c, True)  # Umfang
        # implementation of Douglas-Peucker algorithm
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        area = cv2.contourArea(c)  # Fläche
        if len(approx) > 5:
            ((x, y), r) = cv2.minEnclosingCircle(c)
            circles.append(np.array([[x], [y]]))

    if len(circles) != 1:
        print(len(circles))
        return None
    return circles[0]


def initialise_kalman_filter():
    global kalman_filter
    kalman_filter = KalmanFilter(dim_x=4, dim_z=2)

    kalman_filter.x = np.array([0.0, 0.0, 0.0, 0.0])  # initial state

    kalman_filter.F = np.array(
        [
            [1.0, 0.1, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.1],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    kalman_filter.P *= 1000.0

    kalman_filter.R = 5

    kalman_filter.Q = Q_discrete_white_noise(dim=4, dt=0.1, var=0.13)


def predict_circle_movement(image_cv):
    sensor_data = get_exactly_one_red_circle(image_cv)

    global kalman_filter
    kalman_filter.predict()
    if sensor_data is not None:
        kalman_filter.update(sensor_data)
        print(sensor_data)

    print(kalman_filter.x)
    (new_x, new_y, vel_x, vel_y) = kalman_filter.x
    cv2.circle(image_cv, (int(new_x), int(new_y)), int(3), (36, 255, 12), 2)

    return image_cv


if __name__ == "__main__":
    print("loading...")

    # Farbspektrum zum untersuchen:
    # lower_red1 = np.array((0, 125, 50))
    # upper_red1 = np.array((7, 200, 250))
    # lower_red2 = np.array((175, 50, 50))
    # upper_red2 = np.array((180, 255, 255))

    lower_red1 = np.array((0, 150, 50))
    upper_red1 = np.array((10, 255, 255))
    lower_red2 = np.array((170, 150, 50))
    upper_red2 = np.array((180, 255, 255))

    redBGR = np.uint8([[[0, 0, 255]]])
    hsv_red = cv2.cvtColor(redBGR, cv2.COLOR_BGR2HSV)
    print("Red in HSV: ", hsv_red)  # => [[[  0 255 255]]]

    color_Filter = ((lower_red1, upper_red1), (lower_red2, upper_red2))

    initialise_kalman_filter()

    vid = cv2.VideoCapture(0)
    global previous_frame
    _, previous_frame = vid.read()
    while True:
        _, frame = vid.read()

        image = frame.copy()

        # cv2.imshow('frame', np.hstack((frame, find_red_circle(frame), test(frame))))
        # cv2.imshow('Original Bild', image)
        # cv2.imshow('Roter Kreis', find_red_circle(image))
        # cv2.imshow('Bild nur in Rot', get_Pics_with_Filters(image.copy()))

        # cv2.imshow('Test', test3(image.copy()))

        cv2.imshow("Test", predict_circle_movement(image.copy()))

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

import rospy
import cv2
import numpy as np

import time

# 640x480


def get_pic():
    """Liest einen Frame der Kamera und gibt dieses Bild zurück."""
    vid = cv2.VideoCapture(0)
    ret, frame = vid.read()
    vid.release()
    return frame


def find_people(image):
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # rects, weights = hog.detectMultiScale(img_gray, winStride=(2, 2), padding=(10, 10), scale=1.02) # slow

    # rects, weights = hog.detectMultiScale(img_gray, padding=(4, 4), scale=1.02) # fast1

    rects, weights = hog.detectMultiScale(img_gray, winStride=(8, 8))  # fast2

    for i, (x, y, w, h) in enumerate(rects):
        if weights[i] < 0.13:
            continue
        elif weights[i] < 0.3 and weights[i] > 0.13:
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        if weights[i] < 0.7 and weights[i] > 0.3:
            cv2.rectangle(image, (x, y), (x + w, y + h), (50, 122, 255), 2)
        if weights[i] > 0.7:
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    cv2.startWindowThread()
    cv2.putText(
        image,
        "High confidence",
        (10, 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
    )
    cv2.putText(
        image,
        "Moderate confidence",
        (10, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (50, 122, 255),
        2,
    )
    cv2.putText(
        image, "Low confidence", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
    )
    cv2.imshow("HOG detection", image)
    cv2.waitKey(0)


def find_people2(image):
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # img_gray = cv2.threshold(img_gray,80,255,cv2.THRESH_BINARY)

    # rects, weights = hog.detectMultiScale(img_gray, winStride=(2, 2), padding=(10, 10), scale=1.02) # slow
    # rects, weights = hog.detectMultiScale(img_gray, padding=(4, 4), scale=1.02) # fast1
    # rects, weights = hog.detectMultiScale(img_gray, winStride=(8, 8))  # fast2
    (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4), padding=(4, 4), scale=1.05) # slow2

    for i, (x, y, w, h) in enumerate(rects):
        if weights[i] < 0.13:
            continue
        elif weights[i] < 0.3 and weights[i] > 0.13:
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        if weights[i] < 0.7 and weights[i] > 0.3:
            cv2.rectangle(image, (x, y), (x + w, y + h), (50, 122, 255), 2)
        if weights[i] > 0.7:
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    cv2.putText(
        image,
        "High confidence",
        (10, 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
    )
    cv2.putText(
        image,
        "Moderate confidence",
        (10, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (50, 122, 255),
        2,
    )
    cv2.putText(
        image, "Low confidence", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
    )

    return image


if __name__ == "__main__":
    cv2.startWindowThread()
    cap = cv2.VideoCapture(0)

    while True:
        # reading the frame
        ret, frame = cap.read()

        # turn to greyscale:
        # frame = cv2.resize(frame, (640, 480))
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # apply threshold. all pixels with a level larger than 80 are shown in white. the others are shown in black:
        # ret,frame = cv2.threshold(frame,80,255,cv2.THRESH_BINARY)
        # displaying the frame
        # cv2.imshow('frame',frame)

        cv2.imshow("Erkannte Personen", find_people2(frame))

        if cv2.waitKey(1) & 0xFF == ord("q"):
            #     # breaking the loop if the user types q
            #     # note that the video window must be highlighted!
            break

    cap.release()
    cv2.destroyAllWindows()
    # the following is necessary on the mac,
    # maybe not on other platforms:
    cv2.waitKey(1)

    # while True:
    #     find_people(get_pic())

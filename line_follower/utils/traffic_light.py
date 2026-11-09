import cv2
import numpy as np


def traffic_light(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Only a lit light is bright and saturated; an unlit one stays dark (V under 50).
    # Red sits at both ends of OpenCV's hue scale (0-180), so it takes two ranges.
    red = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255)) | cv2.inRange(hsv, (170, 100, 100), (180, 255, 255))
    green = cv2.inRange(hsv, (45, 100, 100), (95, 255, 255))

    mask = cv2.GaussianBlur(red | green, (5, 5), 0)
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20,
                               param1=100, param2=15, minRadius=5, maxRadius=40)
    if circles is None:
        return None

    x, y, r = np.round(circles[0][0]).astype(int)
    inside = np.zeros(mask.shape, np.uint8)
    cv2.circle(inside, (x, y), r, 255, -1)

    if cv2.countNonZero(green & inside) > cv2.countNonZero(red & inside):
        return 'green'
    return 'red'

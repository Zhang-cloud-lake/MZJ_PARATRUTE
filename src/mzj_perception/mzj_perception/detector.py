"""A clearly labeled demo detector, independent from ROS transport."""
import cv2
import numpy as np


def detect_marker(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([40, 100, 80]), np.array([85, 255, 255]))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections = []
    for contour in contours:
        if cv2.contourArea(contour) < 100:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        detections.append(("demo_green_marker", 1.0, x + width/2,
                           y + height/2, float(width), float(height)))
    return detections

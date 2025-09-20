import cv2
import numpy as np

def detect_shapes(mask, min_area=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    shapes = []
    for contour in contours:
        if cv2.contourArea(contour) > min_area:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                shapes.append(((cx, cy), contour))
    return shapes

def draw_shapes(img, shapes):
    for center, contour in shapes:
        cv2.drawContours(img, [contour], -1, (0, 0, 255), 2)
        cv2.circle(img, center, 3, (255, 0, 0), -1)
    return img



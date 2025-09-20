import cv2
import numpy as np
from .utils import detect_shapes, draw_shapes

def process_static_image(input_path, output_path):
    img = cv2.imread(input_path)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 40, 40])
    upper_green = np.array([85, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.bitwise_not(mask)
    shapes = detect_shapes(mask, min_area=500)
    output_img = img.copy()
    draw_shapes(output_img, shapes)
    cv2.imwrite(output_path, output_img)
    return shapes
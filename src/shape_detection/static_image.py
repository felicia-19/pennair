import cv2
import numpy as np
from .utils import detect_shapes, draw_shapes
import os

def process_static_image(input_path, output_path):
    img = cv2.imread(input_path)
    if img is None:
        return []
    else:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_green = np.array([25, 40, 40])  
        upper_green = np.array([95, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        mask = cv2.bitwise_not(mask)
        shapes = detect_shapes(mask, min_area=200) 
        output_img = img.copy()
        draw_shapes(output_img, shapes)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 2
        color = (255, 255, 255)  
        for center, _ in shapes:
            x, y = center
            text = f"({x:.0f}, {y:.0f})"
            cv2.putText(output_img, text, (int(x) + 5, int(y) - 5), font, font_scale, (0, 0, 0), thickness + 2)
            cv2.putText(output_img, text, (int(x) + 5, int(y) - 5), font, font_scale, color, thickness)
            
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        cv2.imwrite(output_path, output_img)
        return shapes
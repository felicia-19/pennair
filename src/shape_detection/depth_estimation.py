import cv2
import numpy as np
from .utils import detect_shapes, draw_shapes
import os
import csv

def estimate_3d_centers(input_path, output_image_path, output_csv_path):
    K = np.array([[2564.3186869, 0, 0],
                  [0, 2569.70273111, 0],
                  [0, 0, 1]])
    focal_length = 2564.3186869
    real_radius = 10.0
    img = cv2.imread(input_path)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_green = np.array([25, 40, 40])
    upper_green = np.array([95, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.bitwise_not(mask)
    shapes = detect_shapes(mask, min_area=500)
    output_img = img.copy()
    draw_shapes(output_img, shapes)
    Z = 1.0

    for center, contour in shapes:
        ellipse = cv2.fitEllipse(contour)
        (center_ellipse, axes, angle) = ellipse
        pixel_radius = min(axes) / 2
        if pixel_radius > 10:
            Z = (focal_length * real_radius) / pixel_radius
            break

    K_inv = np.linalg.inv(K)
    with open(output_csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['shape_id', 'x', 'y', 'z'])
        for i, (center, _) in enumerate(shapes):
            u, v = center
            pixel = np.array([u, v, 1])
            camera_coords = Z * K_inv @ pixel
            x, y, z = camera_coords
            writer.writerow([i, x, y, z])
            text = f"({x:.1f}, {y:.1f}, {z:.1f})"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 2
            color = (255, 255, 255)
            cv2.putText(output_img, text, (int(u) + 5, int(v) - 5), font, font_scale, (0, 0, 0), thickness + 2)
            cv2.putText(output_img, text, (int(u) + 5, int(v) - 5), font, font_scale, color, thickness)

    os.makedirs(os.path.dirname(output_image_path), exist_ok=True)
    cv2.imwrite(output_image_path, output_img)
    return len(shapes)

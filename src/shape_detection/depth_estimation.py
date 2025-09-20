import cv2
import numpy as np
from .utils import detect_shapes, draw_shapes

def estimate_3d_centers(input_path, output_image_path, output_csv_path):
    K = np.array([[2564.3186869, 0, 0],
                  [0, 2569.70273111, 0],
                  [0, 0, 1]])
    R = 10.0 

    img = cv2.imread(input_path)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 40, 40])
    upper_green = np.array([85, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.bitwise_not(mask)
    shapes = detect_shapes(mask, min_area=500)
    with open(output_csv_path, 'w') as f:
        f.write("shape_id,x,y,z\n")
        for i, (center, _) in enumerate(shapes):
            u, v = center
            Z = 1.0
            K_inv = np.linalg.inv(K)
            pixel = np.array([u, v, 1])
            camera_coords = Z * K_inv @ pixel
            x, y, _ = camera_coords
            f.write(f"{i},{x:.2f},{y:.2f},{Z:.2f}\n")
            
    output_img = img.copy()
    draw_shapes(output_img, shapes)
    cv2.imwrite(output_image_path, output_img)
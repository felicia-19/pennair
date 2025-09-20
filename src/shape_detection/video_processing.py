import cv2
import numpy as np
from .utils import detect_shapes, draw_shapes

def process_video(input_path, output_path):
    cap = cv2.VideoCapture(input_path)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    frame_count = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 40, 40])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        mask = cv2.bitwise_not(mask)
        shapes = detect_shapes(mask, min_area=500)
        output_frame = frame.copy()
        draw_shapes(output_frame, shapes)

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 2
        color = (255, 255, 255)  
        for center, _ in shapes:
            x, y = center
            text = f"({x:.0f}, {y:.0f})"
            cv2.putText(output_frame, text, (int(x) + 5, int(y) - 5), font, font_scale, (0, 0, 0), thickness + 2)
            cv2.putText(output_frame, text, (int(x) + 5, int(y) - 5), font, font_scale, color, thickness)

        out.write(output_frame)
        frame_count += 1

    cap.release()
    out.release()
    return frame_count


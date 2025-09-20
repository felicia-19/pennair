import cv2
import numpy as np
from .utils import detect_shapes, draw_shapes
import os

def process_agnostic_video(input_path, output_path):
    cap = cv2.VideoCapture(input_path)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    frame_count = 0
    background_subtractor = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=50, detectShadows=True)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        fg_mask = background_subtractor.apply(frame)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l_channel = lab[:,:,0]
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        enhanced = clahe.apply(l_channel)
        blurred = cv2.GaussianBlur(enhanced, (15, 15), 0)
        edges = cv2.Canny(blurred, 30, 100)
        combined_mask = cv2.bitwise_or(fg_mask, edges)
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        opened = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel_open)
        processed_mask = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel_close)
        contours, _ = cv2.findContours(processed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_shapes = []
        min_area = 1000  
        max_area = width * height * 0.1  
        min_solidity = 0.3  
        min_extent = 0.2  
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area or area > max_area:
                continue
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            x, y, w, h = cv2.boundingRect(contour)
            bounding_area = w * h
            extent = area / bounding_area if bounding_area > 0 else 0
            aspect_ratio = w / h if h > 0 else 0
            if aspect_ratio > 5 or aspect_ratio < 0.2:  
                continue

            if solidity > min_solidity and extent > min_extent:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    filtered_shapes.append(((cx, cy), contour))
        output_frame = frame.copy()
        for center, contour in filtered_shapes:
            cv2.drawContours(output_frame, [contour], -1, (0, 255, 0), 2)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        color = (255, 255, 255)
        
        for center, _ in filtered_shapes:
            x, y = center
            cv2.circle(output_frame, (x, y), 5, (0, 0, 255), -1)
            text = f"({x}, {y})"
            cv2.putText(output_frame, text, (int(x) + 10, int(y) - 10), font, font_scale, (0, 0, 0), thickness + 1)
            cv2.putText(output_frame, text, (int(x) + 10, int(y) - 10), font, font_scale, color, thickness)
        
        out.write(output_frame)
        frame_count += 1

    cap.release()
    out.release()
    return frame_count
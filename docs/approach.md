# Technical Approach
### Part 1: Static Image

Segmentation: Convert to HSV, threshold to isolate shapes from green background.
Detection: Use contour detection with area filtering.
Visualization: Draw contours and centroids on the original image.

### Part 2: Video

Extended Part 1 to process frames sequentially, treating the video as a stream.
Optimized for real-time performance by reusing utility functions.

### Part 3: Background-Agnostic

Replaced HSV thresholding with adaptive thresholding and morphological operations.
Ensured robustness across varied backgrounds.

### Part 4: 3D Estimation

Used camera intrinsic matrix to back-project 2D points to 3D.
Assumed a flat plane due to lack of depth data.

### Part 5: ROS2

Created a video publisher node to stream frames.
Created a detector node to process frames and publish detections.
Used a custom message for shape centers.

### Part 6: Enhancement

Added basic occlusion handling by tracking shapes across frames using centroid proximity.

#### Challenges

Tuning thresholds for robust segmentation.
Handling occlusions and varying lighting conditions.
Simplifying 3D estimation due to limited depth information.

#### Optimizations

Used efficient contour filtering to reduce computation.
Leveraged OpenCV’s optimized functions for real-time video processing.

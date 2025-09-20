# PennAir 2025 Shape Detection
This repository implements a shape detection system for the PennAir 2025 Application Technical Challenge, detecting solid shapes in images and videos, tracing their outlines, locating their centers, and extending to 3D and ROS2 integration.

## Setup Instructions

Clone the repository:[git clone https://github.com/felicia-19/pennair.git](https://github.com/felicia-19/pennair.git)

cd pennair_2024_shape_detection


Install dependencies:pip install -r requirements.txt
Note: python=3.10.13

For ROS2 (Part 5), ensure ROS2 Humble is installed and build the package:colcon build --packages-select pennair_2024_shape_detection
source install/setup.bash

## Running the Code

#### Part 1: Static Image
python src/scripts/run_static.py

Output: results/static/processed_image.png

#### Part 2: Video
python src/scripts/run_video.py

Output: results/video/processed_video.mp4

#### Part 3: Background-Agnostic
python src/scripts/run_agnostic.py

Output: results/agnostic/processed_video_hard.mp4

#### Part 4: 3D Estimation
python src/scripts/run_3d.py

Output: results/3d/processed_image.png, results/3d/3d_results.csv

#### Part 5: ROS2
ros2 launch pennair_2024_shape_detection shape_detection.launch.py

## Results

Static Image: *to be added later*
Video: Processed Video *to be added later*
Background-Agnostic: Processed Hard Video *to be added later*
3D Estimation: See results/3d/3d_results.csv *figure out how to showcase small snippet here*

## File Structure

src/: Source code for shape detection and ROS2 nodes.
data/: Input image and videos.
results/: Processed outputs and reports.
docs/: Detailed approach and references.
launch/: ROS2 launch file.

## Notes

Assumes input files are in data/.
ROS2 nodes require a custom message (detection_msg.msg).
Basic occlusion handling included for Part 6.

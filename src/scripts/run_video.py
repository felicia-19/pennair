import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from shape_detection.video_processing import process_video

def main():
    input_path = "data/PennAir_2024_App_Dynamic.mp4"
    output_path = "results/video/processed_video.mp4"
    process_video(input_path, output_path)
    print("Video processing completed")

if __name__ == "__main__":
    main()

    
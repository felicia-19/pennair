import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from shape_detection.agnostic_detection import process_agnostic_video

def main():
    input_path = "data/PennAir_2024_App_Dynamic_Hard.mp4"
    output_path = "results/agnostic/processed_video_hard.mp4"
    process_agnostic_video(input_path, output_path)
    print("Agnostic video processing completed")

if __name__ == "__main__":
    main()

    
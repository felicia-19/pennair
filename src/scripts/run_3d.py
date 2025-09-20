import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from shape_detection.depth_estimation import estimate_3d_centers

def main():
    input_path = "data/PennAir_2024_App_Static.png"
    output_image_path = "results/3d/processed_image.png"
    output_csv_path = "results/3d/3d_results.csv"
    estimate_3d_centers(input_path, output_image_path, output_csv_path)
    print("3D estimation completed")

if __name__ == "__main__":
    main()

    
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from shape_detection.static_image import process_static_image

def main():
    print(f"sys.path: {sys.path}")
    print(f"Current working directory: {os.getcwd()}")
    input_path = "data/PennAir_2024_App_Static.png"
    output_path = "results/static/processed_image.png"
    shapes = process_static_image(input_path, output_path)
    print(f"Detected {len(shapes)} shapes in static image")

if __name__ == "__main__":
    main()

    
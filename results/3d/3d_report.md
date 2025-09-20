# 3D Estimation Report
### Approach

Used the provided camera intrinsic matrix to back-project 2D centers to 3D.
Assumed a flat plane at Z=1 for simplicity (real depth estimation would require additional data).
Saved 3D coordinates to a CSV file.

### Challenges

Lack of depth information limited accuracy; assumed constant Z.
Circle radius (10 inches) was not directly used due to 2D-to-3D ambiguity.

### Performance

Generated plausible 3D coordinates for shape centers.
Output CSV and visualized image confirm correct processing.

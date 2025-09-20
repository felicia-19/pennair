# Static Image Processing Report
### Approach

Used HSV color space to segment shapes from the grassy background.
Applied contour detection to identify shapes, filtering by area to remove noise.
Computed centroids for each contour to mark centers.

### Challenges

Green background segmentation required tuning HSV thresholds.
Small noise contours were filtered using a minimum area threshold.

### Performance

Successfully detected and outlined shapes in the static image.
Centers marked accurately based on contour moments.

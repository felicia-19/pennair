# Background-Agnostic Processing Report
### Approach

Replaced HSV segmentation with adaptive thresholding for background-agnostic detection.
Used morphological operations to clean up the thresholded image.
Applied the same contour detection as previous parts.

### Challenges

Handling diverse backgrounds required robust preprocessing.
Adaptive thresholding parameters were tuned for stability.

### Performance

Successfully detected shapes in the hard video with varied backgrounds.
Maintained accuracy with minimal false positives.

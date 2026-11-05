# The single most-confident person in one camera frame.
# Published on /perception/detection only when a person is detected.

std_msgs/Header header  # stamp = camera capture time, frame_id = camera frame (copied from the input Image)

float32 confidence      # model score for the person class, 0.0-1.0

# Bounding box in source-image pixels (0,0 = top-left of the camera frame)
float32 x_min
float32 y_min
float32 x_max
float32 y_max

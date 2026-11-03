# Published by perception ONLY when something is detected.
std_msgs/Header header        # stamp = camera frame capture time
float64 confidence
float64 x                     # bounding box center, pixels
float64 y
float64 width                 # pixels
float64 height
uint32 image_width
uint32 image_height

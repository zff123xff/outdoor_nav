# Original camera header, including acquisition time and optical frame.
std_msgs/Header header
uint32 image_width
uint32 image_height
# Empty for a successfully processed frame without objects.
Detection[] detections

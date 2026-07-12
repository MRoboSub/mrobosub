# One detected instance from the segmentation model.
# Mirrors Detection.msg (left/top/right/bottom -> polygon).

geometry_msgs/Point32[] polygon   # instance mask boundary, in pixel coordinates (x, y) of the source image
float32 confidence
int8 classification

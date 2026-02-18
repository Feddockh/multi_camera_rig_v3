# One instance detection with optional segmentation mask (cropped to bbox).

std_msgs/Header header

# Bounding box in the SAME pixel coordinate frame as the image used downstream.
vision_msgs/BoundingBox2D bbox

# Class and confidence
string class_id
float32 score

# Optional binary mask cropped to the bbox region.
# If mask_data is empty => no mask provided (downstream should fall back to bbox fill)
uint32 mask_width
uint32 mask_height

# mono8, row-major, size = mask_width * mask_height
uint8[] mask_data

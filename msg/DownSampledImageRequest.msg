# Enum for available send modes

# Republish all images as downsampled ones
uint8 ALL = 0
# Republish with frequency as set by publish_frequency
uint8 PUBLISH_FREQ = 1
# Republish only once on receiving the message
uint8 ONCE  = 2

# Mode for the image provider should use
uint8 mode

# Only used when mode is equal to PUBLISH_FREQ
float64 publish_frequency

# The region of interest data
# Contains:
# x and y offset
# width and height
sensor_msgs/RegionOfInterest roi

# Binning data (How many pixels should be fused into one, should be power of 2 if possible)
uint32 binning_x
uint32 binning_y


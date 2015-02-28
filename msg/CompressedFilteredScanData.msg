# This message provides per scan mutable data
# in in minimal and lossy discretized form

uint16 INVALID = 65536


# Scan data discretized to millimeters.
# The following conventions apply:
# 0 < value < max_range: valid scan point
# max_range < value < 2*max_range: self filtered scan point
# INVALID: Invalid scan point
uint16[] scan

# Intensities scales in reasonable TBD way
uint8[] intensities

# The pose of the sensor in fixed frame when
# first ray was retrieved 
geometry_msgs/Transform transform_first_ray

# The pose of the sensor in fixed frame when
# last ray was retrieved 
geometry_msgs/Transform transform_last_ray

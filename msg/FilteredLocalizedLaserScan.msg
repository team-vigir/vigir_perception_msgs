# This message provides a scan in a minimal configuration
# that allows high fidelity projection while at the same
# time conserving the compact representation it has when
# coming from the sensor. Also provides self filter
# information

# Types of scans
uint8 SCAN_PREPROCESSED  = 0
uint8 SCAN_SELF_FILTERED = 1
uint8 SCAN_RAW           = 2

# Header
# stamp: Time of first ray in scan
# frame_id: Frame of the fixed frame in which scan is localized
Header header

# The scan
# Follows the following convention:
# ranges[SCAN_PREPROCESSED]: The preprocessed scan (filtered for shadow points etc.)
# ranges[SCAN_SELF_FILTERED]: The self-filtered scan
# ranges[SCAN_RAW]: The original unmodified scan
sensor_msgs/MultiEchoLaserScan processed_scan

# The pose of the sensor in fixed frame when
# first ray was retrieved 
geometry_msgs/Transform transform_first_ray

# The pose of the sensor in fixed frame when
# last ray was retrieved 
geometry_msgs/Transform transform_last_ray

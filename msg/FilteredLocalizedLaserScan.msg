# This message provides a scan in a minimal configuration
# that allows high fidelity projection while at the same
# time conserving the compact representation it has when
# coming from the sensor. Also provides self filter
# information

# Header
# stamp: Time of first ray in scan
# frame_id: Frame of the fixed frame in which scan is localized
Header header

# The scan
# Follows the following convention:
# ranges[0]: The original unmodified scan
# ranges[1]: The preprocessed scan (filtered for shadow points etc.)
# ranges[2]: The self-filtered scan
sensor_msgs/MultiEchoLaserScan processed_scan

# The pose of the sensor in fixed frame when
# first ray was retrieved 
geometry_msgs/Pose pose_first_ray

# The pose of the sensor in fixed frame when
# last ray was retrieved 
geometry_msgs/Pose pose_last_ray
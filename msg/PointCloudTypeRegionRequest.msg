# Specify frame_id of point cloud data here
EnvironmentRegionRequest environment_region_request

# Sensor data sources
uint8 LIDAR_FILTERED = 0
uint8 LIDAR_UNFILTERED = 1
uint8 STEREO = 2
uint8 STEREO_HAND = 3

# The data source. Can be one of above sources
uint8 data_source

# The requested aggregation size for the cloud. Currently not used
# for stereo data. If set to 0, will use default setting (currently
# 500)
uint32 aggregation_size

# LocalizedImage provides image data along with camera pose information,
# allowing for projection of the image data. CameraInfo is also provided
# for convenience

# The image
sensor_msgs/Image image

# The CameraInfo associated with the image
sensor_msgs/CameraInfo camera_info

# The transform of the camera relative to the world frame when the image was taken
geometry_msgs/Transform camera_pose_world_frame
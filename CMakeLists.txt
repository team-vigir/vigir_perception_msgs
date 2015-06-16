cmake_minimum_required(VERSION 2.8.3)
project(vigir_perception_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
  actionlib_msgs
  octomap_msgs
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  DownSampledImageRequest.msg
  EnvironmentRegionRequest.msg
  PointCloudTypeRegionRequest.msg
  RaycastRequest.msg
  FilteredLocalizedLaserScan.msg
  CompressedFilteredScanData.msg
  LocalizedImage.msg)

add_service_files(
  DIRECTORY srv
  FILES
  GetPoseInFrame.srv
  PointCloudRegionRequest.srv
  OctomapRegionRequest.srv
)
  
add_action_files(
   FILES
   GetLocomotionTargetPose.action
)  

generate_messages(DEPENDENCIES sensor_msgs geometry_msgs actionlib_msgs octomap_msgs std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs geometry_msgs message_runtime std_msgs
  DEPENDS)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
  
add_executable(vigir_perception_msgs_test test/main.cpp)
add_dependencies(vigir_perception_msgs_test ${PROJECT_NAME}_generate_messages)
target_link_libraries(vigir_perception_msgs_test  ${catkin_LIBRARIES})

install(TARGETS vigir_perception_msgs_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


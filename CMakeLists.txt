cmake_minimum_required(VERSION 2.8.3)
project(vigir_perception_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  FilteredLocalizedLaserScan.msg)

#add_service_files(
#  DIRECTORY srv
#  FILES SetCameraInfo.srv)

generate_messages(DEPENDENCIES sensor_msgs geometry_msgs std_msgs)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs geometry_msgs message_runtime std_msgs
  DEPENDS)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h")

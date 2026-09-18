cmake_minimum_required(VERSION 3.0.2)
project(object_mapping)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  image_geometry
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

add_message_files(
  FILES
  DetectedObject.msg
  DetectedObjectArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
  scripts/bottle_mapping_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 3.0.2)
project(yolo_rviz_markers)

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
  message_generation
  # yolo_ros_msgs
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

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
  scripts/yolo_to_rviz.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

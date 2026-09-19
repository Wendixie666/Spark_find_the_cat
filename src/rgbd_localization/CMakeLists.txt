cmake_minimum_required(VERSION 3.0.2)
project(rgbd_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_geometry
  tf2_ros
  tf2_geometry_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_geometry
  tf2_ros
  tf2_geometry_msgs
)

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

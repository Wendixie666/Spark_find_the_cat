cmake_minimum_required(VERSION 3.0.2)
project(find_cat)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/find_cat_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

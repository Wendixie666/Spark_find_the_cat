cmake_minimum_required(VERSION 3.0.2)
project(yolo_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  yolo_ros_msgs
)

catkin_package(CATKIN_DEPENDS
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  yolo_ros_msgs
)

catkin_install_python(PROGRAMS
  scripts/yolo.py
  scripts/yolo_3D.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch weights
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

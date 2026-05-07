yolo_rviz_markers
=================

Small catkin package that converts YOLO bounding boxes + depth into RViz markers.

Run:

  cd ~/yolo_rviz_ws
  catkin_make
  source devel/setup.bash
  roslaunch yolo_rviz_markers yolo_to_rviz.launch

If your YOLO messages are in another workspace (current machine: ~/yolo_ros), source it first:

  source ~/yolo_ros/devel/setup.bash
  source ~/yolo_rviz_ws/devel/setup.bash
  roslaunch yolo_rviz_markers yolo_to_rviz.launch

Or run the script directly (after sourcing workspace):

  rosrun yolo_rviz_markers yolo_to_rviz.py

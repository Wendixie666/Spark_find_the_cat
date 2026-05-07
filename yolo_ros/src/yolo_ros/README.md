# yolo_ros

Provides a ROS package based on PyTorch-YOLO.

## Running Environment:

- Ubuntu 20.04
- ROS Noetic
- Python >= 3.7.0, PyTorch >= 1.7

## Environment Configuration:

1. First, install the environment with the corresponding Python and PyTorch versions.

2. Then install the following dependencies:

   ```
   pip install ultralytics
   pip install rospkg
   ```

3. Install Yolo_ROS:

   ```
   cd catkin_ws/src
   git clone https://github.com/HT-hlf/yolo_ros.git
   cd ..
   catkin_make
   ```

## Usage

## Use YOLO to Detect Objects in Images

#### Use in Simulation Environment

- Start the simulation environment and robot:

  ```
  source devel/setup.bash
  roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
  ```

- Start the ros_yolo node:

  ```
  source devel/setup.bash
  roslaunch yolo_ros yolo.launch
  ```


- ![1775750686658](README_CN.assets/1775750802757.png)

  ![1775750875348](README_CN.assets/1775750875348.png)

#### Use with Recorded Dataset

- Play bag data:

  ```
  rosbag play kitti.bag -r 0.2
  ```

- Start the ros_yolo node:

  ```
  source devel/setup.bash
  roslaunch yolo_ros yolo_kitti.launch
  ```

  ![](README_CN.assets/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202026-04-09%20214157.png)

  ![屏幕截图 2026-04-09 214316](README.assets/屏幕截图 2026-04-09 214316.png)

## Use YOLO to Detect Objects in Images and Calculate Their 3D Coordinates

- Start the simulation environment and robot:

  ```
  source devel/setup.bash
  roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
  ```

- Start the ros_yolo3D node:

  ```
  source devel/setup.bash
  roslaunch yolo_ros yolo_3D.launch
  ```

  ​	

  ![](README_CN.assets/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202026-04-09%20212346.png)

  ![屏幕截图 2026-04-09 212443](README_CN.assets/屏幕截图 2026-04-09 212443.png)
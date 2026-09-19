# Original Spark runtime notes

Historical runtime notes recovered from the original working course-project
repository. These commands document the original deployment and are not
guaranteed to work unchanged on a modern machine.

## Platform dependency

The course project ran on the NXROBO Spark ROS Noetic platform. The platform
workspace supplied the robot driver, camera bring-up, TF, SLAM/RTAB-Map,
navigation, and hardware-specific packages. The maintained repository keeps
only the project-specific packages.

Platform repository: <https://github.com/NXROBO/spark_noetic>

## Historical commands

On the Spark robot, the recorded mapping and navigation commands were:

```bash
roslaunch spark_rtabmap spark_rtabmap_teleop.launch
roslaunch spark_navigation amcl_demo.launch map_file:=/home/spark/my_map.yaml
```

On the PC/laptop, the recorded project commands were:

```bash
roslaunch yolo_ros yolo.launch
roslaunch explore_lite explore.launch
roslaunch yolo_rviz_markers yolo_to_rviz.launch
roslaunch find_cat find_cat.launch
```

The historical exploration stack was `m-explore`/`explore_lite`; the project
stopped the node through `/explore`.

## Current maintained equivalents

The maintained package names and launch entry points are:

| Historical command | Current equivalent |
| --- | --- |
| `roslaunch yolo_ros yolo.launch` | `roslaunch yolo_ros yolo.launch` |
| `roslaunch explore_lite explore.launch` | `roslaunch explore_lite explore.launch` |
| `roslaunch yolo_rviz_markers yolo_to_rviz.launch` | `roslaunch object_mapping yolo_to_rviz.launch` |
| `roslaunch find_cat find_cat.launch` | `roslaunch find_cat find_cat.launch` |

The Spark-side commands remain external platform commands. There is no claim
that this repository can start the complete robot stack by itself.

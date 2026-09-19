# Third-party components and attribution

This repository combines course-project code with an external Spark runtime
and several upstream ROS components. The table below makes that boundary
explicit.

| Component | Use in this project | Attribution |
| --- | --- | --- |
| NXROBO Spark ROS Noetic | Robot hardware, RGB-D camera, TF, SLAM/RTAB-Map, navigation, and bring-up | [NXROBO/spark_noetic](https://github.com/NXROBO/spark_noetic) |
| `frontier_exploration` | Frontier exploration used by the final Spark snapshot | [paulbovbel/frontier_exploration](https://github.com/paulbovbel/frontier_exploration) |
| `explore_lite` | Frontier-based autonomous exploration | [hrnr/m-explore](https://github.com/hrnr/m-explore) |
| `move_base` | Navigation action used by the cat-search flow | ROS navigation stack |
| YOLO ROS bridge | Reference for the optional `yolo_ros` bridge and bounding-box messages | [HT-hlf/yolo_ros](https://github.com/HT-hlf/yolo_ros) |
| Rosboard | Development-time ROS visualization | [dheera/rosboard](https://github.com/dheera/rosboard) |

## Adapted YOLO ROS content

The scripts under `src/yolo_ros/` were adapted from the upstream YOLO ROS
implementation for this project. The custom message definitions in
`src/yolo_ros_msgs/msg/` follow the corresponding upstream message files.
Project-specific changes include integration with the retained ROS package
layout, launch parameters, model paths, and the maintained RGB-D perception
flows. The optional bridge is not required by `find_cat` or `object_mapping`.

The former `media/yolo_cn/` and `media/yolo_en/` directories contained copied
upstream README assets from `HT-hlf/yolo_ros`, not project screenshots. They
are retained under `third_party/yolo_ros/README.assets/` so they are not
presented as original project media.

## License notes

- Root repository materials: Apache License 2.0, see [`LICENSE`](LICENSE).
- Individual ROS package licenses are declared in each `src/*/package.xml`.
- Upstream code, messages, and media remain subject to their upstream license
  and attribution requirements.

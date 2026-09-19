# Spark Find the Cat

**Autonomous Exploration, Semantic Object Mapping and Target Navigation with ROS 1**

This repository contains the maintained parts of a ROS Noetic course project.
It combines an RGB-D camera and Ultralytics YOLO with map-based exploration:

```text
RGB-D Camera -> YOLO -> pixel/depth -> 3D camera point -> TF -> map
                                      |                  |
                              Bottle mapping        Cat search
                                      |                  |
                                RViz Marker        explore_lite -> move_base
```

The original project ran on a Spark mobile robot. The physical platform is no
longer available, so the repository preserves the project logic and standard
ROS interfaces without bundling the old Spark workspace.

## Features

### Autonomous target search

`find_cat` embeds YOLO inference and performs the original task flow:

1. Monitor SLAM map coverage while exploration runs.
2. Detect the `cat` class in RGB images.
3. Read aligned depth and project the detection center into 3D.
4. Transform the point into `map` and confirm spatially consistent detections.
5. Stop exploration when the configured coverage threshold is reached.
6. Send a goal near the confirmed cat through `move_base`.

### Semantic object mapping

`object_mapping` detects `bottle` during exploration and keeps a stable map of
objects. It performs RGB-D localization, camera-to-`map` TF conversion,
nearby-position clustering, exponential smoothing, repeated-hit confirmation,
and permanent RViz markers labelled `Bottle_0`, `Bottle_1`, and so on.

## Repository structure

```text
.
├── README.md
├── requirements.txt
├── docs/
│   └── original_robot_setup.md
├── media/
└── src/
    ├── find_cat/        # exploration state machine and cat navigation
    ├── object_mapping/  # bottle localization and persistent RViz markers
    ├── yolo_ros/        # optional image/bounding-box bridge
    └── yolo_ros_msgs/   # messages used by yolo_ros
```

The former `build/`, `devel/`, `rosboard`, copied Ultralytics source tree, and
full Spark workspace are intentionally not part of the maintained project.

## Requirements

- Ubuntu 20.04
- ROS Noetic with `catkin`, `tf2`, `cv_bridge`, `image_geometry`, `actionlib`,
  `move_base`, and the message packages used by the nodes
- `explore_lite` for autonomous exploration
- Python 3 and the packages in [`requirements.txt`](requirements.txt)
- An RGB-D source publishing aligned RGB/depth images and camera info
- A model file such as `src/yolo_ros/weights/yolov8s.pt`

ROS dependencies should be installed with `apt`/`rosdep`; do not put them in
the pip requirements file.

## Installation and build

```bash
cd Spark_find_the_cat
sudo apt update
sudo apt install \
  ros-noetic-cv-bridge \
  ros-noetic-image-geometry \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-move-base-msgs
python3 -m pip install -r requirements.txt

source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

The build creates `build/` and `devel/` locally; they are ignored and should
not be committed.

## Usage

Start the external camera, TF, SLAM, `explore_lite`, and `move_base` stack
first. Then run either maintained capability:

```bash
# Persistent bottle markers in RViz
roslaunch object_mapping yolo_to_rviz.launch

# Autonomous cat search and navigation
roslaunch find_cat find_cat.launch
```

The `yolo_ros` package is optional. It publishes the project's custom
`yolo_ros_msgs/BoundingBoxes` stream for setups that want a standalone YOLO
bridge:

```bash
roslaunch yolo_ros yolo.launch
```

The maintained cat and bottle nodes run YOLO directly, so `find_cat` does not
subscribe to a legacy `/yolo/bounding_boxes` parameter.

The older `yolo_3D.py` / `yolo_3D.launch` path is retained as a legacy
camera-frame detector and is not part of the maintained cat or bottle flows.

## Important topics and parameters

The launch files expose the camera topics, model path, target frame, and
thresholds. Common defaults are:

| Purpose | Default |
| --- | --- |
| RGB image | `/camera/rgb/image_raw` |
| Depth image | `/camera/depth/image_rect_raw` |
| Camera info | `/camera/rgb/camera_info` |
| Map | `/map` |
| Target frame | `map` |
| Bottle markers | `/visualization_marker` |
| Bottle detection messages | `/detected_objects` |

Cat-search parameters include `coverage_threshold`, `confirmation_hits`,
`cluster_distance_m`, `depth_min_m`, `depth_max_m`, `arrival_distance_m`, and
`target_offset_m`. Bottle-mapping parameters additionally include
`inference_confidence`, `cluster_distance_m`, `confirmation_hits`, and
`smoothing_alpha`.

## Demo media

The `media/` directory contains retained project screenshots. A reproducible
live demo still requires a camera or rosbag plus a valid TF/SLAM/navigation
stack; no claim is made that the old Spark hardware is available.

## Current limitations

- The repository does not include Spark hardware drivers, a complete Gazebo
  world, SLAM, `explore_lite`, or `move_base` configuration.
- RGB and depth are expected to be aligned and temporally close. If the camera
  publishes different topic names, pass launch arguments or parameters.
- The included model weights are retained project assets; model accuracy and
  inference speed depend on the selected checkpoint and hardware.
- In this cleanup environment ROS Noetic and the physical sensors are not
  available, so runtime behavior cannot be hardware-validated here.

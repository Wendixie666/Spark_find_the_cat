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
3. Read the latest available depth frame and project the detection center into 3D.
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
│   ├── original_robot_setup.md
│   └── original_runtime.md
├── media/
└── src/
    ├── find_cat/        # exploration state machine and cat navigation
    ├── object_mapping/  # bottle localization and persistent RViz markers
    ├── rgbd_localization/ # shared RGB-D projection and TF adapter
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
- An RGB-D source publishing the topics required by the selected launch file
- A YOLO model file, passed through the `model_path` launch argument

The original course project ran on the [NXROBO Spark ROS Noetic platform](https://github.com/NXROBO/spark_noetic).
That external platform provides the robot driver, camera driver, TF,
SLAM/RTAB-Map, `move_base`, and hardware-specific packages. This repository
provides YOLO detection, RGB-D object localization, stable bottle mapping,
cat-search state management, and navigation-to-cat decisions.

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

### 1. Maintained project packages

Start the external Spark runtime first. Then run either maintained capability:

```bash
# Persistent bottle markers in RViz
roslaunch object_mapping bottle_mapping.launch model_path:=/path/to/model.pt

# Autonomous cat search and navigation
roslaunch find_cat find_cat.launch model_path:=/path/to/model.pt
```

The default maintained model name is the historical `yolov8n.pt`; this
checkpoint is not guaranteed to be present in the repository, so provide a
compatible local checkpoint with `model_path` when needed.

### 2. Required external Spark runtime

The maintained launches do not start the robot driver, camera, TF, SLAM,
`explore_lite`, or `move_base`. Those services must already be available from
the external Spark platform or an equivalent ROS setup.

### 3. Historical deployment

The original commands and their current package-name equivalents are recorded
in [`docs/original_runtime.md`](docs/original_runtime.md). They are historical
notes, not a promise of a one-command modern deployment.

### 4. Running the maintained nodes

`yolo_to_rviz.launch` remains as a compatibility alias for
`bottle_mapping.launch`. The maintained nodes do not require the optional
`yolo_ros` package; pass a local model path when running offline.

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
thresholds. The historical maintained entries intentionally keep different
depth/camera-info defaults:

| Purpose | Default |
| --- | --- |
| `find_cat` RGB image | `/camera/rgb/image_raw` |
| `find_cat` depth image | `/camera/depth/image_rect_raw` |
| `find_cat` camera info | `/camera/depth/camera_info` |
| `object_mapping` RGB image | `/camera/rgb/image_raw` |
| `object_mapping` depth image | `/camera/depth/image_raw` |
| `object_mapping` camera info | `/camera/rgb/camera_info` |
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
- Each maintained node subscribes to RGB and depth independently and uses the
  latest depth frame, matching the original Spark implementation. If the
  camera publishes different topic names, pass launch arguments or parameters.
- The included model weights are retained project assets; model accuracy and
  inference speed depend on the selected checkpoint and hardware.
- In this cleanup environment ROS Noetic, the physical sensors, and the Spark
  robot are not available, so end-to-end runtime behavior cannot be hardware-
  validated here.

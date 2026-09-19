# Spark Find the Cat

**Autonomous target search and semantic object mapping on a physical [NXROBO Spark](https://github.com/NXROBO/spark_noetic) robot.**

This ROS 1 project enables a mobile robot to:

- **Find the cat** — autonomously explore an unknown indoor environment,
  detect and localize a cat with YOLO and RGB-D perception, confirm its
  position, then navigate to it with `move_base`.
- **Map the bottles** — detect bottles during exploration, localize them in
  the global `map` frame, merge repeated observations, and maintain stable,
  persistent RViz markers.

Built and tested on a physical Spark robot with **ROS Noetic, YOLO, RGB-D
perception, TF2, RTAB-Map, frontier exploration, and `move_base`**.

**English** | [中文](README_CN.md)

## Demo

![Spark Find the Cat demo](docs/demo_first_1m05s.gif)

```text
Explore an unknown environment
        ↓
YOLO object detection
        ↓
RGB-D 3D localization
        ↓
Transform detections into the map frame
        ↓
 ┌─────────────────────┬─────────────────────────┐
 │ Cat                 │ Bottle                  │
 │ confirm location    │ cluster observations    │
 │ stop exploration    │ smooth positions        │
 │ navigate to target  │ persistent RViz markers │
 └─────────────────────┴─────────────────────────┘
```

Project documents:

- [Final project report](docs/project.pdf)
- [Final course presentation](docs/Final+pre.pptx)

## What We Built

### Autonomous Cat Search

The robot explores while continuously detecting the `cat` class. Spatially
consistent detections are localized with RGB-D data and transformed into the
global `map` frame. Once the target is confirmed and the configured map
coverage is reached, the state machine stops exploration and sends an approach
goal near the cat through `move_base`.

`exploration → detection → RGB-D localization → confirmation → navigation`

### Semantic Bottle Mapping

While the robot moves through the environment, detected bottles are localized
in 3D and transformed into the global `map` frame. Nearby observations are
clustered, smoothed, and confirmed to avoid duplicates and unstable positions,
producing persistent semantic markers in RViz.

`detection → RGB-D localization → map transform → clustering → smoothing → marker`

## System Architecture

```text
RGB-D camera → YOLO detection → pixel + depth → 3D camera point → TF → map
       │                                                       │
       ├── bottle detections → stable RViz markers              │
       └── cat detections → confirmation → stop exploration → move_base

RTAB-Map / SLAM → map coverage → exploration backend → find_cat state machine
```

The core perception-to-map pipeline is:

```text
YOLO detection
      ↓
RGB pixel + depth lookup
      ↓
Camera-space 3D point
      ↓
TF2 transform to map
      ↓
Spatial clustering and confirmation
      ↓
Position smoothing
      ↓
Persistent RViz marker
```

## Project Code and Platform Boundary

| Component | Role | Boundary |
| --- | --- | --- |
| `find_cat` | Cat-search state machine, detection confirmation, map-frame target approach, and navigation goal | Course project implementation |
| `object_mapping` | RGB-D bottle localization, spatial deduplication, smoothing, repeated-hit confirmation, and persistent RViz markers | Course project implementation |
| `rgbd_localization` | Shared pixel/depth projection and camera-to-`map` TF conversion | Course project implementation |
| [NXROBO Spark](https://github.com/NXROBO/spark_noetic) | Robot drivers, RGB-D camera, TF, SLAM/RTAB-Map, hardware bring-up, and navigation runtime | Upstream platform |
| `frontier_exploration` / [`explore_lite`](https://github.com/hrnr/m-explore) | Frontier-based autonomous exploration | Upstream ROS components |
| `move_base` | Navigation action used to approach the confirmed cat location | Upstream ROS component |
| [`yolo_ros`](https://github.com/HT-hlf/yolo_ros) | Reference for the optional YOLO ROS bridge and bounding-box messages | Adapted/reference component |
| [Rosboard](https://github.com/dheera/rosboard) | Runtime visualization used during development and demonstration | External tool |

The complete Spark hardware workspace is not included in this repository.
Maintained launch files assume that the robot driver, camera, TF, SLAM,
exploration backend, and `move_base` are already running from the Spark
platform or an equivalent ROS setup. See [`THIRD_PARTY.md`](THIRD_PARTY.md) for
upstream repositories, component boundaries, and attribution details.

## Installation

### Requirements

- Ubuntu 20.04
- ROS Noetic with `catkin`, `tf2`, `cv_bridge`, `image_geometry`, `actionlib`,
  `move_base`, and the message packages used by the nodes
- An exploration backend such as `frontier_exploration` or `explore_lite`
- Python 3 and the packages in [`requirements.txt`](requirements.txt)
- An RGB-D source publishing the topics required by the selected launch file
- A compatible YOLO model file passed through the `model_path` launch argument

### Build

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

### Usage

Start the external Spark runtime first. Then run either maintained capability:

```bash
# Persistent bottle markers in RViz
roslaunch object_mapping bottle_mapping.launch model_path:=/path/to/model.pt

# Autonomous cat search and navigation
roslaunch find_cat find_cat.launch model_path:=/path/to/model.pt
```

The maintained launches do not start the robot driver, camera, TF, SLAM, the
exploration backend, or `move_base`. Those services must already be available
from the external Spark platform or an equivalent ROS setup.

`yolo_to_rviz.launch` remains a compatibility alias for
`bottle_mapping.launch`. The maintained cat and bottle nodes run YOLO directly;
the optional `yolo_ros` package is retained for setups that want a standalone
YOLO bridge:

```bash
roslaunch yolo_ros yolo.launch
```

The older `yolo_3D.py` / `yolo_3D.launch` path is retained as a legacy
camera-frame detector and is not part of the maintained cat or bottle flows.

## Important topics and parameters

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

## Current Deployment and Maintenance Status

| Item | Original final demo | Current repository |
| --- | --- | --- |
| YOLO model | YOLO26s | `find_cat` and `object_mapping` default to `yolo26s.pt` |
| Exploration coverage before navigation | 90% | `find_cat.launch` defaults to `0.90` |
| Model files present in the repository | — | `yolo26s.pt` and `yolov8s.pt` |

The bundled YOLO26s weights are at `src/yolo_ros/weights/yolo26s.pt`. For
offline or reproducible runs, pass that absolute path explicitly with
`model_path:=/path/to/yolo26s.pt`. The original commands and their current
package-name equivalents are recorded in
[`docs/original_runtime.md`](docs/original_runtime.md).

## Repository structure

```text
.
├── README.md
├── README_CN.md
├── THIRD_PARTY.md
├── requirements.txt
├── docs/
│   ├── demo_first_1m05s.gif
│   ├── project.pdf
│   ├── Final+pre.pptx
│   ├── original_robot_setup.md
│   └── original_runtime.md
├── media/
│   └── README.md
└── src/
    ├── find_cat/                        # exploration state machine and cat navigation
    ├── object_mapping/                  # bottle localization and persistent RViz markers
    ├── rgbd_localization/               # shared RGB-D projection and TF adapter
    ├── yolo_ros/                        # optional image/bounding-box bridge
    └── yolo_ros_msgs/                   # messages used by yolo_ros
```

## Current limitations

- The repository does not include Spark hardware drivers. For Spark-related
  software, see [NXROBO Spark](https://github.com/NXROBO/spark_noetic).
- Each maintained node subscribes to RGB and depth independently and uses the
  latest depth frame, matching the original Spark implementation. If the
  camera publishes different topic names, pass launch arguments or parameters.
- The included model weights are retained project assets; model accuracy and
  inference speed depend on the selected checkpoint and hardware.

# Spark Find the Cat

ROS 1 Noetic project for autonomous exploration, RGB-D semantic mapping, and
navigation to a detected target on the NXROBO Spark platform.

## Demo

> Demo video placeholder — a 40–90 second project walkthrough will be added
> here.

The planned sequence is: autonomous exploration → YOLO detection → RGB-D
localization and RViz map marker → cat confirmation → exploration stop →
`move_base` navigation to the cat.

## Platform / upstream

This course project was developed and tested on the
[NXROBO Spark ROS Noetic platform](https://github.com/NXROBO/spark_noetic).

The Spark platform provided the robot drivers, RGB-D camera interfaces, TF
tree, SLAM/RTAB-Map, navigation stack, and hardware bring-up. This repository
contains the project-specific perception, RGB-D localization, semantic
mapping, and target-search logic developed for the course project. The old
Spark workspace is not bundled here because the physical platform is no
longer available.

## System overview

```text
RGB-D camera → YOLO detection → pixel + depth → 3D camera point → TF → map
       │                                                       │
       ├── bottle detections → stable RViz markers              │
       └── cat detections → confirmation → stop exploration → move_base

RTAB-Map / SLAM → map coverage → explore_lite → find_cat state machine
```

## What we built / upstream components

| Component | Role | Boundary |
| --- | --- | --- |
| `find_cat` | Cat-search state machine, detection confirmation, map-frame target approach, and navigation goal | Course project implementation |
| `object_mapping` | RGB-D bottle localization, spatial deduplication, smoothing, repeated-hit confirmation, and persistent RViz markers | Course project implementation |
| `rgbd_localization` | Shared pixel/depth projection and camera-to-`map` TF conversion | Course project implementation |
| [NXROBO Spark](https://github.com/NXROBO/spark_noetic) | Robot drivers, RGB-D camera, TF, SLAM/RTAB-Map, hardware bring-up, and navigation runtime | Upstream platform |
| [`explore_lite`](https://github.com/hrnr/m-explore) | Frontier-based autonomous exploration | Upstream ROS component |
| `move_base` | Navigation action used to approach the confirmed cat location | Upstream ROS component |
| [`yolo_ros`](https://github.com/HT-hlf/yolo_ros) | Reference for the optional YOLO ROS bridge and bounding-box messages | Adapted/reference component |
| [Rosboard](https://github.com/dheera/rosboard) | Runtime visualization used during development and demonstration | External tool |

The project contribution is the perception-to-map pipeline and the task logic
that connects detection, spatial confirmation, exploration, and navigation.

## Features

### Autonomous target search

`find_cat` implements the original task flow:

1. Monitor SLAM map coverage while exploration runs.
2. Detect the `cat` class in RGB images.
3. Read the latest available depth frame and project the detection center into 3D.
4. Transform the point into `map` and confirm spatially consistent detections.
5. Stop exploration when the configured coverage threshold is reached.
6. Send an approach goal near the confirmed cat through `move_base`.

### Stable semantic object mapping

`object_mapping` detects `bottle` during exploration and keeps a stable map of
objects. It performs RGB-D localization, camera-to-`map` TF conversion,
nearby-position clustering, exponential smoothing, repeated-hit confirmation,
and permanent RViz markers labelled `Bottle_0`, `Bottle_1`, and so on. The
markers use the `map` frame, so they remain stable while the robot moves.

## Screenshots and project media

Project screenshots will be added under `media/screenshots/` after selecting
the final robot, Rosboard, and RViz views. The intended filenames are:

- `media/screenshots/robot_environment.png`
- `media/screenshots/rosboard_detection.png`
- `media/screenshots/rviz_mapping.png`
- `media/architecture.png`

The full demo video is intentionally left as a placeholder in the
[Demo](#demo) section.

## Installation

### Requirements

- Ubuntu 20.04
- ROS Noetic with `catkin`, `tf2`, `cv_bridge`, `image_geometry`, `actionlib`,
  `move_base`, and the message packages used by the nodes
- `explore_lite` for autonomous exploration
- Python 3 and the packages in [`requirements.txt`](requirements.txt)
- An RGB-D source publishing the topics required by the selected launch file
- A compatible YOLO model file passed through the `model_path` launch argument

ROS dependencies should be installed with `apt`/`rosdep`; do not put them in
the pip requirements file.

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

The build creates `build/` and `devel/` locally; they are ignored and should
not be committed.

## Usage

Start the external Spark runtime first. Then run either maintained capability:

```bash
# Persistent bottle markers in RViz
roslaunch object_mapping bottle_mapping.launch model_path:=/path/to/model.pt

# Autonomous cat search and navigation
roslaunch find_cat find_cat.launch model_path:=/path/to/model.pt
```

The maintained launches do not start the robot driver, camera, TF, SLAM,
`explore_lite`, or `move_base`. Those services must already be available from
the external Spark platform or an equivalent ROS setup.

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

## Original deployment and current maintained status

The presentation describes the original final demo, while the launch files
contain the current maintained defaults. They are not identical:

| Item | Original final demo | Current repository |
| --- | --- | --- |
| YOLO model | YOLO26s | `find_cat` and `object_mapping` default to `yolov8n.pt`; optional `yolo_ros` defaults to `yolo26s.pt` |
| Exploration coverage before navigation | 90% | `find_cat.launch` defaults to `0.95` |
| Model files present in the repository | — | `yolo26s.pt` and `yolov8s.pt`; `yolov8n.pt` is not bundled |

Pass `model_path:=/path/to/model.pt` explicitly when using the maintained
launches. The original commands and their current package-name equivalents are
recorded in [`docs/original_runtime.md`](docs/original_runtime.md).

The course presentation is available at
[`docs/Final+pre.pptx`](docs/Final+pre.pptx).

## Repository structure

```text
.
├── README.md
├── THIRD_PARTY.md
├── requirements.txt
├── docs/
│   ├── original_robot_setup.md
│   ├── original_runtime.md
│   └── dev/CONTEXT.md
├── media/
│   ├── architecture.png                 # planned project architecture image
│   └── screenshots/                     # planned project screenshots
├── third_party/
│   └── yolo_ros/                         # retained upstream README assets
└── src/
    ├── find_cat/                        # exploration state machine and cat navigation
    ├── object_mapping/                  # bottle localization and persistent RViz markers
    ├── rgbd_localization/               # shared RGB-D projection and TF adapter
    ├── yolo_ros/                        # optional image/bounding-box bridge
    └── yolo_ros_msgs/                   # messages used by yolo_ros
```

The former `build/`, `devel/`, `rosboard`, copied Ultralytics source tree, and
full Spark workspace are intentionally not part of the maintained project.

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

## Credits and third-party materials

See [`THIRD_PARTY.md`](THIRD_PARTY.md) for upstream repositories, component
boundaries, and the attribution note for the adapted YOLO ROS files.

## License

The root repository materials are provided under the Apache License 2.0; see
[`LICENSE`](LICENSE). Individual ROS packages declare their own package
licenses in `package.xml`. Third-party components and retained upstream assets
remain subject to their respective licenses.

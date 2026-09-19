# Spark Find the Cat

Autonomous indoor exploration, RGB-D semantic mapping, and target-directed
navigation with ROS 1 Noetic on the [NXROBO Spark](https://github.com/NXROBO/spark_noetic)
platform.

<a href="#english">English</a> · <a href="#中文">中文</a>

<a id="english"></a>

<details open>
<summary>English</summary>

## Demo

![Spark Find the Cat demo](docs/demo_first_1m05s.gif)

The demo shows the complete task flow:

```text
autonomous exploration → YOLO detection → RGB-D localization
→ RViz map marker → cat confirmation → exploration stop
→ move_base navigation to the target
```

Project documents:

- [Final project report](docs/project.pdf)
- [Final course presentation](docs/Final+pre.pptx)

## Overview

This course project was developed and tested on a physical NXROBO Spark robot.
It combines RGB-D perception, semantic object mapping, autonomous exploration,
and target-directed navigation to find a cat in an unknown indoor environment.

The Spark platform provided the robot drivers, RGB-D camera interfaces, TF,
SLAM/RTAB-Map, and navigation runtime. This repository contains the
project-specific perception, RGB-D localization, semantic mapping, and
target-search logic developed for the course project.

## System pipeline

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

## What we built

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

## Platform and upstream dependencies

The complete Spark hardware workspace is not bundled here because the physical
platform is external to this repository. The maintained launches expect the
robot driver, camera, TF, SLAM, exploration backend, and `move_base` to already
be running from the Spark platform or an equivalent ROS setup.

See [`THIRD_PARTY.md`](THIRD_PARTY.md) for upstream repositories, component
boundaries, and attribution details.

## Installation

### Requirements

- Ubuntu 20.04
- ROS Noetic with `catkin`, `tf2`, `cv_bridge`, `image_geometry`, `actionlib`,
  `move_base`, and the message packages used by the nodes
- An exploration backend such as `frontier_exploration` or `explore_lite`
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

## Original deployment and current maintained status

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

- The repository does not include Spark hardware drivers, a complete Gazebo
  world, SLAM, an exploration backend, or `move_base` configuration.
- Each maintained node subscribes to RGB and depth independently and uses the
  latest depth frame, matching the original Spark implementation. If the
  camera publishes different topic names, pass launch arguments or parameters.
- The included model weights are retained project assets; model accuracy and
  inference speed depend on the selected checkpoint and hardware.
- In this cleanup environment ROS Noetic, the physical sensors, and the Spark
  robot are not available, so end-to-end runtime behavior cannot be
  hardware-validated here.

## License

The root repository materials are provided under the Apache License 2.0; see
[`LICENSE`](LICENSE). Individual ROS packages declare their own package
licenses in `package.xml`. Third-party components and retained upstream assets
remain subject to their respective licenses.

</details>

<a id="中文"></a>

<details>
<summary>中文</summary>

## 项目简介

这是一个基于 ROS 1 Noetic 和 [NXROBO Spark](https://github.com/NXROBO/spark_noetic)
机器人的室内自主探索、RGB-D 语义建图与目标导航项目。项目最初在真实
Spark 机器人上完成并部署，用于在未知室内环境中探索、检测并定位猫，最后
导航到目标附近。

本仓库主要包含课程项目自己完成的 RGB-D 定位、语义目标建图、探索状态管理
和目标搜索逻辑。Spark 底层驱动、RGB-D 相机接口、TF、SLAM/RTAB-Map 以及
导航运行环境来自上游 Spark 平台，并不包含在本仓库中。

## Demo

![Spark Find the Cat 演示](docs/demo_first_1m05s.gif)

演示流程如下：

```text
自主探索 → YOLO 检测 → RGB-D 定位 → RViz 地图标记
→ 确认猫的位置 → 停止探索 → move_base 导航到目标
```

项目文档：

- [最终项目报告](docs/project.pdf)
- [课程最终汇报 PPT](docs/Final+pre.pptx)

## 系统流程

```text
RGB-D 相机 → YOLO 检测 → 像素与深度 → 相机坐标系三维点 → TF → map
       │                                                     │
       ├── bottle 检测 → 稳定的 RViz 标记                     │
       └── cat 检测 → 目标确认 → 停止探索 → move_base

RTAB-Map / SLAM → 地图覆盖率 → 探索后端 → find_cat 状态机
```

核心流程是：YOLO 检测 → RGB 像素与深度读取 → 相机坐标系三维点 → TF2 转换到
`map` → 空间聚类与多次确认 → 位置平滑 → 持久化 RViz 标记。

## 我们完成的部分

| 组件 | 作用 | 来源边界 |
| --- | --- | --- |
| `find_cat` | 猫搜索状态机、检测确认、map 坐标系目标接近和导航目标发送 | 课程项目实现 |
| `object_mapping` | RGB-D bottle 定位、空间去重、位置平滑、多次确认和持久化 RViz 标记 | 课程项目实现 |
| `rgbd_localization` | 像素/深度投影以及相机到 `map` 的 TF 转换 | 课程项目实现 |
| [NXROBO Spark](https://github.com/NXROBO/spark_noetic) | 机器人驱动、RGB-D 相机、TF、SLAM/RTAB-Map、硬件启动和导航运行环境 | 上游平台 |
| `frontier_exploration` / [`explore_lite`](https://github.com/hrnr/m-explore) | 基于 frontier 的自主探索 | 上游 ROS 组件 |
| `move_base` | 接近已确认猫位置的导航动作 | 上游 ROS 组件 |
| [`yolo_ros`](https://github.com/HT-hlf/yolo_ros) | 可选 YOLO ROS 桥接和检测框消息的参考实现 | 改编/参考组件 |
| [Rosboard](https://github.com/dheera/rosboard) | 开发和演示期间使用的运行时可视化工具 | 外部工具 |

## 主要功能

### 自主寻找目标

`find_cat` 会在探索过程中监控地图覆盖率，检测 `cat`，利用深度数据将目标
转换到 `map` 坐标系，确认空间上稳定的多次检测；达到覆盖率阈值后停止探索，
并通过 `move_base` 导航到猫附近。

### 稳定的语义目标建图

`object_mapping` 在探索过程中检测 `bottle`，通过 RGB-D 定位、TF 转换、
邻近位置聚类、指数平滑和多次确认生成持久化的 RViz 标记。标记使用 `map`
坐标系，因此机器人移动时仍保持稳定。

## 安装与运行

### 环境要求

- Ubuntu 20.04
- ROS Noetic，以及 `catkin`、`tf2`、`cv_bridge`、`image_geometry`、
  `actionlib`、`move_base` 和节点使用的消息包
- `frontier_exploration` 或 `explore_lite` 等探索后端
- Python 3 及 [`requirements.txt`](requirements.txt) 中的依赖
- 能发布启动文件所需话题的 RGB-D 数据源
- 通过 `model_path` 启动参数传入兼容的 YOLO 模型文件

### 构建

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

### 运行

先启动外部 Spark 运行环境，再运行需要的功能：

```bash
# 在 RViz 中显示持久化 bottle 标记
roslaunch object_mapping bottle_mapping.launch model_path:=/path/to/model.pt

# 自主寻找猫并导航
roslaunch find_cat find_cat.launch model_path:=/path/to/model.pt
```

维护中的启动文件不会启动机器人驱动、相机、TF、SLAM、探索后端或
`move_base`；这些服务必须已经由外部 Spark 平台或等价的 ROS 环境提供。

## 重要话题和参数

| 用途 | 默认值 |
| --- | --- |
| `find_cat` RGB 图像 | `/camera/rgb/image_raw` |
| `find_cat` 深度图像 | `/camera/depth/image_rect_raw` |
| `find_cat` 相机信息 | `/camera/depth/camera_info` |
| `object_mapping` RGB 图像 | `/camera/rgb/image_raw` |
| `object_mapping` 深度图像 | `/camera/depth/image_raw` |
| `object_mapping` 相机信息 | `/camera/rgb/camera_info` |
| 地图 | `/map` |
| 目标坐标系 | `map` |
| Bottle 标记 | `/visualization_marker` |
| Bottle 检测消息 | `/detected_objects` |

猫搜索参数包括 `coverage_threshold`、`confirmation_hits`、
`cluster_distance_m`、`depth_min_m`、`depth_max_m`、`arrival_distance_m` 和
`target_offset_m`。Bottle 建图另外使用 `inference_confidence`、
`cluster_distance_m`、`confirmation_hits` 和 `smoothing_alpha`。

## 当前限制

- 仓库不包含 Spark 硬件驱动、完整 Gazebo 世界、SLAM、探索后端或
  `move_base` 配置。
- 节点使用最新深度帧，与原始 Spark 实现一致；如果相机话题名称不同，需要
  传入启动参数或节点参数。
- 模型准确率和推理速度取决于所选 checkpoint 与硬件。
- 当前环境没有 ROS Noetic、实体传感器和 Spark 机器人，因此无法完成端到端
  硬件验证。

## 许可证

根仓库材料采用 Apache License 2.0，详见 [`LICENSE`](LICENSE)。各 ROS 包在
`package.xml` 中声明自己的许可证；第三方组件和保留的上游资源仍受其各自
许可证约束。

</details>

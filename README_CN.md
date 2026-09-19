# Spark Find the Cat

[English](README.md) | **中文**

**在真实 [NXROBO Spark](https://github.com/NXROBO/spark_noetic) 机器人上实现的自主目标搜索与语义目标建图。**

这个 ROS 1 项目让移动机器人能够：

-  **寻找猫**：在未知室内环境中自主探索，利用 YOLO 和 RGB-D 感知检测、
  定位并确认猫的位置，再通过 `move_base` 导航到目标附近。
- **标记瓶子**：在探索过程中检测瓶子，在全局 `map` 坐标系中定位并合并
  重复观测，在 RViz 中维护稳定、持久的语义标记。

项目在真实 Spark 机器人上完成并测试，使用 **ROS Noetic、YOLO、RGB-D 感知、
TF2、RTAB-Map、frontier exploration 和 `move_base`**。

## Demo

![Spark Find the Cat 演示](docs/demo_first_1m05s.gif)

```text
探索未知环境
        ↓
YOLO 目标检测
        ↓
RGB-D 三维定位
        ↓
将检测结果转换到 map 坐标系
        ↓
 ┌─────────────────────┬─────────────────────────┐
 │ 猫                  │ 瓶子                    │
 │ 确认位置            │ 聚类重复观测            │
 │ 停止探索            │ 平滑位置                │
 │ 导航到目标          │ 持久化 RViz 标记        │
 └─────────────────────┴─────────────────────────┘
```

项目文档：

- [最终项目报告](docs/project.pdf)
- [课程最终汇报 PPT](docs/Final+pre.pptx)

## 我们完成的功能

###  自主寻找猫

机器人在探索过程中持续检测 `cat`。空间上稳定的多次检测会通过 RGB-D 数据
定位，并转换到全局 `map` 坐标系。当目标位置得到确认且地图覆盖率达到配置
阈值后，状态机会停止探索，并通过 `move_base` 发送猫附近的接近目标。

`探索 → 检测 → RGB-D 定位 → 位置确认 → 导航`

### 语义瓶子建图

机器人移动时，检测到的瓶子会被三维定位并转换到全局 `map` 坐标系。邻近
观测会经过聚类、平滑和多次确认，以避免重复和不稳定的位置，最终在 RViz 中
生成持久的语义标记。

`检测 → RGB-D 定位 → map 转换 → 聚类 → 平滑 → 持久化标记`

## 系统架构

```text
RGB-D 相机 → YOLO 检测 → 像素与深度 → 相机坐标系三维点 → TF → map
       │                                                     │
       ├── bottle 检测 → 稳定的 RViz 标记                     │
       └── cat 检测 → 目标确认 → 停止探索 → move_base

RTAB-Map / SLAM → 地图覆盖率 → 探索后端 → find_cat 状态机
```

核心流程是：YOLO 检测 → RGB 像素与深度读取 → 相机坐标系三维点 → TF2 转换到
`map` → 空间聚类与多次确认 → 位置平滑 → 持久化 RViz 标记。

## 项目代码与平台边界

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

完整 Spark 硬件工作空间不包含在本仓库中。维护中的启动文件假设机器人驱动、
相机、TF、SLAM、探索后端和 `move_base` 已由 Spark 平台或等价 ROS 环境启动。
上游仓库、组件边界和归属信息见 [`THIRD_PARTY.md`](THIRD_PARTY.md)。

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

`yolo_to_rviz.launch` 保留为 `bottle_mapping.launch` 的兼容别名。维护中的猫和
瓶子节点直接运行 YOLO；如需独立的 YOLO 桥接，可使用保留的 `yolo_ros` 包：

```bash
roslaunch yolo_ros yolo.launch
```

旧的 `yolo_3D.py` / `yolo_3D.launch` 是保留的相机坐标系检测路径，不属于当前
维护的找猫或瓶子流程。

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

## 当前部署与维护状态

| 项目 | 原始最终演示 | 当前仓库 |
| --- | --- | --- |
| YOLO 模型 | YOLO26s | `find_cat` 和 `object_mapping` 默认使用 `yolo26s.pt` |
| 导航前的探索覆盖率 | 90% | `find_cat.launch` 默认值为 `0.90` |
| 仓库中包含的模型文件 | — | `yolo26s.pt` 和 `yolov8s.pt` |

随仓库提供的 YOLO26s 权重位于 `src/yolo_ros/weights/yolo26s.pt`。为离线或可复现
运行，请通过 `model_path:=/path/to/yolo26s.pt` 显式传入其绝对路径。原始命令及其
当前包名对应关系记录在 [`docs/original_runtime.md`](docs/original_runtime.md)。

## 仓库结构

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
    ├── find_cat/                        # 探索状态机和猫导航
    ├── object_mapping/                  # Bottle 定位和持久化 RViz 标记
    ├── rgbd_localization/               # 共享 RGB-D 投影与 TF 适配器
    ├── yolo_ros/                        # 可选图像/检测框桥接
    └── yolo_ros_msgs/                   # yolo_ros 使用的消息
```

## 当前限制

- 仓库不包含 Spark 硬件驱动，spark相关请参考[NXROBO Spark](https://github.com/NXROBO/spark_noetic)
- 节点使用最新深度帧，与原始 Spark 实现一致；如果相机话题名称不同，需要
  传入启动参数或节点参数。
- 模型准确率和推理速度取决于所选 checkpoint 与硬件。

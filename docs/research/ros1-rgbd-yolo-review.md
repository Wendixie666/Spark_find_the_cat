# ROS 1 RGB-D / YOLO 下一轮工作的资料核查

日期：2026-09-19  
范围：当前 `main`（`7dedb0a`）以及 ROS Noetic、`tf2_ros`、`message_filters`、Ultralytics 官方资料。

## 结论先行

用户列出的优先级总体正确。建议下一轮只做以下四项代码工作：

1. 立即修正 `rgbd_localization` 的 TF 调用，并补 image-time / latest-time 两条 adapter 测试。
2. 把模型路径改成正式节点自己的 `model_path` 参数，移除对 `yolo_ros` 包路径的默认依赖；权重文件不再提交到 Git。
3. 用 `message_filters.ApproximateTimeSynchronizer` 配对 RGB 和 depth，再进入现有检测流程。
4. 做小范围表面整理；重命名 launch 文件时保留旧文件作为兼容入口。

不建议为了这轮工作创建 detector 接口、公共 YOLO 工厂或继续拆状态机。当前代码规模不需要这些抽象。

## 1. TF 判断：这是确定的正确性 bug

仓库当前的调用位于 [`ros.py`](../../src/rgbd_localization/src/rgbd_localization/ros.py#L94-L100)：

```python
self._tf_buffer.transform(point, target_frame, stamp, self._transform_timeout)
```

ROS Noetic 官方源码给出的 Python 简单 API 是：

```text
transform(object_stamped, target_frame,
          timeout=rospy.Duration(0.0), new_type=None)
```

同时，源码明确说明真正的查询时间来自 `object_stamped.header.stamp`；`lookup_transform(..., time=0)` 才表示取最新 TF。因此当前代码把 image stamp 当成 timeout，把 timeout 当成 `new_type`，判断完全成立。[`tf2_ros/buffer_interface.py`](https://github.com/ros/geometry2/blob/noetic-devel/tf2_ros/src/tf2_ros/buffer_interface.py#L48-L66) [`lookup_transform` 文档](https://github.com/ros/geometry2/blob/noetic-devel/tf2_ros/src/tf2_ros/buffer_interface.py#L102-L112)

推荐的修复语义是：

```python
point_camera.header.stamp = image_stamp
tf_buffer.transform(point_camera, target_frame, transform_timeout)
```

image-time 失败后，必须复制 `PointStamped` 并把副本的 `header.stamp` 设为 `rospy.Time(0)`，然后再次调用同一个三参数形式。只把 `rospy.Time(0)` 作为第三个位置参数传入并不能表达“最新 TF”，因为第三个参数是 timeout。

当前 fallback 代码还复用了原来的 `PointStamped`，所以即使把调用参数数量修正，也应同步修正 fallback 对象的 header 时间。[`ros.py`](../../src/rgbd_localization/src/rgbd_localization/ros.py#L71-L92)

## 2. 权重与 `yolo_ros` 解耦：判断成立，而且应做

两个正式 launch 文件都把默认路径写成 `$(find yolo_ros)/weights/yolov8s.pt`：[`find_cat.launch`](../../src/find_cat/launch/find_cat.launch#L1-L8)、[`yolo_to_rviz.launch`](../../src/object_mapping/launch/yolo_to_rviz.launch#L1-L6)。但两个正式节点实际都直接执行 `YOLO(self.model_path)`，并不依赖 `yolo_ros` 的检测节点：[`find_cat_node.py`](../../src/find_cat/scripts/find_cat_node.py#L40-L48)、[`bottle_mapping_node.py`](../../src/object_mapping/scripts/bottle_mapping_node.py#L29-L41)。

这造成两个实际问题：

- `find_cat` 和 `object_mapping` 的 `package.xml` 没有声明 `yolo_ros`，但 roslaunch 解析默认值时仍必须能找到这个包；ROS 的 `$(find PKG)` 本质上就是通过 rospack 查找该包路径。[roslaunch substitution source](https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/substitution_args.py#L1338-L1426)
- 当前仓库实际跟踪了两个权重，合计约 42 MB；Git pack 当前约 54 MB。`yolo_ros` 被标为 optional，但模型资产却挂在它下面，模块边界不一致。

建议：

- 正式 launch 的默认值改为 `yolov8s.pt`，同时保留 `model_path:=/absolute/path/model.pt` 的覆盖方式。
- README 明确说明：离线运行必须传本地权重路径；不要把“Ultralytics 能自动下载官方模型”当作机器人运行时的可靠依赖。Ultralytics 官方 Python API 支持 `YOLO("path/to/model.pt")`，官方安装方式是 pip，但 PyTorch 版本还取决于操作系统和 CUDA。[Ultralytics Python usage](https://docs.ultralytics.com/usage/python) [`Ultralytics quickstart`](https://docs.ultralytics.com/quickstart)
- 删除 Git 中的 `.pt` 文件前先确认是否还有历史/发布流程需要它；本次代码变更可以先移除当前工作树资产并加入忽略规则，历史大文件清理是另一个需要明确授权的操作。

## 3. RGB/depth 时间同步：值得做，但要保持现有模块边界

当前两个节点都是“独立 depth subscriber 保存最新 depth，RGB callback 使用当前缓存”：[`find_cat_node.py`](../../src/find_cat/scripts/find_cat_node.py#L140-L178)、[`bottle_mapping_node.py`](../../src/object_mapping/scripts/bottle_mapping_node.py#L109-L127)。因此用户给出的 `RGB(t=10.0) + depth(t=9.8)` 风险真实存在；机器人移动或相机运动时，会直接影响像素对应的深度和最终 map 坐标。

ROS 1 的 `ApproximateTimeSynchronizer` 正是针对这个问题：按消息 `header.stamp` 配对，用 `slop` 定义允许的时间差，并要求消息有时间戳。[ROS Noetic `message_filters` source](https://github.com/ros/ros_comm/blob/noetic-devel/utilities/message_filters/src/message_filters/__init__.py#L245-L334)

实施建议：

- 两个节点各自创建 RGB / depth 的 `message_filters.Subscriber`，用 `ApproximateTimeSynchronizer` 注册成对 callback。
- `find_cat` 仍可把成对消息放进现有处理线程；不要把 YOLO 推理重新放回订阅线程。
- 初始 `slop` 先设成参数，按相机实际帧率和 rosbag/机器人日志调，不要硬编码成“理论最优值”。
- 这只解决时间配对，不替代 depth 与 RGB 的空间对齐、相机标定和正确的 camera frame。
- 两个包需要补 `message_filters` 的 ROS 依赖声明；当前 package/CMake 尚未声明它。

## 4. ROS adapter 测试：投入产出比高

现有 [`test_core.py`](../../src/rgbd_localization/test/test_core.py) 只覆盖 ROS-independent 数学核心，没有覆盖 `RgbdLocalizer.localize()` 到 fake TF buffer 的调用链。建议新增一个独立 adapter 测试文件，至少包含：

1. fake TF buffer 在 image timestamp 成功时，断言收到的 `PointStamped.header.stamp` 是图像时间，并断言 transform 只收到 `(point, target_frame, timeout)`。
2. fake TF buffer 第一次抛出 `ExtrapolationException`，第二次检查副本的 `header.stamp == rospy.Time(0)` 后返回成功。
3. 两次都失败时返回 `None`。

这类测试应在 ROS Noetic 环境中运行；当前主机没有 `rospy`、`tf2_ros`、`cv_bridge`，所以本机不能直接运行 adapter 测试。当前 `pytest` 命令也未安装，现有 core 测试本轮只做了文件检查，未执行。

## 5. 小整理项的判断

- `yolo_to_rviz.launch` 改名为 `bottle_mapping.launch` 是合理的，但建议保留旧 launch 文件作为薄兼容入口，避免已有命令立即失效。
- README 的结构应补上 `rgbd_localization/`，并把“模型位于 `yolo_ros/weights`”改成外部/参数化模型路径说明。
- `yolo_3D.py` 当前确实被 `yolo_ros/CMakeLists.txt` 安装；若判定 legacy 不再支持，应同时移除安装项或保留明确的兼容入口，不能只移动文件。
- `_robot_pose()` 的 `except Exception: pass` 应至少加 `rospy.logdebug`，最好缩小到 TF 相关异常；这属于可读性和可诊断性改进，不应排在 TF 参数 bug、同步和测试之前。

## 6. 是否需要 Python 虚拟环境

### 本机事实

当前工作机是 Ubuntu 22.04.5，Python 3.10.12 来自 pyenv；没有 `/opt/ros/noetic`，也没有 `roslaunch`。`numpy` 可导入，但 `ultralytics`、`rospy`、`tf2_ros`、`cv_bridge`、`message_filters` 都不可导入。

这说明当前阻塞点首先是 ROS Noetic 运行环境不存在，不是缺少 venv。ROS Noetic 的官方支持已在 2025-05-31 结束；项目 README 仍要求 Ubuntu 20.04 + ROS Noetic。[ROS Noetic EOL](https://www.ros.org/blog/noetic-eol/) [`README.md`](../../README.md#L59-L70)

### 建议

不用为了完成大部分代码工作而立刻创建 venv。下面这些工作不需要完整 ROS：

- 审查和修改 Python/XML/package 文件；
- 运行 `rgbd_localization/core.py` 的纯数学测试；
- 做 launch/package 静态检查；
- 在单独 Python 环境里做 Ultralytics 的模型加载/单张图片 smoke test。

但在真正运行 ROS 节点时，建议使用“Ubuntu 20.04 + ROS Noetic 容器/专用环境”，并可在其中创建项目 venv 来隔离 `ultralytics`、PyTorch、numpy 等 pip 依赖。Python 官方说明 venv 默认隔离系统 site-packages；`--system-site-packages` 才会暴露系统包。[Python `venv` 文档](https://docs.python.org/3.12/library/venv.html)

关键边界是：`rospy`、消息生成包、`cv_bridge`、`tf2_ros` 等来自 ROS apt/工作空间，不会因为创建 venv 自动出现。若在 ROS 环境中使用 venv，必须验证同一个解释器能同时导入 ROS 和 Ultralytics：

```bash
source /opt/ros/noetic/setup.bash
source .venv/bin/activate
python -c "import rospy, cv_bridge, tf2_ros; from ultralytics import YOLO"
```

如果这条命令失败，继续调整 venv 并不能替代安装/构建 ROS；应先修复 ROS 基础环境。由于当前主机是 Ubuntu 22.04，最省时间的全链路方案是把 ROS Noetic 放到 Ubuntu 20.04 容器、旧机器人电脑或专用虚拟机中，而不是在本机单独建 venv 试图“补出” Noetic。

## 最终执行顺序

```text
TF API 修复
  -> adapter fake-TF 测试
  -> 模型路径与权重解耦
  -> RGB/depth approximate sync
  -> launch/README/legacy 小整理
  -> 在 ROS Noetic 容器或机器人上做 rosbag/仿真验证
```

本轮完成后，代码层面的重构基本应停止；剩余主要工作应转为 ROS 环境、传感器时间戳、TF 树、模型推理性能和完整任务链验证。

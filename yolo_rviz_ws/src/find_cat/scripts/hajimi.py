#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
CatFollower + Coverage Explorer 融合节点 (Python, YOLOv8 内嵌推理)

整合点：
1. 保留第二份代码的覆盖率计算 + 探索状态机
2. 保留第一份代码的猫检测 → 停止探索 → 导航逻辑
3. STATE_EXPLORE 阶段：同时进行 YOLO 猫检测 与 覆盖率监控
4. 覆盖率达标：
   - 已确认猫 → 杀掉 explore → 清目标 → 导航到猫
   - 未确认猫 → 宣告失败
5. 导航阶段持续检查到达条件

话题名称 / 消息类型 / 相机模型投影方式 与第二份代码保持一致。
"""

import math
import subprocess
import threading
import queue
import numpy as np
import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image

# 尝试导入 ultralytics
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None
    rospy.logerr("Please install ultralytics: pip install ultralytics")


class CatFollowerWithCoverage:
    """
    融合节点：覆盖率探索 + 猫检测 + 导航到猫
    """

    # 状态枚举
    STATE_EXPLORE  = "EXPLORE"   # 探索中：建图 + 猫检测并行
    STATE_NAVIGATE = "NAVIGATE"  # 导航到猫
    STATE_SUCCESS  = "SUCCESS"   # 任务成功
    STATE_FAILED   = "FAILED"    # 任务失败

    def __init__(self):
        rospy.init_node("cat_follower_coverage", anonymous=False)

        # ---------- 加载参数 ----------
        self._load_params()

        # ---------- YOLO 模型 ----------
        if YOLO is None:
            rospy.signal_shutdown("YOLO not available")
            return
        self.model = YOLO(self.model_path)
        self.target_class_lower = self.target_class.lower()

        # ---------- 组件 ----------
        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self.cam_frame = ""
        self.have_caminfo = False

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # move_base 客户端
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        try:
            self.move_base_client.wait_for_server(rospy.Duration(5.0))
            rospy.loginfo("Connected to move_base")
        except Exception:
            rospy.logwarn("move_base not available yet, will retry later")

        # ---------- 线程安全数据 ----------
        self.lock = threading.Lock()

        # ---- 深度 + 地图 ----
        self.latest_depth = None          # 最新深度图 (np.ndarray)
        self.latest_map = None            # 最新占据栅格地图 (OccupancyGrid)

        # ---- 猫检测相关 ----
        self.cat_detections = []          # 候选猫位置列表 [np.array]
        self.confirmed_cat_pos = None     # 确认后的猫世界坐标 (np.array)
        self.killed_explore = False       # 是否已杀 explore

        # ---- 状态机 ----
        self.state = self.STATE_EXPLORE

        # RGB 队列（容量 1）
        self.rgb_queue = queue.Queue(maxsize=1)

        # ---------- 订阅 ----------
        rospy.Subscriber(self.rgb_topic, Image, self._rgb_cb, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self._depth_cb, queue_size=1)
        rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._caminfo_cb, queue_size=1
        )
        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_cb, queue_size=1)

        # ---------- 启动处理线程 ----------
        self.processing_thread = threading.Thread(target=self._processing_loop)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        # ---------- 主定时器：状态机调度 ----------
        self.timer = rospy.Timer(rospy.Duration(1.0), self._main_timer)

        rospy.loginfo(
            "CatFollowerWithCoverage ready. "
            "Exploring (target: cat '%s'), coverage threshold %.0f%%",
            self.target_class,
            self.coverage_threshold * 100,
        )

    # ==================================================================
    #   参数加载
    # ==================================================================
    def _load_params(self):
        """与第二份代码参数名保持一致，均可通过 ROS param 覆盖"""

        # --- YOLO ---
        self.model_path        = rospy.get_param("~yolo_model",         "yolov8n.pt")

        # --- 话题 ---
        self.rgb_topic         = rospy.get_param("~rgb_topic",          "/camera/rgb/image_raw")
        self.depth_topic       = rospy.get_param("~depth_topic",        "/camera/depth/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic",  "/camera/rgb/camera_info")
        self.map_topic         = rospy.get_param("~map_topic",          "/map")

        # --- 目标猫 ---
        self.target_class      = rospy.get_param("~target_class",       "cat")

        # --- 覆盖率 ---
        self.lab_width         = rospy.get_param("~lab_width_m",        5.1)
        self.lab_height        = rospy.get_param("~lab_height_m",       3.6)
        self.lab_area          = self.lab_width * self.lab_height
        self.coverage_threshold = rospy.get_param("~coverage_threshold", 0.9)

        # --- 猫检测确认 ---
        self.confirm_count     = rospy.get_param("~confirm_count",      3)
        self.dist_threshold    = rospy.get_param("~dist_threshold",     0.5)
        self.max_detect_dist   = rospy.get_param("~max_detect_dist",    4.0)

        # --- 导航 ---
        self.arrival_dist      = rospy.get_param("~arrival_distance_m", 0.8)
        self.target_offset     = rospy.get_param("~target_offset",      0.5)

        # --- 其他 ---
        self.explore_node_name = rospy.get_param("~explore_node_name",  "/explore")
        self.base_frame        = rospy.get_param("~base_frame",          "base_link")

    # ==================================================================
    #   回调 (CameraInfo / Depth / RGB / Map)
    # ==================================================================
    def _caminfo_cb(self, msg):
        with self.lock:
            self.cam_model.fromCameraInfo(msg)
            self.cam_frame = msg.header.frame_id
            self.have_caminfo = True

    def _depth_cb(self, msg):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            with self.lock:
                self.latest_depth = cv_depth
        except Exception:
            pass

    def _rgb_cb(self, msg):
        # 队列满则丢弃旧帧
        if self.rgb_queue.full():
            _ = self.rgb_queue.get()
        self.rgb_queue.put(msg)

    def _map_cb(self, msg):
        with self.lock:
            self.latest_map = msg

    # ==================================================================
    #   处理线程（YOLO 推理 + 猫确认）—— 仅在 EXPLORE 阶段运行
    # ==================================================================
    def _processing_loop(self):
        """独立线程：取 RGB，推理 YOLO，确认猫"""
        while not rospy.is_shutdown():
            try:
                msg = self.rgb_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            # 仅探索阶段处理检测
            if self.state != self.STATE_EXPLORE:
                continue

            # 已确认猫则跳过
            with self.lock:
                if self.confirmed_cat_pos is not None:
                    continue
                if not self.have_caminfo or self.latest_depth is None:
                    continue
                depth_snap = self.latest_depth.copy()
                cam_frame = self.cam_frame
                cam_model = self.cam_model   # 只读引用

            # 图像转换
            try:
                rgb_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                continue

            # YOLO 推理
            try:
                results = self.model.predict(rgb_cv, conf=0.5, verbose=False)[0]
            except Exception:
                continue

            if results.boxes is None:
                continue

            for box in results.boxes:
                cls_id = int(box.cls.item())
                label = self.model.names.get(cls_id, str(cls_id)).lower()
                if label != self.target_class_lower:
                    continue

                # 中心像素
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                u = int((xyxy[0] + xyxy[2]) / 2)
                v = int((xyxy[1] + xyxy[3]) / 2)

                # 深度值
                z = self._get_depth_val(depth_snap, u, v)
                if z is None or z > self.max_detect_dist or z < 0.3:
                    continue

                # 相机模型投影 → 相机坐标系 3D
                ray = cam_model.projectPixelTo3dRay((u, v))
                s = z / ray[2]
                p_cam = PointStamped()
                p_cam.header.frame_id = cam_frame
                p_cam.header.stamp = rospy.Time(0)
                p_cam.point.x = ray[0] * s
                p_cam.point.y = ray[1] * s
                p_cam.point.z = ray[2] * s

                # TF 到 map
                try:
                    p_map = self.tf_buffer.transform(
                        p_cam, "map", rospy.Duration(0.3)
                    )
                except Exception:
                    continue

                # 送入确认机制
                self._confirm_cat(
                    np.array([p_map.point.x, p_map.point.y])
                )

    # ==================================================================
    #   深度值获取（鲁棒搜索）—— 与第二份代码一致
    # ==================================================================
    def _get_depth_val(self, depth_img, u, v):
        """返回深度值（米），兼容 16UC1(mm) 和 32FC1(m)"""
        h, w = depth_img.shape[:2]
        u = max(0, min(w - 1, u))
        v = max(0, min(h - 1, v))

        def valid(d):
            if depth_img.dtype == np.uint16:
                return d > 0
            else:
                return d > 0 and not np.isnan(d)

        # 先试中心点
        z = depth_img[v, u]
        if valid(z):
            if depth_img.dtype == np.uint16:
                z_m = float(z) * 0.001
            else:
                z_m = float(z)
            return z_m

        # 周围搜索
        for r in range(1, 6):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = u + dx, v + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        val = depth_img[ny, nx]
                        if valid(val):
                            if depth_img.dtype == np.uint16:
                                z_m = float(val) * 0.001
                            else:
                                z_m = float(val)
                            return z_m
        return None

    # ==================================================================
    #   猫确认机制 —— 与第二份代码逻辑对齐
    # ==================================================================
    def _confirm_cat(self, new_pos):
        """
        平滑 + 连续计数确认。
        确认后设置 confirmed_cat_pos，但不在此处切换状态；
        等待覆盖率达标后由主状态机统一处理。
        """
        with self.lock:
            if self.confirmed_cat_pos is not None:
                return  # 已经确认

            # 检查是否与已有候选接近
            found_nearby = False
            for det in self.cat_detections:
                if np.linalg.norm(new_pos - det) < self.dist_threshold:
                    found_nearby = True
                    break

            if found_nearby:
                self.cat_detections.append(new_pos)
            else:
                # 新候选组，重置
                self.cat_detections = [new_pos]

            # 检查是否达到确认阈值
            if len(self.cat_detections) >= self.confirm_count:
                avg = np.mean(self.cat_detections, axis=0)
                self.confirmed_cat_pos = avg
                rospy.loginfo(
                    "!!! CAT CONFIRMED at (%.2f, %.2f) !!! "
                    "(waiting for coverage threshold)",
                    avg[0], avg[1],
                )
                # 注意：不在此处切换状态/杀 explore
                # 等待覆盖率达标后由 _main_timer 统一处理

    # ==================================================================
    #   覆盖率计算 —— 与第二份代码一致
    # ==================================================================
    def _compute_coverage(self):
        """计算当前地图的覆盖率（0.0 ~ 1.0）"""
        with self.lock:
            grid = self.latest_map
        if grid is None:
            return 0.0
        data = np.array(grid.data, dtype=np.int8)
        known = np.count_nonzero(data != -1)  # 已知栅格数
        resolution = grid.info.resolution
        known_area = known * (resolution * resolution)
        return min(1.0, known_area / self.lab_area)

    # ==================================================================
    #   主定时器：状态机调度
    # ==================================================================
    def _main_timer(self, event):
        """
        每秒执行一次状态机调度：
        - EXPLORE:  检查覆盖率，达标后根据猫确认情况切换状态
        - NAVIGATE: 发送导航目标 / 检查到达
        """

        # ========== STATE_EXPLORE ==========
        if self.state == self.STATE_EXPLORE:
            cov = self._compute_coverage()
            rospy.loginfo_throttle(5, "Coverage: %.1f%%", cov * 100)

            if cov >= self.coverage_threshold:
                # 覆盖率已达标
                with self.lock:
                    cat_pt = (
                        self.confirmed_cat_pos.copy()
                        if self.confirmed_cat_pos is not None
                        else None
                    )

                if cat_pt is not None:
                    # 找到猫 → 进入导航
                    rospy.loginfo(
                        "Coverage reached (%.1f%%) with cat found. "
                        "Starting navigation.",
                        cov * 100,
                    )
                    self.state = self.STATE_NAVIGATE
                    # 停止探索 + 清空旧目标
                    self._stop_explore_and_clear()
                    # 立即发送第一个导航点
                    self._send_nav_goal(cat_pt)
                else:
                    # 覆盖率够了但没找到猫 → 失败
                    self._trigger_failure(
                        "Coverage threshold reached (%.1f%%), "
                        "but cat not found." % (cov * 100)
                    )
                return

        # ========== STATE_NAVIGATE ==========
        elif self.state == self.STATE_NAVIGATE:
            with self.lock:
                cat_pt = (
                    self.confirmed_cat_pos.copy()
                    if self.confirmed_cat_pos is not None
                    else None
                )
            if cat_pt is None:
                return

            # 检查到达
            dist = self._robot_dist_to(cat_pt)
            if dist is not None and dist < self.arrival_dist:
                rospy.loginfo("Arrived at cat! Mission success.")
                self.state = self.STATE_SUCCESS
                self.move_base_client.cancel_all_goals()
                rospy.signal_shutdown("Success")
                return

            # 尝试发送导航目标（内部防重复）
            self._send_nav_goal(cat_pt)

    # ==================================================================
    #   停止探索 & 清除旧目标
    # ==================================================================
    def _stop_explore_and_clear(self):
        """杀掉 explore 节点，清空 move_base 目标队列（仅执行一次）"""
        if self.killed_explore:
            return

        # 1) 杀 explore
        rospy.loginfo("Stopping explore node: %s ...", self.explore_node_name)
        try:
            subprocess.run(
                ["rosnode", "kill", self.explore_node_name],
                capture_output=True, timeout=3.0
            )
            self.killed_explore = True
            rospy.loginfo("Explore node killed.")
        except Exception as e:
            rospy.logwarn("Failed to kill explore: %s. Proceeding anyway.", e)
            self.killed_explore = True

        # 2) 清空 move_base 目标队列
        rospy.loginfo("Cancelling all move_base goals.")
        self.move_base_client.cancel_all_goals()

    # ==================================================================
    #   任务失败
    # ==================================================================
    def _trigger_failure(self, reason):
        """宣告任务失败并关机"""
        rospy.logerr("Mission failed: %s", reason)
        self.state = self.STATE_FAILED
        self.move_base_client.cancel_all_goals()
        # 杀 explore（如果还没杀）
        if not self.killed_explore:
            try:
                subprocess.run(
                    ["rosnode", "kill", self.explore_node_name],
                    capture_output=True, timeout=3.0
                )
                self.killed_explore = True
            except Exception:
                pass
        rospy.signal_shutdown(reason)

    # ==================================================================
    #   机器人位置辅助
    # ==================================================================
    def _robot_pos(self):
        """返回机器人在 map 系坐标 np.array([x,y]) 或 None"""
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", self.base_frame, rospy.Time(0)
            )
            return np.array([
                tf.transform.translation.x,
                tf.transform.translation.y,
            ])
        except Exception as e:
            rospy.logwarn_throttle(5, "Failed to get robot pos: %s", e)
            return None

    def _robot_dist_to(self, pt):
        """返回机器人到目标点的水平距离（米）"""
        rpos = self._robot_pos()
        if rpos is None:
            return None
        return float(np.linalg.norm(pt - rpos))

    # ==================================================================
    #   发送导航目标
    # ==================================================================
    def _send_nav_goal(self, cat_pos):
        """
        发送导航目标到猫前方 target_offset 米处。
        仅当 move_base 无活动目标时才发送，避免规划频繁重置。
        """
        # 避免覆盖正在执行的目标
        state = self.move_base_client.get_state()
        if state in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
            rospy.logdebug("Navigation goal already active, skipping resend.")
            return

        rpos = self._robot_pos()
        if rpos is None:
            rospy.logwarn("Cannot send goal: robot pose unknown")
            return

        # 计算方向
        dx = cat_pos[0] - rpos[0]
        dy = cat_pos[1] - rpos[1]
        dist = math.hypot(dx, dy)
        if dist < 0.01:
            rospy.loginfo("Already at cat position, no goal sent.")
            return

        dir_x = dx / dist
        dir_y = dy / dist

        # 目标位姿 = 猫位置 - 方向 * offset（停在猫前方）
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = cat_pos[0] - dir_x * self.target_offset
        goal.target_pose.pose.position.y = cat_pos[1] - dir_y * self.target_offset
        goal.target_pose.pose.position.z = 0.0

        # 朝向猫
        yaw = math.atan2(dy, dx)
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.move_base_client.send_goal(goal)
        rospy.loginfo(
            "Sent goal to (%.2f, %.2f), yaw=%.2f rad",
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y,
            yaw,
        )


# ======================================================================
#   main
# ======================================================================
if __name__ == "__main__":
    try:
        CatFollowerWithCoverage()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

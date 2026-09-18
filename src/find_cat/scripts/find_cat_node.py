#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Explore a mapped area, confirm a cat detection, and navigate to it."""

import math
import queue
import subprocess
import threading

import numpy as np
import rospy
import actionlib
import tf2_geometry_msgs  # noqa: F401 - registers geometry message conversions
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class FindCatNode:
    """Coverage monitor, embedded YOLO detector, and move_base controller."""

    EXPLORE = "EXPLORE"
    NAVIGATE = "NAVIGATE"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"

    def __init__(self):
        rospy.init_node("find_cat")
        self._load_params()

        if YOLO is None:
            rospy.logerr("ultralytics is not installed; install requirements.txt")
            rospy.signal_shutdown("missing ultralytics")
            return

        try:
            self.model = YOLO(self.model_path)
        except Exception as exc:
            rospy.logerr("Unable to load YOLO model '%s': %s", self.model_path, exc)
            rospy.signal_shutdown("unable to load YOLO model")
            return

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_frame = ""
        self.have_camera_info = False
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        if self.move_base.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Connected to move_base")
        else:
            rospy.logwarn("move_base is not available yet; navigation will retry")

        self.lock = threading.Lock()
        self.latest_depth = None
        self.latest_map = None
        self.cat_candidates = []
        self.confirmed_cat = None
        self.explore_stopped = False
        self.state = self.EXPLORE

        self.rgb_queue = queue.Queue(maxsize=1)
        rospy.Subscriber(self.rgb_topic, Image, self._rgb_callback, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self._depth_callback, queue_size=1)
        rospy.Subscriber(
            self.camera_info_topic,
            CameraInfo,
            self._camera_info_callback,
            queue_size=1,
        )
        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_callback, queue_size=1)

        self.processing_thread = threading.Thread(
            target=self._processing_loop,
            name="cat-detection",
            daemon=True,
        )
        self.processing_thread.start()
        self.timer = rospy.Timer(rospy.Duration(1.0), self._state_machine)

        rospy.loginfo(
            "find_cat ready: target=%s, coverage threshold=%.1f%%",
            self.target_class,
            self.coverage_threshold * 100.0,
        )

    def _load_params(self):
        """Load the public ROS parameters used by this node."""

        self.model_path = rospy.get_param("~model_path", "yolov8s.pt")
        self.inference_confidence = float(
            rospy.get_param("~inference_confidence", 0.5)
        )
        self.target_class = str(rospy.get_param("~target_class", "cat")).lower()

        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/depth/image_rect_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/rgb/camera_info"
        )
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.explore_node_name = rospy.get_param("~explore_node_name", "/explore")

        self.coverage_threshold = float(
            rospy.get_param("~coverage_threshold", 0.95)
        )
        self.coverage_origin_x = float(
            rospy.get_param("~coverage_origin_x", 0.15)
        )
        self.coverage_origin_y = float(
            rospy.get_param("~coverage_origin_y", 0.15)
        )
        self.coverage_width = float(rospy.get_param("~coverage_width_m", 3.60))
        self.coverage_height = float(rospy.get_param("~coverage_height_m", 5.80))

        self.confirmation_hits = int(rospy.get_param("~confirmation_hits", 3))
        self.cluster_distance = float(
            rospy.get_param("~cluster_distance_m", 0.30)
        )
        self.depth_min = float(rospy.get_param("~depth_min_m", 0.20))
        self.depth_max = float(rospy.get_param("~depth_max_m", 6.0))
        self.arrival_distance = float(
            rospy.get_param("~arrival_distance_m", 0.50)
        )
        self.target_offset = float(rospy.get_param("~target_offset_m", 0.50))

    def _camera_info_callback(self, msg):
        with self.lock:
            self.camera_model.fromCameraInfo(msg)
            self.camera_frame = msg.header.frame_id
            self.have_camera_info = True

    def _depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Depth conversion failed: %s", exc)
            return
        with self.lock:
            self.latest_depth = depth

    def _rgb_callback(self, msg):
        try:
            if self.rgb_queue.full():
                self.rgb_queue.get_nowait()
            self.rgb_queue.put_nowait(msg)
        except (queue.Empty, queue.Full):
            pass

    def _map_callback(self, msg):
        with self.lock:
            self.latest_map = msg

    def _processing_loop(self):
        while not rospy.is_shutdown():
            try:
                image_msg = self.rgb_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            with self.lock:
                if self.state != self.EXPLORE or self.confirmed_cat is not None:
                    continue
                if not self.have_camera_info or self.latest_depth is None:
                    continue
                depth = self.latest_depth.copy()
                camera_frame = self.camera_frame
                camera_model = self.camera_model

            try:
                image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
                result = self.model.predict(
                    image,
                    conf=self.inference_confidence,
                    verbose=False,
                )[0]
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "Cat image processing failed: %s", exc)
                continue

            if result.boxes is None:
                continue

            for box in result.boxes:
                if self._class_name(box, self.model.names) != self.target_class:
                    continue

                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                u = int((xyxy[0] + xyxy[2]) / 2)
                v = int((xyxy[1] + xyxy[3]) / 2)
                depth_m = self._depth_value(depth, u, v)
                if depth_m is None or not self.depth_min <= depth_m <= self.depth_max:
                    continue

                ray = camera_model.projectPixelTo3dRay((u, v))
                scale = depth_m / ray[2]
                point_camera = PointStamped()
                point_camera.header.frame_id = camera_frame
                point_camera.header.stamp = rospy.Time(0)
                point_camera.point.x = ray[0] * scale
                point_camera.point.y = ray[1] * scale
                point_camera.point.z = ray[2] * scale

                try:
                    point_map = self.tf_buffer.transform(
                        point_camera,
                        self.target_frame,
                        rospy.Duration(0.3),
                    )
                except Exception as exc:
                    rospy.logdebug("Cat TF transform failed: %s", exc)
                    continue

                self._confirm_cat(
                    np.array([point_map.point.x, point_map.point.y], dtype=float)
                )

    @staticmethod
    def _class_name(box, names):
        class_id = int(box.cls.item())
        if isinstance(names, dict):
            return str(names.get(class_id, class_id)).lower()
        if 0 <= class_id < len(names):
            return str(names[class_id]).lower()
        return str(class_id)

    @staticmethod
    def _depth_value(depth, u, v):
        height, width = depth.shape[:2]
        u = max(0, min(width - 1, u))
        v = max(0, min(height - 1, v))

        def as_meters(value):
            if depth.dtype == np.uint16:
                return float(value) * 0.001 if value > 0 else None
            value = float(value)
            return value if np.isfinite(value) and value > 0 else None

        value = as_meters(depth[v, u])
        if value is not None:
            return value

        for radius in range(1, 6):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    x, y = u + dx, v + dy
                    if 0 <= x < width and 0 <= y < height:
                        value = as_meters(depth[y, x])
                        if value is not None:
                            return value
        return None

    def _confirm_cat(self, new_position):
        with self.lock:
            if self.confirmed_cat is not None:
                return

            nearby = any(
                np.linalg.norm(new_position - candidate) < self.cluster_distance
                for candidate in self.cat_candidates
            )
            if nearby:
                self.cat_candidates.append(new_position)
            else:
                self.cat_candidates = [new_position]

            if len(self.cat_candidates) >= self.confirmation_hits:
                self.confirmed_cat = np.mean(self.cat_candidates, axis=0)
                rospy.loginfo(
                    "Cat confirmed at (%.2f, %.2f); waiting for map coverage",
                    self.confirmed_cat[0],
                    self.confirmed_cat[1],
                )

    def _compute_coverage(self):
        with self.lock:
            grid = self.latest_map
        if grid is None or grid.info.resolution <= 0:
            return 0.0

        resolution = grid.info.resolution
        width = grid.info.width
        height = grid.info.height
        data = np.asarray(grid.data, dtype=np.int8).reshape((height, width))
        min_x = self.coverage_origin_x
        min_y = self.coverage_origin_y
        max_x = min_x + self.coverage_width
        max_y = min_y + self.coverage_height
        map_x = grid.info.origin.position.x
        map_y = grid.info.origin.position.y

        x0 = max(0, int(math.floor((min_x - map_x) / resolution)))
        y0 = max(0, int(math.floor((min_y - map_y) / resolution)))
        x1 = min(width, int(math.ceil((max_x - map_x) / resolution)))
        y1 = min(height, int(math.ceil((max_y - map_y) / resolution)))
        if x0 >= x1 or y0 >= y1:
            return 0.0

        roi = data[y0:y1, x0:x1]
        return float(np.count_nonzero(roi != -1)) / float(roi.size)

    def _state_machine(self, _event):
        if self.state == self.EXPLORE:
            coverage = self._compute_coverage()
            rospy.loginfo_throttle(5.0, "Map coverage: %.1f%%", coverage * 100.0)
            if coverage < self.coverage_threshold:
                return

            with self.lock:
                cat_position = (
                    self.confirmed_cat.copy() if self.confirmed_cat is not None else None
                )
            if cat_position is None:
                self._fail("coverage threshold reached, but no cat was confirmed")
                return

            self.state = self.NAVIGATE
            self._stop_exploration()
            self._send_navigation_goal(cat_position)
            return

        if self.state != self.NAVIGATE:
            return

        with self.lock:
            cat_position = (
                self.confirmed_cat.copy() if self.confirmed_cat is not None else None
            )
        if cat_position is None:
            return

        distance = self._robot_distance(cat_position)
        if distance is not None and distance <= self.arrival_distance:
            rospy.loginfo("Arrived at the cat; mission succeeded")
            self.state = self.SUCCESS
            self.move_base.cancel_all_goals()
            rospy.signal_shutdown("cat found")
            return
        self._send_navigation_goal(cat_position)

    def _stop_exploration(self):
        if self.explore_stopped:
            return
        self.explore_stopped = True
        self.move_base.cancel_all_goals()
        if not self.explore_node_name:
            return
        try:
            subprocess.run(
                ["rosnode", "kill", self.explore_node_name],
                check=False,
                capture_output=True,
                timeout=3.0,
            )
            rospy.loginfo("Stopped explore node %s", self.explore_node_name)
        except (OSError, subprocess.SubprocessError) as exc:
            rospy.logwarn("Could not stop explore node: %s", exc)

    def _fail(self, reason):
        if self.state == self.FAILED:
            return
        rospy.logerr("Mission failed: %s", reason)
        self.state = self.FAILED
        self.move_base.cancel_all_goals()
        self._stop_exploration()
        rospy.signal_shutdown(reason)

    def _robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.2),
            )
            return np.array(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                ],
                dtype=float,
            )
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Robot pose unavailable: %s", exc)
            return None

    def _robot_distance(self, target):
        position = self._robot_position()
        return None if position is None else float(np.linalg.norm(target - position))

    def _send_navigation_goal(self, cat_position):
        if self.move_base.get_state() in (GoalStatus.ACTIVE, GoalStatus.PENDING):
            return

        robot_position = self._robot_position()
        if robot_position is None:
            return

        dx = cat_position[0] - robot_position[0]
        dy = cat_position[1] - robot_position[1]
        distance = math.hypot(dx, dy)
        if distance < 0.01:
            return

        direction_x = dx / distance
        direction_y = dy / distance
        yaw = math.atan2(dy, dx)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.target_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = (
            cat_position[0] - direction_x * self.target_offset
        )
        goal.target_pose.pose.position.y = (
            cat_position[1] - direction_y * self.target_offset
        )
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.move_base.send_goal(goal)
        rospy.loginfo(
            "Navigating to cat approach point (%.2f, %.2f)",
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y,
        )


if __name__ == "__main__":
    try:
        FindCatNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

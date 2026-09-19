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
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image
from rgbd_localization import RgbdLocalizer

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
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.localizer = RgbdLocalizer(self.tf_buffer, rospy.Duration(0.3))

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        if self.move_base.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Connected to move_base")
        else:
            rospy.logwarn("move_base is not available yet; navigation will retry")

        self.lock = threading.Lock()
        self.latest_map = None
        self.latest_depth = None
        self.cat_candidates = []
        self.confirmed_cat = None
        self.explore_stopped = False
        self.state = self.EXPLORE

        self.rgb_queue = queue.Queue(maxsize=1)
        rospy.Subscriber(
            self.rgb_topic,
            Image,
            self._rgb_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.depth_topic,
            Image,
            self._depth_callback,
            queue_size=1,
        )
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

        self.model_path = rospy.get_param("~model_path", "yolo26s.pt")
        self.inference_confidence = float(
            rospy.get_param("~inference_confidence", 0.5)
        )
        self.target_class = str(rospy.get_param("~target_class", "cat")).lower()

        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/rgb/camera_info"
        )
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        # m-explore uses ``/explore``.  The final Spark snapshot launches the
        # frontier_exploration client/server as ``/explore_client`` and
        # ``/explore_server``.  Keep the old single-name parameter working,
        # while making the platform runtime work out of the box too.
        configured_nodes = rospy.get_param("~explore_node_names", None)
        if configured_nodes is None:
            legacy_name = rospy.get_param("~explore_node_name", "/explore")
            configured_nodes = [legacy_name]
            if legacy_name == "/explore":
                configured_nodes.extend(("/explore_client", "/explore_server"))
        elif isinstance(configured_nodes, str):
            configured_nodes = [name.strip() for name in configured_nodes.split(",")]
        self.explore_node_names = [str(name) for name in configured_nodes if str(name)]
        self.coverage_threshold = float(
            rospy.get_param("~coverage_threshold", 0.9)
        )
        self.lab_width = float(rospy.get_param("~lab_width_m", 5.1))
        self.lab_height = float(rospy.get_param("~lab_height_m", 3.6))
        self.lab_area = self.lab_width * self.lab_height

        self.confirmation_hits = int(rospy.get_param("~confirmation_hits", 3))
        self.cluster_distance = float(rospy.get_param("~cluster_distance_m", 0.5))
        self.depth_min = float(rospy.get_param("~depth_min_m", 0.3))
        self.depth_max = float(rospy.get_param("~depth_max_m", 4.0))
        self.arrival_distance = float(
            rospy.get_param("~arrival_distance_m", 0.50)
        )
        self.target_offset = float(rospy.get_param("~target_offset_m", 0.50))

    def _camera_info_callback(self, msg):
        with self.lock:
            self.localizer.update_camera_info(msg)

    def _depth_callback(self, msg):
        try:
            depth = self.localizer.depth_image(msg)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Depth conversion failed: %s", exc)
            return
        with self.lock:
            self.latest_depth = depth

    def _rgb_callback(self, rgb_msg):
        with self.lock:
            if not self.localizer.ready or self.latest_depth is None:
                return
            depth = self.latest_depth.copy()
            camera_frame = self.localizer.camera_frame
        try:
            if self.rgb_queue.full():
                self.rgb_queue.get_nowait()
            self.rgb_queue.put_nowait((rgb_msg, depth, camera_frame))
        except (queue.Empty, queue.Full):
            pass

    def _map_callback(self, msg):
        with self.lock:
            self.latest_map = msg

    def _processing_loop(self):
        while not rospy.is_shutdown():
            try:
                image_msg, depth, camera_frame = self.rgb_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            with self.lock:
                if self.state != self.EXPLORE or self.confirmed_cat is not None:
                    continue

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
                point_map = self.localizer.localize(
                    depth,
                    (u, v),
                    self.target_frame,
                    rospy.Time(0),
                    camera_frame,
                    self.depth_min,
                    self.depth_max,
                )
                if point_map is None:
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
        known = np.count_nonzero(data != -1)
        known_area = known * resolution * resolution
        return min(1.0, float(known_area) / self.lab_area)

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
            # Allow the exploration node and move_base to finish clearing
            # their previous goals before submitting the cat goal.
            rospy.sleep(3.0)
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
        if not self.explore_node_names:
            return
        for node_name in self.explore_node_names:
            try:
                result = subprocess.run(
                    ["rosnode", "kill", node_name],
                    check=False,
                    capture_output=True,
                    timeout=3.0,
                )
                if result.returncode == 0:
                    rospy.loginfo("Stopped explore node %s", node_name)
            except (OSError, subprocess.SubprocessError) as exc:
                rospy.logwarn("Could not stop explore node %s: %s", node_name, exc)

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

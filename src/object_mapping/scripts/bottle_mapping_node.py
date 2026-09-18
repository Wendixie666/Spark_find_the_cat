#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Detect bottles with RGB-D, map them into ``map``, and keep RViz markers."""

import threading

import numpy as np
import rospy
import tf2_geometry_msgs  # noqa: F401 - registers geometry message conversions
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from object_mapping.msg import DetectedObject, DetectedObjectArray

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class BottleMappingNode:
    """YOLO bottle detector with spatial confirmation and persistent markers."""

    def __init__(self):
        rospy.init_node("bottle_mapping")
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
        self.depth_image = None
        self.lock = threading.Lock()

        self.bottle_tracks = []
        self.next_marker_id = 0
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(self.rgb_topic, Image, self._rgb_callback, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self._depth_callback, queue_size=1)
        rospy.Subscriber(
            self.camera_info_topic,
            CameraInfo,
            self._camera_info_callback,
            queue_size=1,
        )
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=50)
        self.detected_pub = rospy.Publisher(
            self.detected_topic,
            DetectedObjectArray,
            queue_size=5,
        )

        rospy.loginfo(
            "bottle_mapping ready: class=%s, cluster=%.2fm, confirmation=%d hits",
            self.target_class,
            self.cluster_distance,
            self.confirmation_hits,
        )
        rospy.spin()

    def _load_params(self):
        self.model_path = rospy.get_param("~model_path", "yolov8s.pt")
        self.inference_confidence = float(
            rospy.get_param("~inference_confidence", 0.5)
        )
        self.target_class = str(rospy.get_param("~target_class", "bottle")).lower()
        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/depth/image_rect_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/rgb/camera_info"
        )
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/visualization_marker"
        )
        self.detected_topic = rospy.get_param(
            "~detected_topic", "/detected_objects"
        )
        self.depth_min = float(rospy.get_param("~depth_min_m", 0.20))
        self.depth_max = float(rospy.get_param("~depth_max_m", 6.0))
        self.cluster_distance = float(
            rospy.get_param("~cluster_distance_m", 0.50)
        )
        self.confirmation_hits = int(rospy.get_param("~confirmation_hits", 2))
        self.smoothing_alpha = float(rospy.get_param("~smoothing_alpha", 0.20))
        self.marker_scale = float(rospy.get_param("~marker_scale_m", 0.15))
        self.label_height = float(rospy.get_param("~label_height_m", 0.20))

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
            self.depth_image = depth

    def _rgb_callback(self, msg):
        with self.lock:
            if not self.have_camera_info or self.depth_image is None:
                return
            depth = self.depth_image.copy()
            camera_frame = self.camera_frame
            camera_model = self.camera_model

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            result = self.model.predict(
                image,
                conf=self.inference_confidence,
                verbose=False,
            )[0]
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Bottle image processing failed: %s", exc)
            return

        detections = DetectedObjectArray()
        detections.header = msg.header
        if result.boxes is None:
            return

        for box in result.boxes:
            label = self._class_name(box, self.model.names)
            if label != self.target_class:
                continue

            confidence = float(box.conf.item())
            xmin, ymin, xmax, ymax = (
                box.xyxy[0].cpu().numpy().astype(int).tolist()
            )
            u = int((xmin + xmax) / 2.0)
            v = int((ymin + ymax) / 2.0)
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
                    rospy.Duration(0.5),
                )
            except Exception as exc:
                rospy.logdebug("Bottle TF transform failed: %s", exc)
                continue

            object_id, position = self._process_detection(point_map.point)
            self._publish_markers(object_id, position, msg.header.stamp)

            detection = DetectedObject()
            detection.header = msg.header
            detection.class_name = label
            detection.confidence = confidence
            detection.position = point_map.point
            detection.robot_pose = self._robot_pose(msg.header.stamp)
            detection.xmin = xmin
            detection.ymin = ymin
            detection.xmax = xmax
            detection.ymax = ymax
            detections.objects.append(detection)

        if detections.objects:
            self.detected_pub.publish(detections)

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

    def _process_detection(self, point):
        new_position = np.array([point.x, point.y, point.z], dtype=float)
        for track in self.bottle_tracks:
            if np.linalg.norm(new_position - track["position"]) > self.cluster_distance:
                continue

            track["position"] = (
                (1.0 - self.smoothing_alpha) * track["position"]
                + self.smoothing_alpha * new_position
            )
            track["hits"] += 1
            if track["hits"] == self.confirmation_hits:
                rospy.loginfo(
                    "Confirmed Bottle_%d at (%.2f, %.2f, %.2f)",
                    track["id"],
                    *track["position"],
                )
            return track["id"], track["position"]

        track = {
            "id": self.next_marker_id,
            "position": new_position,
            "hits": 1,
        }
        self.next_marker_id += 1
        self.bottle_tracks.append(track)
        return track["id"], track["position"]

    def _publish_markers(self, object_id, position, stamp):
        track = next(track for track in self.bottle_tracks if track["id"] == object_id)
        if track["hits"] < self.confirmation_hits:
            return

        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = stamp
        marker.ns = "stable_bottles"
        marker.id = object_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = self.marker_scale
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        marker.lifetime = rospy.Duration(0)
        self.marker_pub.publish(marker)

        label = Marker()
        label.header = marker.header
        label.ns = "stable_labels"
        label.id = object_id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = position[0]
        label.pose.position.y = position[1]
        label.pose.position.z = position[2] + self.label_height
        label.pose.orientation.w = 1.0
        label.scale.z = 0.12
        label.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        label.text = "Bottle_{}".format(object_id)
        label.lifetime = rospy.Duration(0)
        self.marker_pub.publish(label)

    def _robot_pose(self, stamp):
        pose = Pose()
        pose.orientation.w = 1.0
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                stamp,
                rospy.Duration(0.1),
            )
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
        except Exception:
            pass
        return pose


if __name__ == "__main__":
    try:
        BottleMappingNode()
    except rospy.ROSInterruptException:
        pass

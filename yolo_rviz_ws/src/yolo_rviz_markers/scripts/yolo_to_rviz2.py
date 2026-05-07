#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Pose
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import numpy as np
import math

from yolo_rviz_markers.msg import DetectedObject, DetectedObjectArray

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None
##这一份文件是初始的没有打印坐标的版本，后续在yolo_to_rviz.py基础上增加了坐标打印功能，并且改了一些细节。

class YoloToRviz(object):
    def __init__(self):
        rospy.init_node('yolo_to_rviz')
        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self.have_caminfo = False
        self.depth_cv = None
        self.robot_pose = None

        rgb_topic = rospy.get_param('~rgb_topic', '/camera/rgb/image_raw')
        depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        caminfo_topic = rospy.get_param('~camera_info_topic', '/camera/rgb/camera_info')
        odom_topic = rospy.get_param('~odom_topic', '/rtabmap/odom')
        self.target_frame = rospy.get_param('~target_frame', 'map')
        self.marker_topic = rospy.get_param('~marker_topic', '/visualization_marker')
        self.detected_topic = rospy.get_param('~detected_topic', '/detected_objects')
        self.target_class = rospy.get_param('~target_class', 'bottle').lower()
        model_path = rospy.get_param('~yolo_model', 'yolov8n.pt')

        if YOLO is None:
            rospy.logerr('ultralytics not available. Please install python package ultralytics in current ROS python environment.')
            rospy.signal_shutdown('missing ultralytics')
            return

        self.model = YOLO(model_path)

        rospy.Subscriber(rgb_topic, Image, self.rgb_cb, queue_size=1)
        rospy.Subscriber(depth_topic, Image, self.depth_cb, queue_size=1)
        rospy.Subscriber(caminfo_topic, CameraInfo, self.caminfo_cb, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_cb, queue_size=5)

        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=30)
        self.detected_pub = rospy.Publisher(self.detected_topic, DetectedObjectArray, queue_size=5)
        self.tfbuf = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfbuf)

        self.last_marker_ids = set()

        rospy.loginfo('yolo_to_rviz started')
        rospy.spin()

    def caminfo_cb(self, msg):
        if not self.have_caminfo:
            self.cam_model.fromCameraInfo(msg)
            self.cam_frame = msg.header.frame_id
            self.have_caminfo = True

    def depth_cb(self, msg):
        try:
            self.depth_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logwarn('Depth convert failed: %s', e)
            self.depth_cv = None

    def odom_cb(self, msg):
        self.robot_pose = msg.pose.pose

    def rgb_cb(self, msg):
        if not self.have_caminfo or self.depth_cv is None:
            return

        try:
            rgb_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn('RGB convert failed: %s', e)
            return

        try:
            yolo_result = self.model.predict(rgb_cv, verbose=False)[0]
        except Exception as e:
            rospy.logwarn('YOLO inference failed: %s', e)
            return

        detections = DetectedObjectArray()
        detections.header = msg.header
        live_ids = set()

        if yolo_result.boxes is None:
            #self._clear_stale_markers(msg.header, live_ids)
            return

        idx = 0
        for box in yolo_result.boxes:
            cls_id = int(box.cls.item())
            label = self.model.names.get(cls_id, str(cls_id)).lower()
            if label != self.target_class:
                continue

            conf = float(box.conf.item())
            xyxy = box.xyxy[0].cpu().numpy().astype(int)
            xmin, ymin, xmax, ymax = xyxy.tolist()

            u = int((xmin + xmax) / 2.0)
            v = int((ymin + ymax) / 2.0)

            h, w = self.depth_cv.shape[:2]
            u = max(0, min(w - 1, u))
            v = max(0, min(h - 1, v))

            z = float(self.depth_cv[v, u])
            if self.depth_cv.dtype == np.uint16:
                if z == 0:
                    z = self._search_valid_depth_uint16(self.depth_cv, u, v)
                if z is not None:
                    z = z * 0.001
            else:
                if z == 0 or math.isnan(z):
                    z = self._search_valid_depth_float(self.depth_cv, u, v)
            if z is None or z == 0 or math.isnan(z):
                continue

            ray = self.cam_model.projectPixelTo3dRay((u, v))
            scale = z / ray[2]
            x_cam = ray[0] * scale
            y_cam = ray[1] * scale
            z_cam = ray[2] * scale

            p_cam = PointStamped()
            p_cam.header.frame_id = self.cam_frame
            p_cam.header.stamp = rospy.Time(0)
            p_cam.point.x = x_cam
            p_cam.point.y = y_cam
            p_cam.point.z = z_cam

            try:
                p_map = self.tfbuf.transform(p_cam, self.target_frame, rospy.Duration(1.0))
            except Exception as e:
                rospy.logwarn('TF transform failed: %s', e)
                continue

            obj = DetectedObject()
            obj.header = msg.header
            obj.class_name = label
            obj.confidence = conf
            obj.position = p_map.point
            obj.robot_pose = self._robot_pose_in_target_frame(msg.header.stamp)
            obj.xmin = xmin
            obj.ymin = ymin
            obj.xmax = xmax
            obj.ymax = ymax
            detections.objects.append(obj)

            self._publish_object_markers(idx, p_map.point, label, conf, msg.header)
            live_ids.add(idx)
            live_ids.add(idx + 10000)

            idx += 1

        #self._clear_stale_markers(msg.header, live_ids)
        self.last_marker_ids = live_ids

        if detections.objects:
            self.detected_pub.publish(detections)

    def _robot_pose_in_target_frame(self, stamp):
        pose = Pose()
        pose.orientation.w = 1.0
        try:
            tr = self.tfbuf.lookup_transform(self.target_frame, 'base_link', stamp, rospy.Duration(0.1))
            pose.position.x = tr.transform.translation.x
            pose.position.y = tr.transform.translation.y
            pose.position.z = tr.transform.translation.z
            pose.orientation = tr.transform.rotation
            return pose
        except Exception:
            pass

        if self.robot_pose is not None:
            return self.robot_pose
        return pose

    def _publish_object_markers(self, idx, point, label, conf, header):
        m = Marker()
        m.header.frame_id = self.target_frame
        m.header.stamp = header.stamp
        m.ns = 'yolo_objs'
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)
        m.lifetime = rospy.Duration(15.0)
        self.marker_pub.publish(m)

        t = Marker()
        t.header = m.header
        t.ns = 'yolo_labels'
        t.id = idx + 10000
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position = point
        t.pose.position.z += 0.25
        t.pose.orientation.w = 1.0
        t.scale.z = 0.15
        t.color = ColorRGBA(1.0, 1.0, 1.0, 0.95)
        t.text = '{} {:.2f}'.format(label, conf)
        t.lifetime = rospy.Duration(15.0)
        self.marker_pub.publish(t)

    def _clear_stale_markers(self, header, live_ids):
        stale = self.last_marker_ids - live_ids
        for marker_id in stale:
            d = Marker()
            d.header.frame_id = self.target_frame
            d.header.stamp = header.stamp
            d.ns = 'yolo_labels' if marker_id >= 10000 else 'yolo_objs'
            d.id = marker_id
            d.action = Marker.DELETE
            self.marker_pub.publish(d)

    def _search_valid_depth_uint16(self, depth, u, v, radius=5):
        h, w = depth.shape
        for r in range(1, radius+1):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    x = u + dx; y = v + dy
                    if 0 <= x < w and 0 <= y < h:
                        val = depth[y, x]
                        if val != 0:
                            return float(val)
        return None

    def _search_valid_depth_float(self, depth, u, v, radius=5):
        h, w = depth.shape
        for r in range(1, radius+1):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    x = u + dx; y = v + dy
                    if 0 <= x < w and 0 <= y < h:
                        val = depth[y, x]
                        if not np.isnan(val) and val > 0.0:
                            return float(val)
        return None


if __name__ == '__main__':
    try:
        YoloToRviz()
    except rospy.ROSInterruptException:
        pass

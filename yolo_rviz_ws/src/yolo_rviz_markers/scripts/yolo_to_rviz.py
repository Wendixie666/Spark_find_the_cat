#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import ColorRGBA
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import numpy as np
import math

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

class YoloToRviz(object):
    def __init__(self):
        rospy.init_node('yolo_to_rviz')
        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self.have_caminfo = False
        self.depth_cv = None
        self.confirmed_bottles_count = 0   # 已确认瓶子的序号（从1开始）
        
        # --- 核心改进参数 ---
        self.bottle_candidates = []  # 存储候选瓶子：[{'pos': [x,y,z], 'count': 0, 'id': 0, 'active': False}]
        self.marker_id_counter = 0
        self.dist_threshold = 1.3    # 聚类阈值：1.3米内视为同一个瓶子
        self.confirm_threshold = 1   # 确认阈值：连续看到2次才显示Marker
        self.max_detect_dist = 2   # 距离限制：超过2米的检测被视为穿透墙壁的错误深度
        # --------------------

        # 参数获取
        rgb_topic = rospy.get_param('~rgb_topic', '/camera/rgb/image_raw')
        depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        caminfo_topic = rospy.get_param('~camera_info_topic', '/camera/rgb/camera_info')
        self.target_frame = rospy.get_param('~target_frame', 'map')
        self.marker_topic = rospy.get_param('~marker_topic', '/visualization_marker')
        self.target_class = rospy.get_param('~target_class', 'bottle').lower()
        model_path = rospy.get_param('~yolo_model', 'yolov8n.pt')

        if YOLO is None:
            rospy.logerr('ultralytics not available.')
            return
        self.model = YOLO(model_path)

        # 订阅与发布
        rospy.Subscriber(rgb_topic, Image, self.rgb_cb, queue_size=1)
        rospy.Subscriber(depth_topic, Image, self.depth_cb, queue_size=1)
        rospy.Subscriber(caminfo_topic, CameraInfo, self.caminfo_cb, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=50)
        
        self.tfbuf = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfbuf)

        rospy.loginfo('Yolo-Persistence-Node: Robust Mode Started.')
        rospy.spin()

    def caminfo_cb(self, msg):
        if not self.have_caminfo:
            self.cam_model.fromCameraInfo(msg)
            self.cam_frame = msg.header.frame_id
            self.have_caminfo = True

    def _log_bottle(self, pos):
        self.confirmed_bottles_count += 1
        x, y, z = pos[0], pos[1], pos[2]
        rospy.loginfo("find_bottle(%d) at (%.2f, %.2f, %.2f)", 
                    self.confirmed_bottles_count, x, y, z)        

    def depth_cb(self, msg):
        try:
            self.depth_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logwarn('Depth convert failed: %s', e)

    def rgb_cb(self, msg):
        if not self.have_caminfo or self.depth_cv is None:
            return

        try:
            rgb_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # 提高置信度要求(conf=0.7)
            results = self.model.predict(rgb_cv, conf=0.5, verbose=False)[0]
        except:
            return

        if results.boxes is None:
            return

        for box in results.boxes:
            cls_id = int(box.cls.item())
            label = self.model.names.get(cls_id, str(cls_id)).lower()
            if label != self.target_class:
                continue

            # 获取深度
            xyxy = box.xyxy[0].cpu().numpy().astype(int)
            u, v = int((xyxy[0]+xyxy[2])/2), int((xyxy[1]+xyxy[3])/2)
            z = self._get_depth_val(u, v)
            
            # 深度无效或离相机太远（可能是打到了墙上）
            if z is None or z > self.max_detect_dist or z < 0.2:
                continue

            # 坐标转换到地图
            ray = self.cam_model.projectPixelTo3dRay((u, v))
            s = z / ray[2]
            p_cam = PointStamped()
            p_cam.header.frame_id = self.cam_frame
            p_cam.header.stamp = rospy.Time(0)
            p_cam.point.x, p_cam.point.y, p_cam.point.z = ray[0]*s, ray[1]*s, ray[2]*s

            try:
                p_map = self.tfbuf.transform(p_cam, self.target_frame, rospy.Duration(0.5))
            except:
                continue

            # --- 核心处理：平滑与消抖 ---
            self._process_detection(p_map.point)

    def _process_detection(self, point):
        new_pos = np.array([point.x, point.y, point.z])
        found_near = False

        for b in self.bottle_candidates:
            dist = np.linalg.norm(new_pos - b['pos'])
            if dist < self.dist_threshold:
                # 认为是同一个瓶子，更新位置（滤波平滑）
                b['pos'] = (b['pos'] * 0.8 + new_pos * 0.2)
                b['count'] += 1
                found_near = True
                
                # 只有看到次数达到阈值，才显示
                if b['count'] >= self.confirm_threshold and not b['active']:
                    b['active'] = True
                    self._pub_marker(b['pos'], b['id'])
                    rospy.loginfo(f"Bottle confirmed at {b['pos']}")
                # 如果已经激活了，可以更新位置（可选，让Marker跟随微调）
                elif b['active']:
                    self._pub_marker(b['pos'], b['id'])
                break

        if not found_near:
            # 发现新候选点，暂时不发布Marker
            new_candidate = {
                'pos': new_pos,
                'count': 1,
                'id': self.marker_id_counter,
                'active': False
            }
            self.bottle_candidates.append(new_candidate)
            self.marker_id_counter += 1

    def _get_depth_val(self, u, v):
        h, w = self.depth_cv.shape[:2]
        u, v = max(0, min(w-1, u)), max(0, min(h-1, v))
        z = self.depth_cv[v, u]
        if self.depth_cv.dtype == np.uint16:
            if z == 0: return self._search_depth(u, v)
            return float(z) * 0.001
        else:
            if np.isnan(z) or z <= 0: return self._search_depth(u, v)
            return float(z)

    def _search_depth(self, u, v):
        """周围搜索有效深度"""
        h, w = self.depth_cv.shape[:2]
        for r in range(1, 6):
            for dx in [-r, r]:
                for dy in [-r, r]:
                    nu, nv = u+dx, v+dy
                    if 0<=nu<w and 0<=nv<h:
                        val = self.depth_cv[nv, nu]
                        if val > 0 and not np.isnan(val):
                            return float(val) * 0.001 if self.depth_cv.dtype==np.uint16 else float(val)
        return None

    def _pub_marker(self, pos, mid):
        rospy.loginfo(f"find_bottle at ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
        m = Marker()
        m.header.frame_id = self.target_frame
        m.header.stamp = rospy.Time.now()
        m.ns = "stable_bottles"
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color = ColorRGBA(0.0, 1.0, 0.0, 1.0) # 绿色
        m.lifetime = rospy.Duration(0) # 永久
        self.marker_pub.publish(m)

        t = Marker()
        t.header = m.header
        t.ns = "stable_labels"
        t.id = mid
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x, t.pose.position.y, t.pose.position.z = pos
        t.pose.position.z += 0.2
        t.scale.z = 0.12
        t.color = ColorRGBA(1, 1, 1, 1)
        t.text = f"Bottle_{mid}"
        t.lifetime = rospy.Duration(0)
        self.marker_pub.publish(t)

if __name__ == '__main__':
    try:
        YoloToRviz()
    except rospy.ROSInterruptException:
        pass



#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from ultralytics import YOLO

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from yolo_ros_msgs.msg import BoundingBox, BoundingBoxes

from visualization_msgs.msg import Marker

class Yolo_Dect:
    def __init__(self):

        # ==================== 参数 ====================
        weight_path = rospy.get_param('~weight_path', '')
        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        depth_topic = rospy.get_param('~depth_topic', "/camera/depth/image_raw")

        camera_info_topic = rospy.get_param('~camera_info_topic', "/camera/rgb/camera_info")


        pub_topic = rospy.get_param('~pub_topic', '/yolo/BoundingBoxes')

        self.camera_frame = rospy.get_param('~camera_frame', '')
        self.visualize = rospy.get_param('~visualize', True)
        conf = float(rospy.get_param('~conf', 0.5))

        # ==================== 模型 ====================
        self.model = YOLO(weight_path)
        self.model.fuse()
        self.model.conf = conf

        # ==================== 图像数据 ====================
        self.color_image = None
        self.depth_image = None
        self.depth_dtype = None   # ⭐记录深度数据类型

        # ==================== 相机内参 ====================
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # ==================== 订阅 ====================
        self.color_sub = rospy.Subscriber(
            image_topic, Image, self.image_callback, queue_size=1, buff_size=52428800
        )

        # ⭐建议使用对齐后的深度图（非常重要）
        self.depth_sub = rospy.Subscriber(
            depth_topic,
            Image,
            self.depth_callback,
            queue_size=1
        )

        self.camera_info_sub = rospy.Subscriber(
            camera_info_topic,
            CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )

        # ==================== 发布 ====================
        self.position_pub = rospy.Publisher(pub_topic, BoundingBoxes, queue_size=1)
        self.image_pub = rospy.Publisher('/yolo/detection_image', Image, queue_size=1)


        self.marker_pub = rospy.Publisher(
            "/yolo/markers",
            Marker,
            queue_size=10
        )

    # ==================== 深度回调（修复重点） ====================
    def depth_callback(self, msg):

        try:
            if msg.encoding == "32FC1":
                # float32（单位：米）
                self.depth_image = np.frombuffer(
                    msg.data, dtype=np.float32
                ).reshape(msg.height, msg.width)
                self.depth_dtype = np.float32

            elif msg.encoding == "16UC1":
                # uint16（单位：毫米）
                self.depth_image = np.frombuffer(
                    msg.data, dtype=np.uint16
                ).reshape(msg.height, msg.width)
                self.depth_dtype = np.uint16

            else:
                rospy.logerr(f"Unsupported depth encoding: {msg.encoding}")
                return

        except Exception as e:
            rospy.logerr(f"Depth callback error: {e}")

    # ==================== 相机内参 ====================
    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    # ==================== 图像回调 ====================
    def image_callback(self, image):

        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header

        # ROS → numpy
        self.color_image = np.frombuffer(
            image.data, dtype=np.uint8
        ).reshape(image.height, image.width, -1)

        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        # YOLO推理
        results = self.model(self.color_image, show=False)

        self.dectshow(results, image.height, image.width)

        cv2.waitKey(1)

    # ==================== 检测处理 ====================
    def dectshow(self, results, height, width):

        self.frame = results[0].plot()

        # FPS
        fps = 1000.0 / results[0].speed['inference']
        cv2.putText(self.frame, f'FPS: {int(fps)}',
                    (20, 50), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 2)

        for i, result in enumerate(results[0].boxes):

            boundingBox = BoundingBox()

            xmin = int(result.xyxy[0][0].item())
            ymin = int(result.xyxy[0][1].item())
            xmax = int(result.xyxy[0][2].item())
            ymax = int(result.xyxy[0][3].item())

            boundingBox.xmin = xmin
            boundingBox.ymin = ymin
            boundingBox.xmax = xmax
            boundingBox.ymax = ymax

            boundingBox.Class = results[0].names[result.cls.item()]
            boundingBox.probability = result.conf.item()

            # ==================== ⭐ 3D坐标 ====================
            u = int((xmin + xmax) / 2)
            v = int((ymin + ymax) / 2)

            if self.depth_image is not None and self.fx is not None:

                depth = self.depth_image[v, u]

                # 跳过无效深度
                if depth == 0 or np.isnan(depth):
                    continue

                # ⭐ 根据类型判断单位
                if self.depth_dtype == np.uint16:
                    Z = depth / 1000.0   # mm → m
                else:
                    Z = depth            # 已经是 m

                X = (u - self.cx) * Z / self.fx
                Y = (v - self.cy) * Z / self.fy

                self.publish_marker(X, Y, Z, i)

                print(f"[{boundingBox.Class}] 3D: X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}")

                # 显示
                cv2.putText(
                    self.frame,
                    f"{boundingBox.Class} ({X:.2f},{Y:.2f},{Z:.2f})",
                    (u, v),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2
                )

            self.boundingBoxes.bounding_boxes.append(boundingBox)

        # 发布
        self.position_pub.publish(self.boundingBoxes)
        self.publish_image(self.frame, height, width)

        if self.visualize:
            cv2.imshow("yolo_3d", self.frame)

    # ==================== 发布图像 ====================
    def publish_image(self, imgdata, height, width):

        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame

        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3

        self.image_pub.publish(image_temp)


    def publish_marker(self, X, Y, Z, obj_id):

        marker = Marker()

        marker.header.frame_id = self.camera_frame   # ⭐ 相机坐标系
        marker.header.stamp = rospy.Time.now()

        marker.ns = "yolo_objects"
        marker.id = obj_id
        marker.type = Marker.CYLINDER   # ⭐ 圆柱体
        marker.action = Marker.ADD

        # ==================== 位置 ====================
        marker.pose.position.x = X
        marker.pose.position.y = Y
        marker.pose.position.z = Z

        # 无旋转
        marker.pose.orientation.x = 0.7071
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.7071

        # ==================== 尺寸 ====================
        marker.scale.x = 0.4   # 直径
        marker.scale.y = 0.4
        marker.scale.z = 0.5   # 高度

        # ==================== 颜色 ====================
        marker.color.a = 0.8   # 透明度
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        # 生命周期
        marker.lifetime = rospy.Duration(0.5)

        self.marker_pub.publish(marker)


# ==================== 主函数 ====================
def main():
    rospy.init_node('yolo_ros', anonymous=True)
    Yolo_Dect()
    rospy.spin()


if __name__ == "__main__":
    main()
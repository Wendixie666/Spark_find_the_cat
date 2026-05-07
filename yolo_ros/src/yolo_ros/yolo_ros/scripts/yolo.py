#!/usr/bin/env python3
# 指定解释器路径，确保用 python3 运行

# -*- coding: utf-8 -*-
# 指定文件编码为 UTF-8（支持中文）

import cv2                  # OpenCV，用于图像处理与显示
import torch                # PyTorch（YOLO依赖）
import rospy                # ROS Python接口
import numpy as np          # 数值计算
from ultralytics import YOLO  # YOLOv8模型接口
from time import time       # 时间函数

from std_msgs.msg import Header          # ROS标准消息头
from sensor_msgs.msg import Image        # ROS图像消息类型
from yolo_ros_msgs.msg import BoundingBox, BoundingBoxes  # 自定义目标框消息


class Yolo_Dect:
    def __init__(self):

        # ==================== 读取 ROS 参数 ====================

        weight_path = rospy.get_param('~weight_path', '')
        # YOLO模型权重路径（launch文件中配置）

        image_topic = rospy.get_param(
            '~image_topic', '/camera/color/image_raw')
        # 订阅的图像话题

        pub_topic = rospy.get_param('~pub_topic', '/yolo/BoundingBoxes')
        # 发布检测结果的话题

        self.camera_frame = rospy.get_param('~camera_frame', '')
        # 相机坐标系名称（用于TF）

        conf = rospy.get_param('~conf', '0.5')
        # 置信度阈值

        self.visualize = rospy.get_param('~visualize', 'True')
        # 是否显示检测窗口

        # ==================== 设备选择 ====================

        if (rospy.get_param('/use_cpu', 'false')):
            self.device = 'cpu'
        else:
            self.device = 'cuda'
        # 根据参数选择 CPU 或 GPU

        # ==================== 加载 YOLO 模型 ====================

        self.model = YOLO(weight_path)
        # 加载模型

        self.model.fuse()
        # 融合卷积层和BN层，加速推理

        self.model.conf = conf
        # 设置置信度阈值

        # Image：ROS图像消息类型
        self.color_image = Image()
        # 创建图像消息，用于存储当前图像

        # self.getImageStatus = False
        # 标志是否收到图像


        # ==================== 订阅图像 ====================

        self.color_sub = rospy.Subscriber(
            image_topic,              # 输入图像话题
            Image,                    # 消息类型
            self.image_callback,      # 回调函数
            queue_size=1,
            buff_size=52428800        # 大缓冲区（避免图像丢帧）
        )

        # ==================== 发布检测结果 ====================

        self.position_pub = rospy.Publisher(
            pub_topic, BoundingBoxes, queue_size=1)
        # 发布目标框信息

        self.image_pub = rospy.Publisher(
            '/yolo/detection_image', Image, queue_size=1)
        # 发布带检测结果的图像




    # ==================== 图像回调函数 ====================
    def image_callback(self, image):

        # 初始化BoundingBoxes消息
        self.boundingBoxes = BoundingBoxes()
        # 头信息同步到 BoundingBoxes中
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header

        # self.getImageStatus = True

        # ROS Image → numpy数组
        self.color_image = np.frombuffer(
            image.data, dtype=np.uint8
        ).reshape(image.height, image.width, -1)

        # BGR → RGB（YOLO需要RGB）
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        # ==================== YOLO 推理 ====================
        results = self.model(self.color_image, show=False, conf=0.3)

        # 结果处理与显示
        self.dectshow(results, image.height, image.width)

        cv2.waitKey(3)


    # ==================== 结果处理函数 ====================
    def dectshow(self, results, height, width):
##改动：把这些注释掉了----------------------------------------------------
        # # 在图像上画出检测框
        # self.frame = results[0].plot()

        # # 打印推理时间（毫秒）
        # print(str(results[0].speed['inference']))

        # # 计算 FPS
        # fps = 1000.0 / results[0].speed['inference']
##-----------------------------------------------------------------------------------
        self.frame_rgb = results[0].plot()

        # 2. 【修改点】将 RGB 转换为 BGR，以便后续使用 OpenCV 显示和 ROS 发布
        # 如果不转换，看到的图像红色和蓝色会互换
        self.frame = cv2.cvtColor(self.frame_rgb, cv2.COLOR_RGB2BGR)

        # 打印推理时间（毫秒）
        print(str(results[0].speed['inference']))

        # 计算 FPS
        fps = 1000.0 / results[0].speed['inference']



####——-加入了这个——————————————————————

####---------------------------------------------
        # 在图像上写 FPS
        cv2.putText(
            self.frame,
            f'FPS: {int(fps)}',
            (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA
        )

        # ==================== 遍历检测框 ====================
        for result in results[0].boxes:

            boundingBox = BoundingBox()

            # 获取框坐标
            boundingBox.xmin = np.int64(result.xyxy[0][0].item())
            boundingBox.ymin = np.int64(result.xyxy[0][1].item())
            boundingBox.xmax = np.int64(result.xyxy[0][2].item())
            boundingBox.ymax = np.int64(result.xyxy[0][3].item())

            # 类别名称
            boundingBox.Class = results[0].names[result.cls.item()]

            # 置信度
            boundingBox.probability = result.conf.item()

            # 添加到boundingBoxes
            self.boundingBoxes.bounding_boxes.append(boundingBox)

        # 发布检测框
        self.position_pub.publish(self.boundingBoxes)

        # 发布带框图像
        self.publish_image(self.frame, height, width)

        # 显示窗口（如果开启）
        if self.visualize:
            cv2.imshow('yolo', self.frame)


    # ==================== 发布图像 ====================
    def publish_image(self, imgdata, height, width):

        # 创建 空的 ROS 图像消息
        image_temp = Image()

        # 创建 ROS 消息头
        '''
        Header 一般包含：
        时间戳
        坐标系 ID
        序列号（可选）
        '''
        
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame

        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'

        # numpy → bytes
        image_temp.data = np.array(imgdata).tobytes()

        image_temp.header = header

        image_temp.step = width * 3
        # 每行字节数（3通道）

        self.image_pub.publish(image_temp)


# ==================== 主函数 ====================
def main():
    rospy.init_node('yolo_ros', anonymous=True)
    # 初始化ROS节点

    yolo_dect = Yolo_Dect()
    # 创建检测对象

    rospy.spin()
    # 卡住主程序，不让节点死掉，从而让后台能一直监听和处理消息
    # 持续运行（等待回调）


if __name__ == "__main__":
    main()

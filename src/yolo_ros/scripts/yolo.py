#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Optional ROS image-to-bounding-box bridge for Ultralytics YOLO."""

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from ultralytics import YOLO

from yolo_ros_msgs.msg import BoundingBox, BoundingBoxes


def _as_bool(value):
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


class YoloDetector:
    def __init__(self):
        weight_path = rospy.get_param("~weight_path", "yolo26s.pt")
        image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        pub_topic = rospy.get_param("~pub_topic", "/yolo/BoundingBoxes")
        self.camera_frame = rospy.get_param("~camera_frame", "")
        self.confidence = float(rospy.get_param("~conf", 0.5))
        self.visualize = _as_bool(rospy.get_param("~visualize", False))
        use_cpu = _as_bool(rospy.get_param("/use_cpu", True))
        self.device = "cpu" if use_cpu else 0

        self.model = YOLO(weight_path)
        self.image_pub = rospy.Publisher(
            "/yolo/detection_image", Image, queue_size=1
        )
        self.box_pub = rospy.Publisher(pub_topic, BoundingBoxes, queue_size=1)
        rospy.Subscriber(
            image_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=52428800,
        )

    def _image_callback(self, image_msg):
        try:
            bgr_image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(
                image_msg.height,
                image_msg.width,
                -1,
            )
            rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            result = self.model.predict(
                rgb_image,
                conf=self.confidence,
                device=self.device,
                verbose=False,
            )[0]
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "YOLO image processing failed: %s", exc)
            return

        boxes = BoundingBoxes()
        boxes.header = image_msg.header
        boxes.image_header = image_msg.header
        annotated = result.plot()

        for detection in result.boxes:
            box = BoundingBox()
            xmin, ymin, xmax, ymax = (
                detection.xyxy[0].cpu().numpy().astype(int).tolist()
            )
            box.xmin = xmin
            box.ymin = ymin
            box.xmax = xmax
            box.ymax = ymax
            box.Class = str(result.names[int(detection.cls.item())])
            box.probability = float(detection.conf.item())
            boxes.bounding_boxes.append(box)

        self.box_pub.publish(boxes)
        self._publish_image(annotated, image_msg.header)
        if self.visualize:
            cv2.imshow("yolo", annotated)
            cv2.waitKey(1)

    def _publish_image(self, image, source_header):
        message = Image()
        message.header = Header(stamp=rospy.Time.now())
        message.header.frame_id = self.camera_frame or source_header.frame_id
        message.height, message.width = image.shape[:2]
        message.encoding = "bgr8"
        message.step = message.width * 3
        message.data = np.asarray(image, dtype=np.uint8).tobytes()
        self.image_pub.publish(message)


if __name__ == "__main__":
    rospy.init_node("yolo_ros")
    try:
        YoloDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

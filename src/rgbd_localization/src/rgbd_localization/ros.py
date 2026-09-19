"""ROS adapter for the RGB-D localization module."""

import copy

import rospy
import tf2_geometry_msgs  # noqa: F401 - registers geometry conversions
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel

from .core import localize_pixel


class RgbdLocalizer:
    """Adapt ROS camera, depth, and TF messages to the localization core."""

    def __init__(self, tf_buffer, transform_timeout):
        self._bridge = CvBridge()
        self._camera_model = PinholeCameraModel()
        self._camera_frame = ""
        self._camera_info_ready = False
        self._tf_buffer = tf_buffer
        self._transform_timeout = transform_timeout

    @property
    def camera_frame(self):
        return self._camera_frame

    @property
    def ready(self):
        return self._camera_info_ready

    def update_camera_info(self, message):
        self._camera_model.fromCameraInfo(message)
        self._camera_frame = message.header.frame_id
        self._camera_info_ready = True

    def depth_image(self, message):
        return self._bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")

    def localize(
        self,
        depth,
        pixel,
        target_frame,
        stamp,
        source_frame=None,
        depth_min_m=None,
        depth_max_m=None,
    ):
        if not self._camera_info_ready:
            return None

        source_frame = source_frame or self._camera_frame
        point = localize_pixel(
            depth,
            pixel,
            self._camera_model.projectPixelTo3dRay,
            depth_min_m,
            depth_max_m,
        )
        if point is None:
            return None

        point_camera = PointStamped()
        point_camera.header.frame_id = source_frame
        point_camera.header.stamp = stamp
        point_camera.point.x = point[0]
        point_camera.point.y = point[1]
        point_camera.point.z = point[2]

        try:
            return self._transform(point_camera, target_frame)
        except (tf2_ros.TransformException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exact_error:
            if stamp == rospy.Time(0):
                rospy.logdebug("RGB-D TF transform failed: %s", exact_error)
                return None

            rospy.logdebug(
                "RGB-D TF at image time failed; trying latest transform: %s",
                exact_error,
            )
            try:
                latest_point = copy.deepcopy(point_camera)
                latest_point.header.stamp = rospy.Time(0)
                return self._transform(latest_point, target_frame)
            except (tf2_ros.TransformException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as latest_error:
                rospy.logdebug("RGB-D latest TF transform failed: %s", latest_error)
                return None

    def _transform(self, point, target_frame):
        return self._tf_buffer.transform(
            point,
            target_frame,
            self._transform_timeout,
        )

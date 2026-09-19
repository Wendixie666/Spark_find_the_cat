import numpy as np
import rospy
import tf2_ros

from rgbd_localization.ros import RgbdLocalizer


class FakeCameraModel:
    def projectPixelTo3dRay(self, _pixel):
        return 0.0, 0.0, 1.0


class FakeTfBuffer:
    def __init__(self, failures=0):
        self.failures = failures
        self.calls = []

    def transform(self, point, target_frame, timeout):
        self.calls.append((point, target_frame, timeout))
        if self.failures:
            self.failures -= 1
            raise tf2_ros.ExtrapolationException("test transform failure")
        return point


def make_localizer(tf_buffer, timeout):
    localizer = RgbdLocalizer.__new__(RgbdLocalizer)
    localizer._camera_model = FakeCameraModel()
    localizer._camera_frame = "camera_optical_frame"
    localizer._camera_info_ready = True
    localizer._tf_buffer = tf_buffer
    localizer._transform_timeout = timeout
    return localizer


def test_localize_uses_image_stamp_in_point_and_timeout_in_transform():
    timeout = rospy.Duration(0.3)
    tf_buffer = FakeTfBuffer()
    localizer = make_localizer(tf_buffer, timeout)
    stamp = rospy.Time.from_sec(12.3)

    result = localizer.localize(
        np.array([[2.0]], dtype=np.float32),
        (0, 0),
        "map",
        stamp,
    )

    assert result.header.stamp == stamp
    assert len(tf_buffer.calls) == 1
    point, target_frame, call_timeout = tf_buffer.calls[0]
    assert point.header.stamp == stamp
    assert target_frame == "map"
    assert call_timeout == timeout


def test_localize_retries_with_latest_transform_timestamp():
    timeout = rospy.Duration(0.3)
    tf_buffer = FakeTfBuffer(failures=1)
    localizer = make_localizer(tf_buffer, timeout)
    stamp = rospy.Time.from_sec(12.3)

    result = localizer.localize(
        np.array([[2.0]], dtype=np.float32),
        (0, 0),
        "map",
        stamp,
    )

    assert result is not None
    assert len(tf_buffer.calls) == 2
    first_point = tf_buffer.calls[0][0]
    latest_point = tf_buffer.calls[1][0]
    assert first_point is not latest_point
    assert first_point.header.stamp == stamp
    assert latest_point.header.stamp == rospy.Time(0)
    assert tf_buffer.calls[1][2] == timeout

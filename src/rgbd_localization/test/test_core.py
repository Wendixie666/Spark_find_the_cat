import numpy as np

from rgbd_localization.core import (
    camera_point_from_pixel,
    depth_value_meters,
    localize_pixel,
)


def test_uint16_depth_is_converted_from_millimeters():
    depth = np.zeros((2, 2), dtype=np.uint16)
    depth[1, 0] = 1250

    assert depth_value_meters(depth, (0, 1)) == 1.25


def test_invalid_center_depth_uses_nearby_valid_depth():
    depth = np.full((5, 5), np.nan, dtype=np.float32)
    depth[2, 3] = 2.0

    assert depth_value_meters(depth, (2, 2)) == 2.0


def test_invalid_depth_returns_none():
    depth = np.zeros((3, 3), dtype=np.uint16)

    assert localize_pixel(depth, (1, 1), lambda _pixel: (0.0, 0.0, 1.0)) is None


def test_depth_range_rejects_out_of_range_observations():
    depth = np.array([[2.0]], dtype=np.float32)

    assert localize_pixel(
        depth,
        (0, 0),
        lambda _pixel: (0.0, 0.0, 1.0),
        depth_min_m=2.1,
    ) is None


def test_pixel_and_depth_become_a_metric_camera_point():
    depth = np.array([[2.0]], dtype=np.float32)

    point = localize_pixel(
        depth,
        (0, 0),
        lambda _pixel: (0.5, -0.25, 1.0),
    )

    np.testing.assert_allclose(point, [1.0, -0.5, 2.0])


def test_zero_ray_depth_returns_none():
    point = camera_point_from_pixel(
        (0, 0),
        2.0,
        lambda _pixel: (1.0, 1.0, 0.0),
    )

    assert point is None

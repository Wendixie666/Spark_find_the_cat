"""ROS-independent RGB-D localization math."""

import numpy as np


def depth_value_meters(depth, pixel, search_radius=5):
    """Return the first valid depth around ``pixel`` in meters."""

    height, width = depth.shape[:2]
    if height == 0 or width == 0:
        return None

    u, v = pixel
    u = max(0, min(width - 1, int(u)))
    v = max(0, min(height - 1, int(v)))

    def as_meters(value):
        if depth.dtype == np.uint16:
            return float(value) * 0.001 if value > 0 else None
        value = float(value)
        return value if np.isfinite(value) and value > 0 else None

    value = as_meters(depth[v, u])
    if value is not None:
        return value

    for radius in range(1, search_radius + 1):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                x, y = u + dx, v + dy
                if 0 <= x < width and 0 <= y < height:
                    value = as_meters(depth[y, x])
                    if value is not None:
                        return value
    return None


def camera_point_from_pixel(pixel, depth_m, project_pixel_to_3d_ray):
    """Return a metric camera-frame point or ``None`` for an invalid ray."""

    ray = np.asarray(project_pixel_to_3d_ray(pixel), dtype=float)
    if ray.shape[0] < 3 or not np.all(np.isfinite(ray[:3])) or ray[2] == 0:
        return None

    scale = depth_m / ray[2]
    return np.array([ray[0] * scale, ray[1] * scale, ray[2] * scale])


def localize_pixel(
    depth,
    pixel,
    project_pixel_to_3d_ray,
    depth_min_m=None,
    depth_max_m=None,
):
    """Convert an image pixel and aligned depth into a metric camera point."""

    depth_m = depth_value_meters(depth, pixel)
    if depth_m is None:
        return None
    if depth_min_m is not None and depth_m < depth_min_m:
        return None
    if depth_max_m is not None and depth_m > depth_max_m:
        return None
    return camera_point_from_pixel(pixel, depth_m, project_pixel_to_3d_ray)

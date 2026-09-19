from .core import camera_point_from_pixel, depth_value_meters, localize_pixel

__all__ = [
    "RgbdLocalizer",
    "camera_point_from_pixel",
    "depth_value_meters",
    "localize_pixel",
]


def __getattr__(name):
    if name == "RgbdLocalizer":
        from .ros import RgbdLocalizer

        return RgbdLocalizer
    raise AttributeError(name)

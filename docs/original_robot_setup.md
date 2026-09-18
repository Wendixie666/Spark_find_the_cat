# Original Spark deployment

The original course project was deployed on a Spark mobile robot with an
RGB-D camera. The old repository also contained a complete Spark/Noetic
workspace, `darknet_ros`, robot bring-up scripts, hardware rules, and other
packages that are not part of the cat-finding or semantic-mapping logic.

Those snapshots have been removed from the main source tree. The maintained
nodes now expect the standard ROS interfaces supplied by an external robot or
simulation:

- RGB image, depth image, and camera calibration topics;
- a TF tree containing the camera, `base_link`, and `map` frames;
- `/map` from SLAM;
- `explore_lite` (or another exploration node);
- `move_base` for target navigation.

This keeps the repository useful without pretending that a physical Spark is
available. Spark-specific bring-up can be added later as a separate package or
workspace without copying the complete vendor system into this project.

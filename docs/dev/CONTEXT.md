# Spark Find the Cat

This context names the project's RGB-D perception and map-based target-search concepts.

## Language

**RGB-D localization**:
Converts a detected image pixel and aligned depth measurement into a metric 3D point in a requested coordinate frame.
_Avoid_: 3D detection, object mapping (when referring only to coordinate conversion)

**Cat search**:
The mission flow that explores a mapped area, confirms a cat location, and navigates to an approach point.
_Avoid_: generic detection pipeline

**Stable object map**:
The persistent set of spatially confirmed objects represented in the map and visualized in RViz.
_Avoid_: detection list

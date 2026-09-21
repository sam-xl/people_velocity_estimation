# coresense_people_estimation

## Overview

Turns 2-D person detections into `people_msgs/People`: samples depth under each detection,
back-projects it through the camera model, and fits a velocity over a short per-track history.

Input is [`riskam_msgs/PersonDetections`](https://github.com/sam-xl/risk-awareness-module)
from [RiskAM](https://github.com/sam-xl/risk-awareness-module)'s
`riskam_node`: bounding boxes, ByteTrack ids and COCO-17 keypoints only. Everything metric
happens here, so the depth policy can change without touching RiskAM or the interface between
them.

Output is in the **colour camera's optical frame** and **robot-relative**; re-framing into a
world-fixed frame and removing the robot's own motion is not done here yet, so `/people` is not
yet what nav2's social costmap layer consumes. The pipeline is described in
[`docs/`](./docs/index.md).

**Authors: [George Sotirchos](https://github.com/gsotirchos-samxl) ([SAM XL](https://github.com/sam-xl))**

**Maintainers: g.sotirchos@tudelft.nl**

Developed and tested under ROS 2 Jazzy on Ubuntu 24.04.

## Installation

### Dependencies

All resolvable with `rosdep`, except two source dependencies listed in `.repos`:
[`riskam_msgs`](https://github.com/sam-xl/risk-awareness-module) and
[`people_msgs`](https://github.com/wg-perception/people).

### Building

To build from source, clone the latest version from this repository into your workspace:

```bash
cd ros_ws/src
git clone https://github.com/sam-xl/people_velocity_estimation.git
```

Use [`vcstool`](https://github.com/dirk-thomas/vcstool) to clone the source dependencies and
complete the workspace:

```bash
cd ros_ws/src
vcs import < coresense_people_estimation/.repos
```

Install the dependencies of the cloned packages using `rosdep`:

```bash
cd ros_ws/src
rosdep install --from-paths . --ignore-src -y
```

Finally, build all packages in the workspace:

```bash
colcon build
```

## Usage

```bash
ros2 launch coresense_people_estimation people_estimation.launch.py
```

Override the parameter file with `config:=/path/to/your.yml`. Add `debug:=true` to draw the
geometry as a **MarkerArray** on `/people_estimation/debug_markers`; view it in RViz with the
fixed frame set to `camera_color_optical_frame`. See
[Debug visualisation](./docs/debug_visualisation.md) for what is drawn and how to read it.

No TF tree is needed: nothing here is transformed. Detections, a registered depth image and the
colour `camera_info` are the whole input, and nothing is published until `camera_info` arrives.

### Testing from a bag

A replay carrying the detections and the aligned depth image is the whole rig — there is no TF
tree to fake:

```bash
ros2 launch coresense_people_estimation people_estimation.launch.py debug:=true
ros2 bag play <bag>
ros2 topic echo /people
```

If the replay carries the colour images but not their `camera_info`, nothing is ever published.
`bag_camera_shim.py` publishes a stand-in colour `camera_info`, paced by the depth stream:

```bash
ros2 run coresense_people_estimation bag_camera_shim.py
```

Pass `--color-intrinsics fx fy cx cy` when the real calibration is known — the defaults are
nominal D435i intrinsics at 640x480, which are close enough to see the pipeline work and not
close enough to trust a number from. Takes `--help`.

> The scripts in `scripts/` need their executable bit (`chmod +x`) — `--symlink-install`
> links straight to the source file, and `ros2 run` will not find them otherwise.

## Launch files

* **`people_estimation.launch.py`**: brings up the node as `/people_estimation`, with
  `config/people_estimation.yml`. Arguments: `config` and `debug`. `debug:=` always overrides
  the parameter file's `debug` key.

## Nodes

### `people_estimation`

Also available as the composable component `coresense_people_estimation::PeopleEstimationNode`.

Reads a depth for each detection, back-projects it and fits a velocity over an adaptive
per-track window; each stage is described in [`docs/`](./docs/index.md).

#### Parameters

See [`config/people_estimation.yml`](coresense_people_estimation/config/people_estimation.yml),
which documents every parameter inline.

#### Published Topics

* `/people` ([people_msgs/People](https://github.com/wg-perception/people))\
  Positions and velocities in the **colour optical frame**, relative to the robot.
  Per-person depth diagnostics ride along in `tagnames`/`tags`, joined by the window and
  velocity-covariance numbers under `publish_covariance_tags`.
* `/people_estimation/debug_markers` ([visualization_msgs/MarkerArray](https://docs.ros2.org/latest/api/visualization_msgs/msg/MarkerArray.html))\
  Debug geometry, only under `debug`. See [Debug visualisation](./docs/debug_visualisation.md).
* `/tf` ([tf2_msgs/TFMessage](https://docs.ros2.org/latest/api/tf2_msgs/msg/TFMessage.html))\
  One `person_<id>` frame per person, only under `debug` and `debug_person_frames`.

#### Subscribed Topics

* `/riskam/person_detections` ([riskam_msgs/PersonDetections](https://github.com/sam-xl/risk-awareness-module))\
  Bounding boxes, ByteTrack ids and COCO-17 keypoints, stamped in the colour optical frame.
  A detection ByteTrack did not associate arrives as `track_id: -1`; those share one id, so they
  are published as measured and stationary rather than given a history.
* `/camera/camera/aligned_depth_to_color/image_raw` ([sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html))\
  `16UC1` millimetres or `32FC1` metres. Paired with the detections by stamp.
  **Must be registered to the colour frame** — needs `align_depth.enable: true` on the driver.
  `/camera/camera/depth/image_rect_raw` is the wrong image; see [Depth read-out](./docs/depth_readout.md).
* `/camera/camera/color/camera_info` ([sensor_msgs/CameraInfo](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html))\
  Intrinsics for the back-projection. Nothing is published until this arrives.
* `/cmd_vel` ([geometry_msgs/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html))\
  Dead-reckons the camera yaw that de-rotates a window before its consistency is judged. It does
  not otherwise enter the published velocity.

## Detailed Documentation

Other than the brief introduction in this page, you can also check the detailed documentation
[here](./docs/index.md): the depth read-out, the adaptive-window velocity fit and the debug
visualisation.

## Bugs & Feature Requests

Please report bugs and request features using the
[Issue Tracker](https://github.com/sam-xl/people_velocity_estimation/issues).

# people_velocity_estimation

[![Pipeline status](https://github.com/sam-xl/people_velocity_estimation/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/sam-xl/people_velocity_estimation/actions/workflows/ci.yml?query=branch%3Amain) \
[![Latest Release](https://img.shields.io/github/v/release/sam-xl/people_velocity_estimation?sort=semver)](https://github.com/sam-xl/people_velocity_estimation/releases/latest) \
[![Maturity](https://img.shields.io/badge/maturity-M1-blue.svg)](#) \
[![License](https://img.shields.io/github/license/sam-xl/people_velocity_estimation)](LICENSE)

## Overview

Turns 2-D person detections into `people_msgs/People`: samples depth under each detection,
back-projects it through the camera model, fits a velocity over a short per-track history, and
re-frames the result into a world-fixed frame, compensated for the robot's own motion.

Input is [`riskam_msgs/PersonDetections`](https://github.com/sam-xl/risk-awareness-module)
from [RiskAM](https://github.com/sam-xl/risk-awareness-module)'s
`riskam_node`: bounding boxes, ByteTrack ids and COCO-17 keypoints only. Everything metric
happens here, so the depth policy can change without touching RiskAM or the interface between
them.

Everything metric is measured in the **colour camera's optical frame** and re-framed into
`target_frame` (`map` by default) on the way out, so `/people` is world-fixed and ready for
nav2's social costmap layer. The pipeline is described in [`docs/`](./docs/index.md).

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
vcs import < people_velocity_estimation/.repos
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
ros2 launch people_velocity_estimation people_estimation.launch
```

Override the parameter file with `config:=/path/to/your.yml`. Add `debug:=true` to draw the
geometry as a **MarkerArray** on `/people_estimation/debug_markers`; view it in RViz with the
fixed frame set to `target_frame` (`map`). See
[Debug visualisation](./docs/debug_visualisation.md) for what is drawn and how to read it.

TF must supply `source_frame -> target_frame` and `robot_base_frame -> target_frame`, so this
needs a localisation stack — or `scripts/dummy_tf.py` on the bench, see
[Testing without a robot](#testing-without-a-robot). Without it nothing is published, only a
throttled "Could not transform" warning. Nothing is published until `camera_info` arrives either.

### Testing without a robot

`scripts/dummy_tf.py` fills in whichever part of the TF chain is missing, and
`scripts/bag_camera_shim.py` publishes a stand-in colour `camera_info` for replays that lack one:

```bash
ros2 run people_velocity_estimation dummy_tf.py               # map -> odom -> base_link -> camera
ros2 run people_velocity_estimation bag_camera_shim.py
```

Which `dummy_tf.py` flags suit a real robot or a replay, and how to check the velocity
compensation, are in [Bench tooling](./docs/bench_tooling.md).

> The scripts in `scripts/` need their executable bit (`chmod +x`) — `--symlink-install`
> links straight to the source file, and `ros2 run` will not find them otherwise.

## Launch files

* **`people_estimation.launch`**: brings up the node as `/people_estimation`, with
  `config/people_estimation.yml`. Arguments: `config` and `debug`. `debug:=` always overrides
  the parameter file's `debug` key.

## Nodes

### `people_estimation`

Also available as the composable component `people_velocity_estimation::PeopleEstimationNode`.

Confirms each detection has persisted, reads its depth, rejects posters and depth spikes,
back-projects it, fits a velocity over an adaptive per-track window, re-frames it into
`target_frame` with the robot's motion compensated, coasts people the camera has lost and
attaches a `reliability`; each stage is described in [`docs/`](./docs/index.md).

#### Parameters

See [`config/people_estimation.yml`](people_velocity_estimation/config/people_estimation.yml),
which documents every parameter inline.

#### Published Topics

* `/people` ([people_msgs/People](https://github.com/wg-perception/people))\
  Positions and velocities in `target_frame` (`map` by default), compensated for the robot's
  own motion — or in the colour optical frame, uncompensated, when `target_frame` is empty.
  Per-person depth, hold and background diagnostics ride along in `tagnames`/`tags`, joined by
  the window and covariance numbers under `publish_covariance_tags` and given in the colour
  optical frame. People the camera has lost are carried forward here too, tagged `coast_s`;
  see [Coasting](./docs/coasting.md).
* `/people_estimation/debug_markers` ([visualization_msgs/MarkerArray](https://docs.ros2.org/latest/api/visualization_msgs/msg/MarkerArray.html))\
  Debug geometry, only under `debug`. See [Debug visualisation](./docs/debug_visualisation.md).
* `/tf` ([tf2_msgs/TFMessage](https://docs.ros2.org/latest/api/tf2_msgs/msg/TFMessage.html))\
  One `person_<id>` frame per person, only under `debug` and `debug_person_frames`.

#### Subscribed Topics

* `/riskam/person_detections` ([riskam_msgs/PersonDetections](https://github.com/sam-xl/risk-awareness-module))\
  Bounding boxes, ByteTrack ids and COCO-17 keypoints, stamped in the colour optical frame.
  A detection is ignored until its id has been seen `confirm_frames` frames running, and one
  carrying no id at all (`track_id: -1`) is dropped; see [Confirmation](./docs/confirmation.md).
* `/camera/camera/aligned_depth_to_color/image_raw` ([sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html))\
  `16UC1` millimetres or `32FC1` metres. Paired with the detections by stamp.
  **Must be registered to the colour frame** — needs `align_depth.enable: true` on the driver.
  `/camera/camera/depth/image_rect_raw` is the wrong image; see [Plausibility](./docs/plausibility.md).
* `/camera/camera/color/camera_info` ([sensor_msgs/CameraInfo](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html))\
  Intrinsics for the back-projection. Nothing is published until this arrives.
* `/cmd_vel` ([geometry_msgs/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html))\
  Dead-reckons the camera yaw that de-rotates a window before its consistency is judged, and
  supplies `v_base` and `ω` for the motion compensation, where it *does* enter the published
  velocity. It does not scale the spike-rejection bound, which is a flat `max_person_speed`.

## Detailed Documentation

Other than the brief introduction in this page, you can also check the detailed documentation
[here](./docs/index.md): confirmation, depth read-out, plausibility and spike filtering, the
adaptive-window velocity fit, the map transform and motion compensation, coasting, the
uncertainty model, the debug visualisation and the bench tooling.

## Bugs & Feature Requests

Please report bugs and request features using the
[Issue Tracker](https://github.com/sam-xl/people_velocity_estimation/issues).

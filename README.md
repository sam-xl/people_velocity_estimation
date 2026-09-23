# people_velocity_estimation

[![Pipeline status](https://github.com/sam-xl/people_velocity_estimation/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/sam-xl/people_velocity_estimation/actions/workflows/ci.yml?query=branch%3Amain) \
[![Latest Release](https://img.shields.io/github/v/release/sam-xl/people_velocity_estimation?sort=semver)](https://github.com/sam-xl/people_velocity_estimation/releases/latest) \
[![Maturity](https://img.shields.io/badge/maturity-M1-blue.svg)](#) \
[![License](https://img.shields.io/github/license/sam-xl/people_velocity_estimation)](LICENSE)

## Overview

Turns 2-D person detections into `people_msgs/People`: samples depth under each detection,
back-projects it through the camera model, tracks position and velocity with a per-track
Kalman filter, and re-frames the result into a world-fixed frame, compensated for the robot's own
motion.

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

`scripts/dummy_tf.py` fills in whichever part of the TF chain is missing, so the node can be
exercised with no localisation stack and no robot:

```bash
ros2 run people_velocity_estimation dummy_tf.py               # map -> odom -> base_link -> camera
ros2 launch people_velocity_estimation people_estimation.launch debug:=true
```

#### Which invocation

The choice is driven by how much of the tree something else already owns. Publishing a
transform that is already owned gives its child frame two parents, which tf2 resolves by
silently keeping whichever arrived last — so err towards publishing less.

| Situation | Command |
|---|---|
| No robot and no replay — pure bench | `dummy_tf.py` |
| Replay carrying only the camera | `dummy_tf.py` |
| Real robot, or a replay with a full tree **including localisation** | `dummy_tf.py --camera-only` |
| Real robot or replay with odometry but **no localisation** | `dummy_tf.py --camera-only --static-odom` |

`--camera-only` publishes the camera mount alone — one transform, unless `--static-odom` below adds
the second — for when the robot's own TF comes from a bag replay or a live robot. Publishing the rest on top of one is actively harmful — `base_link`,
`chassis_link` and `mid_mount` all end up with two parents, and wall-clock stamps against an
older recording fall outside tf2's cache window. The script's docstring spells this out.

`--static-base` pins the base at the origin instead. That makes hand-computed checks easy, but
it zeroes **both** velocity-compensation terms, so it is not a test of the velocity path.

`--static-odom` adds that identity `map -> odom` back under `--camera-only`, restoring the target
frame without touching the odometry or the structure below it. Reach for it rather than
`--static-base`, which pins `odom -> base_link` and would make this a second publisher of an
edge the robot already broadcasts — the pose then flips between the two every few
milliseconds.

`map -> odom` is the one transform here that is never integrated from `/cmd_vel`, in either
mode. It is the localisation *correction*, and the robot's own odometry already carries the
motion in `odom -> base_link` — driving it from `/cmd_vel` too would move the base twice as
far and turn it twice as fast in `map`, and double `v_base` in the velocity compensation with
it.

Check the tree with `ros2 run tf2_ros tf2_echo map camera_color_optical_frame` — the rotation
matrix should read `[[0,0,1],[-1,0,0],[0,-1,0]]`, optical z (forward) onto base x (forward).

#### Exercising the velocity compensation

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.3}}'
```

A person with zero velocity *relative to the camera* is not standing still: with the base
driving they are carried along with it, so a correct cyan arrow in `map` is non-zero and
roughly parallel to the magenta robot arrow. Zero map velocity there means the compensation
is not being applied.

The sharpest check is a person genuinely motionless in the world: project a fixed map-frame
point into the optical frame using live TF and difference it between frames. Everything the
node receives then changes every frame while the correct answer is constant. At 0.5 m/s and
0.3 rad/s with the person 4.5 m out that gave 0.0000 m max position error and 0.0050 m/s max
residual speed — differencing noise, not bias. A residual that grows with *range* means
`ω × r` is wrong; one proportional to *speed* points at `v_base` or `robot_base_frame`.

A replay that carries the colour images but not their `camera_info` publishes nothing.
`bag_camera_shim.py` publishes a stand-in colour `camera_info`, paced by the depth stream:

```bash
ros2 run people_velocity_estimation bag_camera_shim.py
```

Pass `--color-intrinsics fx fy cx cy` when the real calibration is known — the defaults are
nominal D435i intrinsics at 640x480, which are close enough to see the pipeline work and not
close enough to trust a number from. Both scripts take `--help`.

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
back-projects it, tracks it with a per-track Kalman filter, re-frames it into
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
  the covariance and innovation numbers under `publish_covariance_tags` and given in the colour
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
  Dead-reckons the camera yaw the Kalman filter turns its state by on each prediction, and
  supplies `v_base` and `ω` for the motion compensation.

## Detailed Documentation

Other than the brief introduction in this page, you can also check the detailed documentation
[here](./docs/index.md): confirmation, depth read-out, plausibility, the Kalman filter and its
spike gate, the map transform and motion compensation, coasting, the uncertainty model and the
debug visualisation.

## Bugs & Feature Requests

Please report bugs and request features using the
[Issue Tracker](https://github.com/sam-xl/people_velocity_estimation/issues).

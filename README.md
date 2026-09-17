# coresense_people_estimation

## Overview

Turns 2-D person detections into `people_msgs/People`: samples depth under each detection and
back-projects it through the camera model.

Input is [`riskam_msgs/PersonDetections`](https://github.com/sam-xl/risk-awareness-module)
from [RiskAM](https://github.com/sam-xl/risk-awareness-module)'s
`riskam_node`: bounding boxes, ByteTrack ids and COCO-17 keypoints only. Everything metric
happens here, so the depth policy can change without touching RiskAM or the interface between
them.

**Authors: [George Sotirchos](https://github.com/gsotirchos-samxl) ([SAM XL](https://github.com/sam-xl))**

**Maintainers: g.sotirchos@tudelft.nl**

Developed and tested under ROS 2 Jazzy on Ubuntu 24.04.

## Where this sits

```
        riskam_node                        people_estimation
 /riskam/person_detections   ──>                /people
(bbox, track id, keypoints)            (depth, 3D position)
```

Everything is measured and published in the **colour camera's optical frame**, and everything is
**robot-relative**: a person standing still while the robot drives at them gets closer every
frame without having moved. Re-framing into a world-fixed frame and taking the robot's own motion
back out is a separate job, and not one this node does yet — so `/people` here is not what nav2's
social costmap layer wants to consume.

## Installation

### Dependencies

All resolvable with `rosdep`, except two source dependencies listed in `.repos`:
[`riskam_msgs`](https://github.com/sam-xl/risk-awareness-module) and
[`people_msgs`](https://github.com/wg-perception/people).

### Building

```bash
cd ros_ws/src
git clone https://github.com/sam-xl/people_velocity_estimation.git
vcs import < coresense_people_estimation/.repos
rosdep install --from-paths . --ignore-src -y
cd .. && colcon build
```

## Usage

```bash
ros2 launch coresense_people_estimation people_estimation.launch.py
```

Override the parameter file with `config:=/path/to/your.yml`, and add `debug:=true` for the
[marker visualisation](#debug-visualisation).

No TF tree is needed: nothing here is transformed. Detections, a registered depth image and the
colour `camera_info` are the whole input, and nothing is published until `camera_info` arrives.

## Launch files

* **`people_estimation.launch.py`**: brings up the node as `/people_estimation`, with
  `config/people_estimation.yml`. Arguments: `config` and `debug`. `debug:=` always overrides
  the parameter file's `debug` key.

## Scripts

Bench tooling, run with `ros2 run` rather than `ros2 launch`. Takes `--help`.

* **`bag_camera_shim.py`**: publishes a stand-in colour `camera_info`, paced by the depth
  stream, for a replay that carries the colour images but not their `camera_info`. Defaults to
  nominal D435i intrinsics at 640x480; pass `--color-intrinsics fx fy cx cy` for a real
  calibration.

## Nodes

### `people_estimation`

Also available as the composable component `coresense_people_estimation::PeopleEstimationNode`.

The stages below run in the order they are listed, which is also roughly their order of
importance: each one depends on the ones above it holding up.

| # | Stage | In one line |
|---|---|---|
| 1 | [Depth read-out](#depth-read-out) | Turn a box and keypoints into a range and an anchor pixel |
| 2 | [Debug visualisation](#debug-visualisation) | Markers for checking the geometry by eye |

#### Depth read-out

`DepthReader::read` takes the median of small patches around the shoulder and hip keypoints, which
keeps the samples on the body so background inside the bounding box never enters the estimate.
A keypoint the pose model did not find reads as exactly `(0, 0)` and is treated as absent. With
no usable torso keypoint it falls back to a low percentile over the whole box — the closest
valid surface, as in RiskAM's proximity sub-score, minus its near-clip fallback: a constant is
not a measurement and must not be back-projected.

The depth is attributed to the **hip midpoint** when both hips are visible and the box centre
otherwise, since the hips track the centre of mass more stably than a box that shifts whenever a
limb extends. That pixel is what gets back-projected. A detection whose depth cannot be measured
is **dropped**.

Do not go back to a single pixel at the box centre with a zero/NaN check, as `riskam_node` once
did: a sample landing on background is a perfectly valid number, and it placed people metres too
far away.

**Depth must be registered to the colour frame** — `/camera/camera/depth/image_rect_raw` has a
wider field of view and about two thirds the focal length, so a colour pixel indexed into it
lands somewhere else. Set `align_depth.enable: true` on the driver and point `depth_topic` at
`/camera/camera/aligned_depth_to_color/image_raw`.

#### Debug visualisation

The geometry is invisible in the output topic — a wrong depth read still produces
plausible-looking numbers. `debug:=true` draws it instead, on
`/people_estimation/debug_markers`. Add a **MarkerArray** display on that topic in RViz, with the
fixed frame set to the **colour optical frame** (`camera_color_optical_frame`), since that is the
frame everything is published in. Each group is its own namespace, toggleable under
*Displays → People markers → Namespaces*:

| Namespace | Appearance | Meaning |
|---|---|---|
| `people` | green spheres | Back-projected positions |
| `label` | white text | `name` per person |

With `debug_person_frames` on, each person also gets a TF frame at their position. Markers
expire after `debug_marker_lifetime`, so a stalled pipeline shows as everything vanishing rather
than a frozen scene; people who leave are explicitly deleted. Off is genuinely off — neither the
publisher nor the broadcaster is created.

**Reading it.** Remember the frame is optical: x right, y down, z forward. A person 3 m in front
of the camera sits at `z ≈ 3`, not `x ≈ 3`.

- **A person sits metres too far away** → the depth read landed on background. Check that
  `depth_topic` is the *aligned* image and watch `depth_valid_frac` in the tags.
- **A sphere sits off to one side of the person** → the anchor pixel fell outside the body. Watch
  `depth_spread_m` in the tags; a large spread means the patch straddled an edge.

#### Parameters

See [`config/people_estimation.yml`](config/people_estimation.yml), which documents every
parameter inline.

#### Subscribed Topics

* `/riskam/person_detections` ([riskam_msgs/PersonDetections](https://github.com/sam-xl/risk-awareness-module))\
  Bounding boxes, ByteTrack ids and COCO-17 keypoints, stamped in the colour optical frame.
  A detection ByteTrack did not associate arrives as `track_id: -1`; those all share that one id,
  so each is given a name of its own instead.
* `/camera/camera/aligned_depth_to_color/image_raw` ([sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html))\
  `16UC1` millimetres or `32FC1` metres. Paired with the detections by stamp.
  **Must be registered to the colour frame** — needs `align_depth.enable: true` on the driver.
  `/camera/camera/depth/image_rect_raw` is the wrong image; see the depth read-out section.
* `/camera/camera/color/camera_info` ([sensor_msgs/CameraInfo](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html))\
  Intrinsics for the back-projection. Nothing is published until this arrives.

#### Published Topics

* `/people` ([people_msgs/People](https://github.com/wg-perception/people))\
  Positions in the **colour optical frame**, relative to the robot. Per-person depth diagnostics
  ride along in `tagnames`/`tags`.
* `/people_estimation/debug_markers` ([visualization_msgs/MarkerArray](https://docs.ros2.org/latest/api/visualization_msgs/msg/MarkerArray.html))\
  Debug geometry, only under `debug`. See [Debug visualisation](#debug-visualisation).
* `/tf` ([tf2_msgs/TFMessage](https://docs.ros2.org/latest/api/tf2_msgs/msg/TFMessage.html))\
  One `person_<id>` frame per person, only under `debug` and `debug_person_frames`.

## Testing from a bag

A replay carrying the detections and the aligned depth image is the whole rig — there is no TF
tree to fake:

```bash
ros2 launch coresense_people_estimation people_estimation.launch.py debug:=true
ros2 bag play <bag>
ros2 topic echo /people
```

If the replay carries the colour images but not their `camera_info`, nothing is ever published.
`bag_camera_shim.py` fills that in, paced by the depth stream:

```bash
ros2 run coresense_people_estimation bag_camera_shim.py
```

Pass `--color-intrinsics fx fy cx cy` when the real calibration is known — the defaults are
nominal D435i intrinsics at 640x480, which are close enough to see the pipeline work and not
close enough to trust a number from.

> The scripts in `scripts/` need their executable bit (`chmod +x`) — `--symlink-install`
> links straight to the source file, and `ros2 run` will not find them otherwise.

## Detailed Documentation

See [`docs/`](./docs/).

## Bugs & Feature Requests

Please report bugs and request features using the
[Issue Tracker](https://github.com/sam-xl/people_velocity_estimation/issues).

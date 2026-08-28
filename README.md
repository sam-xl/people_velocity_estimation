# coresense_people_estimation

## Overview

Turns 2-D person detections into `people_msgs/People`: samples depth under each detection,
back-projects it through the camera model, and fits a velocity over a short per-track history.

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
(bbox, track id, keypoints)       (depth, 3D position, velocity)
```

Everything is measured and published in the **colour camera's optical frame**, and everything is
**robot-relative**: a person standing still while the robot drives at them reads as approaching
at the robot's speed. Re-framing into a world-fixed frame and taking the robot's own motion back
out is a separate job, and not one this node does yet — so `/people` here is not what nav2's
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
| 2 | [Velocity adaptive window](#velocity-adaptive-window) | Fit a velocity over a per-track window |
| 3 | [Debug visualisation](#debug-visualisation) | Markers for checking the geometry by eye |

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

#### Velocity adaptive window

The **window** is the trailing `n` samples of a track's position history. `fitVelocity` runs a
least-squares fit of `p(t) = v·t + b` over them, one axis at a
time [[1]](https://iate.oac.uncor.edu/~mario/materia/nr/numrec/f15-2.pdf), and `v` is the published
velocity. Detections arrive at roughly 10 Hz, so a fixed `n` either lags behind somebody stopping
or chases noise; `n` is therefore chosen per track, per frame, by asking whether a straight line
is still a fair description of those
samples [[2]](https://cim.mcgill.ca/~haptic/pub/FS-VH-CSC-TCST-00.pdf):

```
r_i  = p_i − (v·t_i + b)              residual: how far sample i sits off the fitted line
σ_i² = (s · σ_pos,i)² + σ_gait²       how far it was expected to sit off
χ²_ν = Σ_i (r_i² / σ_i²) / (n − 2)    reduced chi-square: the ratio of the two
```

`σ_i²` is **the denominator** — the variance each residual is judged against, per sample and per
axis. `σ_pos,i` is that sample's position sigma from `positionVariance()`, `s` is
`velocity_window_sigma_scale`, `σ_gait` is `velocity_gait_sigma_m`, and the `n − 2` is because the
fit spent two degrees of freedom on `v` and `b`. So `χ²_ν ≈ 1` means the samples scatter about the
line exactly as much as the noise model says they
should [[3]](https://arxiv.org/pdf/2306.07225), and larger means they do not lie on one line at
all. `selectWindow` starts with `n` as long as `velocity_window_max_s` allows and drops
one sample from the front while `χ²_ν` exceeds `velocity_window_chi2_max`, stopping at
`velocity_window_min_s`. Straight-line walking keeps `χ²_ν` low, so the window stays long and
smooths hard; stopping, starting or turning drives it up and collapses the window onto the recent
samples. It is evaluated **per axis** with the largest of the three taken — pooling the three
divides a one-axis manoeuvre by three, enough on its own to hide somebody stopping while walking
across the view.

`positionVariance()` is the Jacobian propagation of
`P = ray · z` [[4]](https://arxiv.org/pdf/1705.06516), kept to its diagonal: `σu`/`σv` scale with
the bounding box, and `σz` is the larger of the measured spread of the depth readings and the
sensor noise floor `coeff · z²`. Nothing publishes it — it exists to be the denominator above,
which is read one axis at a time, so the cross terms are not carried.

Three details make the denominator work:

* **The denominator is frame-to-frame noise, not absolute position error.**
  `positionVariance()` answers "how far off could this person really be", and most of that is
  `pixel_sigma_frac · bbox` — uncertainty about *where on the body* the anchor pixel sits. That
  offset is much the same every frame, so it shifts the whole track without adding scatter around
  a straight-line fit. Divide residuals by it and the fit always looks better than expected: the
  window never shortens and the estimator quietly becomes a fixed `velocity_window_max_s` one.
  `s` is the fraction of `σ_pos` treated as genuinely independent between frames; at `s = 1` even
  a lateral reversal only reaches `χ²_ν ≈ 2`, under the 3.0 threshold.
* **Gait is in the noise model, not the signal.** A walking body oscillates about its own track
  with every step [[5]](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7029926/), which a test
  carrying only sensor noise reads as changing motion and pins the window at its floor for
  exactly the people who need it long. `σ_gait` is that wobble, added in quadrature above so a
  step's worth of sway costs no chi-square, and `velocity_window_min_s` floors the window at
  about one gait oscillation.
* **Chosen de-rotated, fitted body-fixed.** The arc a stationary person traces while the robot
  turns would collapse every window during every turn, so *samples are de-rotated by the
  cmd_vel-integrated camera yaw* [[6]](https://arxiv.org/pdf/2308.09080). Each `σ_pos,i` rotates
  with its own sample — σ_x runs two to three times σ_z, so leaving the sigmas unrotated
  over-counts `χ²_ν` by 2.2× at 0.5 rad of yaw. Stale `/cmd_vel` degrades to windows shortening
  during turns.

Keeping the fit itself body-fixed is what lets a later stage take the robot's own motion out with
a transport-theorem correction.

A speed below `velocity_epsilon` publishes as exactly zero, since each person's debug TF frame is
yawed to `atan2(vx, vz)` about optical −y. Below `velocity_min_span_s` of history there is no
velocity at all, so a new track reports zero with `velocity_prior_variance`. A track unseen for
longer than `velocity_max_gap_s` has its history cleared rather than a slope fitted across the hole.

Velocity covariance comes from the fit itself, `Var(v) = σ²·n/det` with
`σ² = ‖Aβ − p‖²/(n−2)` [[1]](https://iate.oac.uncor.edu/~mario/materia/nr/numrec/f15-2.pdf);
fewer than three samples fit exactly and report `velocity_prior_variance` instead. It is
conditional on a window selected *because* it fits, so it is optimistically biased.
`people_msgs/Person` has no covariance field, so it goes out as a `tagnames`/`tags` string pair,
along with the sample count and window length, under `publish_covariance_tags` (off by default).
The `depth_valid_frac` and `depth_spread_m` diagnostics are published either way.

#### Debug visualisation

The geometry is invisible in the output topic — a wrong depth read and a wrong velocity model
both produce plausible-looking numbers. `debug:=true` draws it instead, on
`/people_estimation/debug_markers`. Add a **MarkerArray** display on that topic in RViz, with the
fixed frame set to the **colour optical frame** (`camera_color_optical_frame`), since that is the
frame everything is published in. Each group is its own namespace, toggleable under
*Displays → People markers → Namespaces*:

| Namespace | Appearance | Meaning |
|---|---|---|
| `people` | green spheres | Back-projected positions |
| `velocity` | cyan arrows | Fitted velocities |
| `label` | white text | `name \| speed` per person |

Arrows are look-ahead vectors: length is speed × `debug_velocity_scale`, so at the default an
arrow tip sits where that person would be one second from now. With `debug_person_frames` on,
each person also gets a TF frame at their position, yawed along their velocity. Markers
expire after `debug_marker_lifetime`, so a stalled pipeline shows as everything vanishing rather
than a frozen scene; people who leave are explicitly deleted. Off is genuinely off — neither the
publisher nor the broadcaster is created.

**Reading it.** Remember the frame is optical: x right, y down, z forward. A person 3 m in front
of the camera sits at `z ≈ 3`, not `x ≈ 3`.

- **A person sits metres too far away** → the depth read landed on background. Check that
  `depth_topic` is the *aligned* image and watch `depth_valid_frac` in the tags.
- **Arrows jitter while somebody walks in a straight line** → the window is collapsing when it
  should not. Set `publish_covariance_tags: true`; `velocity_window_s` in the tags then says how
  long it actually is. Raise `velocity_window_chi2_max` or `velocity_gait_sigma_m`.
- **A stationary person's arrow grows while the robot turns** → the yaw de-rotation is not
  getting `/cmd_vel`.

#### Parameters

See [`config/people_estimation.yml`](config/people_estimation.yml), which documents every
parameter inline.

#### Subscribed Topics

* `/riskam/person_detections` ([riskam_msgs/PersonDetections](https://github.com/sam-xl/risk-awareness-module))\
  Bounding boxes, ByteTrack ids and COCO-17 keypoints, stamped in the colour optical frame.
  A detection ByteTrack did not associate arrives as `track_id: -1`; those share one id, so they
  are published as measured and stationary rather than given a history.
* `/camera/camera/aligned_depth_to_color/image_raw` ([sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html))\
  `16UC1` millimetres or `32FC1` metres. Paired with the detections by stamp.
  **Must be registered to the colour frame** — needs `align_depth.enable: true` on the driver.
  `/camera/camera/depth/image_rect_raw` is the wrong image; see the depth read-out section.
* `/camera/camera/color/camera_info` ([sensor_msgs/CameraInfo](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html))\
  Intrinsics for the back-projection. Nothing is published until this arrives.
* `/cmd_vel` ([geometry_msgs/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html))\
  Dead-reckons the camera yaw that de-rotates a window before its consistency is judged. It does
  not otherwise enter the published velocity.

#### Published Topics

* `/people` ([people_msgs/People](https://github.com/wg-perception/people))\
  Positions and velocities in the **colour optical frame**, relative to the robot. Per-person
  Per-person depth diagnostics ride along in `tagnames`/`tags`, joined by the window and
  velocity-covariance numbers under `publish_covariance_tags`.
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

# Bench tooling

Two scripts in `scripts/` stand in for what a robot would provide, so the node can be exercised
on the bench or against a replay. Both take `--help`.

## `dummy_tf.py`

Fills in whichever part of the TF chain is missing. Publishing a transform that something else
already owns gives its child frame two parents, which tf2 resolves by silently keeping whichever
arrived last — so err towards publishing less.

| Situation | Command |
|---|---|
| No robot and no replay — pure bench | `dummy_tf.py` |
| Replay carrying only the camera | `dummy_tf.py` |
| Real robot, or a replay with a full tree **including localisation** | `dummy_tf.py --camera-only` |
| Real robot or replay with odometry but **no localisation** | `dummy_tf.py --camera-only --static-odom` |

- `--camera-only` publishes the camera mount alone, for when the robot's own TF comes from a bag
  or a live robot. Publishing the rest on top would give `base_link` and its children two
  parents, and wall-clock stamps against an older recording fall outside tf2's cache window.
- `--static-base` pins the base at the origin. That makes hand-computed checks easy, but it zeroes
  **both** velocity-compensation terms, so it is not a test of the velocity path.
- `--static-odom` adds an identity `map -> odom` under `--camera-only`, restoring the target frame
  without a second publisher of the robot's own `odom -> base_link`.
- `map -> odom` is never integrated from `/cmd_vel`, in either mode: it is the localisation
  *correction*, and `odom -> base_link` already carries the motion. Integrating both would
  double `v_base` in the velocity compensation.

Check the tree with `ros2 run tf2_ros tf2_echo map camera_color_optical_frame`: the rotation
matrix should read `[[0,0,1],[-1,0,0],[0,-1,0]]`, optical z (forward) onto base x (forward).

## Exercising the velocity compensation

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.3}}'
```

A person with zero velocity *relative to the camera* is carried along with the driving base, so
in `map` their cyan arrow should be non-zero and roughly parallel to the magenta robot arrow.
Zero map velocity there means the compensation is not being applied.

## `bag_camera_shim.py`

A replay that carries the colour images but not their `camera_info` publishes nothing. This
script publishes a stand-in colour `camera_info`, paced by the depth stream. Pass
`--color-intrinsics fx fy cx cy` when the real calibration is known: the defaults are nominal
D435i intrinsics at 640x480, close enough to see the pipeline work but not to trust a number
from.

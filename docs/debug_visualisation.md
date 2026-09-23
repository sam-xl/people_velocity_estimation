# Debug visualisation

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

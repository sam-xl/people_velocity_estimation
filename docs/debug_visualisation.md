# Debug visualisation

The transform is invisible in the output topic — a wrong TF chain and a wrong velocity model both
produce plausible-looking numbers. `debug:=true` draws it instead, on
`/people_estimation/debug_markers`. Add a **MarkerArray** display on that topic in RViz, fixed to
`map`. Each group is its own namespace, toggleable under *Displays → People markers →
Namespaces*:

| Namespace | Appearance | Frame | Meaning |
|---|---|---|---|
| `people` | green spheres | `target_frame` | Transformed positions |
| `velocity` | cyan arrows | `target_frame` | Transformed velocities |
| `label` | white text | `target_frame` | `name \| speed` per person |
| `raw_people` | orange spheres | *colour optical frame* | The untransformed input |
| `raw_velocity` | orange arrows | *colour optical frame* | The untransformed velocities |
| `robot_velocity` | magenta arrow | `target_frame` | The `cmd_vel` used to compensate |

Arrows are look-ahead vectors: length is speed × `debug_velocity_scale`, so at the default an
arrow tip sits where that person would be one second from now. With `debug_person_frames` on,
each person also gets a TF frame at their map position, yawed along their velocity. Markers
expire after `debug_marker_lifetime`, so a stalled pipeline shows as everything vanishing rather
than a frozen scene; people who leave are explicitly deleted. Off is genuinely off — neither the
publisher nor the broadcaster is created.

**Reading it.** The two point clouds are the diagnostic — `raw_*` is what was measured,
`people`/`velocity` is what the transform made of it.

- **Positions disagree** → the TF chain is wrong. Suspect the source `frame_id`: detections must
  be stamped in an **optical** frame (x right, y down, z forward), since positions come from
  `projectPixelTo3dRay`. `camera_link` (x forward, y left, z up) rotates every pose 90°.
- **Positions agree, velocities do not** → the compensation is wrong. Check the magenta arrow: it
  should point where the robot is actually driving.
- **A stationary person's green arrow grows as the robot turns** → `ω × r` is wrong, and the
  error scales with range.

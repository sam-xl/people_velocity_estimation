# people_velocity_estimation <!-- omit in toc -->

[[_TOC_]]

## Introduction

```
        riskam_node                           people_estimation
 /riskam/person_detections   ──>                   /people                    ──>  nav2 social costmap
(bbox, track id, keypoints)       (depth, 3D, velocity, camera frame -> map)
                                                      │
                                    TF: map <- odom <- base_link <- camera
```

Everything metric is measured in the **colour camera's optical frame** and re-framed into
`target_frame` (`map` by default) on the way out, so `/people` is world-fixed. Set
`target_frame: ""` to skip that and publish in the camera frame — the escape hatch for a rig
with no TF tree, not a normal setting.

The node runs the stages below in the order listed, which is also roughly their order of
importance: each one depends on the ones above it holding up.

| # | Stage | In one line |
|---|---|---|
| 1 | [Confirmation](confirmation.md) | Drop detections that have not persisted |
| 2 | [Depth read-out](depth_readout.md) | Turn a box and keypoints into a range and an anchor pixel |
| 3 | [Plausibility](plausibility.md) | Reject people printed on posters and screens |
| 4 | [Position spike filtering](position_spike_filtering.md) | Reject depth that read the background |
| 5 | [Velocity adaptive window](velocity_adaptive_window.md) | Fit a velocity over a per-track window |
| 6 | [Map transform & motion compensation](map_transform.md) | Re-frame to `target_frame`, take the robot's own motion out |
| 7 | [Coasting](coasting.md) | Carry forward a person the camera has lost |
| 8 | [Uncertainty](uncertainty.md) | Covariances and `reliability` |
| 9 | [Debug visualisation](debug_visualisation.md) | Markers for checking the geometry by eye |

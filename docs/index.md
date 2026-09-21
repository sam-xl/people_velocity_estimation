# coresense_people_estimation <!-- omit in toc -->

[[_TOC_]]

## Introduction

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

The node runs the stages below in the order listed, which is also roughly their order of
importance: each one depends on the ones above it holding up.

| # | Stage | In one line |
|---|---|---|
| 1 | [Depth read-out](depth_readout.md) | Turn a box and keypoints into a range and an anchor pixel |
| 2 | [Velocity adaptive window](velocity_adaptive_window.md) | Fit a velocity over a per-track window |
| 3 | [Debug visualisation](debug_visualisation.md) | Markers for checking the geometry by eye |

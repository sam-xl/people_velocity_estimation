# people_velocity_estimation

## Introduction

`PeopleEstimationNode` owns one collaborator per stage below. Dashed arrows are topics exchanged
with a specific node; the lollipops are interfaces open to any.

```mermaid
classDiagram
  %% Mermaid colours boxes by declaration order: keep the classes in the
  %% sequence diagram's participant order, so both colour each one alike.
  direction LR

  class riskam_node {
    <<external>>
  }
  class RealSense {
    <<external>>
  }
  class PeopleEstimationNode {
    <<component>>
    -detectionsCallback(detections, depth)
    -cameraInfoCallback(camera_info)
    -cmdVelCallback(twist)
  }
  class ConfirmationFilter {
    +confirmed(detection, stamp) bool
    +dropStale(stamp)
  }
  class DepthReader {
    +read(depth, detection) DepthEstimate
  }
  class PlausibilityFilter {
    +screen(depth, detection, range, stamp) Plausibility
    +dropStale(stamp)
  }
  class UncertaintyModel {
    +positionCovariance(ray, depth, bbox, fx, fy) Matrix3d
    +reliability(depth, covariance, n_keypoints) double
  }
  class PersonTracker {
    +update(id, stamp, position, covariance, yaw) TrackUpdate
    +dropStale(stamp)
  }
  class PeopleMapTransform {
    +transform(people, cmd_vel, motion) bool
    +lookupFromTarget(frame, stamp, transform) bool
  }
  class TrackCoaster {
    +observe(people, ids, stamp)
    +coast(stamp, ids, in_view) Person[]
    +forget(id)
    +dropStale(stamp)
  }
  class DebugVisualizer {
    +publish(people, raw, robot)
  }

  class nav2_social_layer {
    <<external>>
  }

  PeopleEstimationNode *-- ConfirmationFilter
  PeopleEstimationNode *-- DepthReader
  PeopleEstimationNode *-- PlausibilityFilter
  PeopleEstimationNode *-- UncertaintyModel
  PeopleEstimationNode *-- PersonTracker
  PeopleEstimationNode *-- PeopleMapTransform
  PeopleEstimationNode *-- TrackCoaster
  PeopleEstimationNode *-- DebugVisualizer


  riskam_node ..> PeopleEstimationNode : /riskam/person_detections
  RealSense ..> PeopleEstimationNode : /camera/camera/aligned_depth_to_color/image_raw
  RealSense ..> PeopleEstimationNode : /camera/camera/color/camera_info
  %% Lollipop ends are class names, which reject "/" unless backtick-quoted.
  `/cmd_vel` ()-- PeopleEstimationNode
  PeopleEstimationNode ..> nav2_social_layer : /people
  `/tf, /tf_static` ()-- PeopleMapTransform
  DebugVisualizer --() `/people_estimation/debug_markers`
  DebugVisualizer --() `/tf`
```

Per synchronised detections/depth pair, the node calls them in this order:

```mermaid
sequenceDiagram
  %% Participant order sets the colours; keep it in step with the class diagram.
  participant R as riskam_node
  participant C as RealSense
  participant N as PeopleEstimationNode
  participant CF as ConfirmationFilter
  participant DR as DepthReader
  participant PF as PlausibilityFilter
  participant UM as UncertaintyModel
  participant PT as PersonTracker
  participant MT as PeopleMapTransform
  participant TC as TrackCoaster
  participant DV as DebugVisualizer
  participant NV as nav2 social layer

  C-)N: /camera/camera/color/camera_info (latched camera model)
  R-)N: /riskam/person_detections
  C-)N: /camera/camera/aligned_depth_to_color/image_raw
  Note over N: ApproximateTime sync, decodeDepth
  loop each detection
    N->>CF: confirmed(detection)
    break unconfirmed
      Note over N: next detection
    end
    N->>DR: read(depth, detection)
    break no depth in band
      Note over N: next detection
    end
    N->>PF: screen(depth, detection, range)
    break implausible
      N->>TC: forget(track_id)
    end
    N->>UM: positionCovariance(ray, depth, bbox)
    N->>PT: update(id, position, covariance, yaw)
    N->>UM: reliability(depth, covariance)
  end
  N->>MT: transform(people, cmd_vel)
  N->>TC: observe(people)
  N->>MT: lookupFromTarget(camera frame)
  N->>TC: coast(in_view)
  N-)NV: /people
  opt debug
    N->>DV: publish(people)
  end
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

For exercising the node with no robot or localisation stack, see
[Bench tooling](bench_tooling.md).

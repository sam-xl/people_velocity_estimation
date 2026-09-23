# Uncertainty

Position covariance is the Jacobian propagation of
`P = ray · z` [[4]](https://arxiv.org/pdf/1705.06516):
`Σ = J diag(σu², σv², σz²) Jᵀ` with `J = [ z/fx·ex , z/fy·ey , ray ]`. `σu`/`σv` scale with the
bounding box; `σz` is the larger of the measured spread of the depth readings and the sensor
noise floor `coeff · z²`. The depth term elongates the ellipsoid along the line of sight but does
not always dominate — a tall box at close range can put more uncertainty across the image than
along the ray — so read the covariance rather than assuming the classic cigar shape. That is the
per-detection covariance, which seeds a new track and sizes the
[Kalman filter](kalman_filter.md)'s measurement noise. What is published is the filter's own
`P`: its position block, and its velocity block, which starts at `velocity_prior_variance`.

`people_msgs/Person` has no covariance field, so both go out as `tagnames`/`tags` string pairs.
**nav2 reads none of it** — only `reliability` reaches the costmap — but they survive for
inspection. The map transform copies them through untouched, so they stay in the **colour optical
frame** while the positions alongside them are in `target_frame`. Set
`publish_covariance_tags: true` to publish them; the depth, hold and background diagnostics go
out either way.

`reliability` scales the amplitude of nav2's social Gaussian, so it reports confidence that a
person is at this position: the product of a valid-depth term, a keypoint-completeness term and a
localisation term from the covariance trace. Velocity confidence is excluded, since an
ill-determined velocity does not make the person less present. While a position is held,
`reliability` combines *this* frame's depth validity and keypoint count with the *predicted*
covariance: the detector did see a body this frame, so only the position is in doubt. `P⁻` grows
with every gated frame, so reliability fades the longer nothing is believed. A converged track
has a smaller `P` than a single detection, so it reads higher than one frame's covariance would.

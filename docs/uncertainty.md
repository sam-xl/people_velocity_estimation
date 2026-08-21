# Uncertainty

Position covariance is the Jacobian propagation of
`P = ray · z` [[4]](https://arxiv.org/pdf/1705.06516):
`Σ = J diag(σu², σv², σz²) Jᵀ` with `J = [ z/fx·ex , z/fy·ey , ray ]`. `σu`/`σv` scale with the
bounding box; `σz` is the larger of the measured spread of the depth readings and the sensor
noise floor `coeff · z²`. The depth term elongates the ellipsoid along the line of sight but does
not always dominate — a tall box at close range can put more uncertainty across the image than
along the ray — so read the covariance rather than assuming the classic cigar shape. Velocity
covariance comes from the fit itself, `Var(v) = σ²·n/det` with
`σ² = ‖Aβ − p‖²/(n−2)` [[1]](https://iate.oac.uncor.edu/~mario/materia/nr/numrec/f15-2.pdf);
fewer than three samples fit exactly and report `velocity_prior_variance` instead. It is
conditional on a window selected *because* it fits, so it is optimistically biased.

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
`reliability` combines *this* frame's depth validity and keypoint count with the *held*
covariance: the detector did see a body this frame, so only the position is in doubt.

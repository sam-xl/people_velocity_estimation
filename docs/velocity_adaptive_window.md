# Velocity adaptive window

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
velocity at all, so a new or re-seeded track reports zero with `velocity_prior_variance`. A track
unseen for longer than `velocity_max_gap_s` has its history cleared rather than a slope fitted
across the hole. While the spike filter holds a position the last fitted velocity is republished
with it — zero would assert the person stopped, which a rejected reading is no evidence of; held
frames never enter the history, and a re-seed clears it rather than differentiating across the
discontinuity.

Velocity covariance comes from the fit itself, `Var(v) = σ²·[(AᵀA)⁻¹]₀₀` with
`σ² = ‖Aβ − p‖²/(n−2)` [[1]](https://iate.oac.uncor.edu/~mario/materia/nr/numrec/f15-2.pdf);
fewer than three samples fit exactly and report `velocity_prior_variance` instead. It is
conditional on a window selected *because* it fits, so it is optimistically biased.
`people_msgs/Person` has no covariance field, so it goes out as a `tagnames`/`tags` string pair,
along with the sample count and window length, under `publish_covariance_tags` (off by default).
The `depth_valid_frac` and `depth_spread_m` diagnostics are published either way.

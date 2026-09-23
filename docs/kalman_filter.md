# Kalman filter

Each track runs one constant-velocity Kalman filter, OpenCV's
[`cv::KalmanFilter`](https://docs.opencv.org/4.x/dd/d6a/classcv_1_1KalmanFilter.html), over
`x = [p; v]` in the colour optical frame. The published position, velocity and covariances are
its state after this frame's update.

```
predict:  x⁻ = T(Δψ) · F(Δt) · x          F = [[I, Δt·I], [0, I]]
          P⁻ = T F P Fᵀ Tᵀ + Q(Δt)         Q = σ_a² · [[Δt⁴/4·I, Δt³/2·I], [Δt³/2·I, Δt²·I]]
gate:     d² = yᵀ S⁻¹ y                    y = z − p⁻,  S = P⁻_pp + R
correct:  only when d² ≤ kalman_gate_chi2
```

`Q` is the discrete white-noise acceleration model, with `σ_a` = `kalman_accel_sigma`: how hard a
person is expected to change velocity between frames. Raise it for a faster response to stops and
turns, lower it for a smoother estimate.

`Q` is statistical, not a bound. On top of it, a corrected velocity that changed by more than
`max_person_accel · Δt` since the prediction is clamped back onto that
bound [[1]](https://doi.org/10.1049/iet-cta.2009.0032), leaving `P` as computed. The clamp is
skipped while the velocity is still less certain than one frame's worth of that acceleration, so it
never slows a new track finding its velocity. It bounds transients; it does not smooth gait jitter,
which is well under 4 m/s² frame to frame. `kalman_accel_sigma` and `R` are what smooth it.

## Tuning

The filter is only as good as `R` and `Q` are honest. With `publish_covariance_tags` on, the mean
`kalman_innovation_chi2` over steady walking should be about 3, the number of measured
axes [[2]](https://doi.org/10.1002/0471221279). Well above 3 means `R` is too small. The filter
then chases every reading, the velocity jitters in magnitude and direction, and the gate starts
rejecting real frames. Raise `gait_sigma_m` or `measurement_sigma_scale`. Well below 3 means `R`
is too large and the filter lags, so lower them. Then trade smoothness against how quickly stops
show up with `kalman_accel_sigma`.

On a synthetic walker at 10 Hz with ±3 cm sway and 5 cm depth noise, the old defaults (`σ_a` 1.0,
`σ_gait` 0.03) gated 6 % of real frames with a heading error of 6°. The current ones (0.5, 0.06)
gate 3 % with 3°, and take about 1 s to report a stop, against 0.8 s. That is a starting point,
not a calibration.

Three details carry over from the least-squares estimator this replaced:

* **The measurement noise is frame-to-frame noise, not absolute position error.**
  `R = s² · Σ_pos + σ_gait² · I`, with `Σ_pos` from `positionCovariance()`, `s` =
  `measurement_sigma_scale` and `σ_gait` = `gait_sigma_m`. Most of `Σ_pos` is uncertainty about
  *where on the body* the anchor pixel sits, which is much the same every frame. Used as `R`
  unscaled, it would make the filter trust its prediction far too much. Gait sway goes into `R`
  too, so a step's worth of wobble does not read as a manoeuvre.
* **The camera's turn is taken out on predict.** `T(Δψ)` rotates both `p` and `v` by the
  `cmd_vel`-integrated camera yaw since the last frame. Without it, the arc a stationary person
  traces while the robot turns would violate the constant-velocity model on every turn. As a
  result, the state velocity excludes the sweep the turn causes, so it is added back on output:
  `v_out = v + ψ̇ · (ŷ × p)`. That is the apparent velocity in the turning camera frame, which is
  what the [map transform](map_transform.md)'s `ω × r` term expects to cancel.
* **A speed below `velocity_epsilon` publishes as exactly zero**, since each person's debug TF frame
  is yawed along it.

## Spike gating

Depth still occasionally reads the background through a bounding box, putting a person metres away
for a frame or two. The squared Mahalanobis distance `d²` of the reading from the prediction is
tested against `kalman_gate_chi2`. The default of 11.34 is the 99 % point of χ² with 3 degrees of
freedom. A reading past it is not folded in, and the prediction is published instead, with
`position_hold_frames` counting the frames gated. `d²` goes out as the `kalman_innovation_chi2`
tag under `publish_covariance_tags`, for tuning the gate from a bag.

Because the gate uses `S`, it is in units of the track's own uncertainty rather than a flat speed.
It widens as `P⁻` grows over a gap, and tightens once the track has converged.

Two escape hatches keep a rejection from becoming permanent, since the jump is sometimes real
(occlusion, or ByteTrack handing an old id to a new person):

* After `spike_max_holds` consecutive gated frames, the sensor is believed and the track re-seeds
  with a throttled warning.
* A track unseen for longer than `spike_reset_timeout` re-seeds, as does one whose clock walked
  backwards (a looping bag).

A re-seeded track starts at the measurement with `Σ_pos` and zero velocity at
`velocity_prior_variance`. `spike_max_holds: 0` disables the gate.

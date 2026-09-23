# Position spike filtering

Depth still occasionally reads background through a bounding box, putting a person metres away
for a frame or two. If `‖Δp‖ / Δt` from the last *accepted* position of that track exceeds
`max_person_speed`, the reading is discarded and the last accepted position republished.

The two positions are brought into a common camera pose before they are subtracted, using the
same `cumulative_yaw` the [velocity window](velocity_adaptive_window.md) is judged on. Without
that the bound is not a bound on the person at all: a robot turning at 1 rad/s sweeps a
*stationary* person at the 3 m edge of the depth band through the optical frame at ω·r = 3 m/s,
clearing `max_person_speed` on its own and freezing a real person for the length of the turn.
What is left after the rotation is taken out is bounded flat, since the errors this catches are
metres per frame.

Two escape hatches keep a rejection from becoming permanent, since the jump is sometimes real
(occlusion, or ByteTrack handing an old id to a new person). A position is held for at most
`spike_max_holds` consecutive frames, after which the sensor is believed and the track re-seeds
with a throttled warning; and an accepted position older than `spike_reset_timeout` is dropped
rather than compared against. `spike_max_holds: 0` disables the filter, accepting every reading as the
tracker did before the filter existed. An unassociated detection
has no previous reading to compare against and goes out as measured — a hole that only reopens
when `confirm_frames` is 0, since confirmation drops those first.

A held position is a stale measurement republished, not an extrapolation: `position_hold_frames`
counts the frames held. The held entry keeps the stamp of the accepted reading, so `Δt` grows and
the same jump looks slower each frame — correct, since the longer nothing has been believed, the
further the person could have gone.

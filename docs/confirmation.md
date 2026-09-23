# Confirmation

The pose model occasionally draws a box in a random spot for exactly one frame. Nothing
downstream can tell that box from a real one, so nav2's social layer ends up planning around an
obstacle that was already gone. A detection is therefore withheld until its ByteTrack id has
been seen `confirm_frames` frames running. This sits **ahead of the depth read**, so an
unconfirmed detection costs nothing but a map lookup, and "seen N frames" means purely that the
detector kept drawing the box — whether that box also had a readable depth is a different
question, answered further down.

**Untracked detections are dropped.** ByteTrack reports every unassociated detection as
`track_id: -1`, so they share one id and no per-detection streak can be kept — and unassociated
is precisely the state a one-frame phantom arrives in. The streak is made consecutive by
`confirm_reset_timeout` alone, with no separate frame-gap test: a gap test strict enough to be
worth having also fails outright the moment the detector stutters under load, and a silent
`/people` is worse than the phantom it prevents. A track's stamp is refreshed on every sighting,
so 0.5 s still confirms down to about 2.5 Hz. `confirm_frames: 0` restores the old behaviour,
untracked detections included, which is the A/B switch for judging whether the gate helps.

Two knock-on effects, both accepted: a new person's velocity history starts `confirm_frames`
frames late, and a poster's `implausible_patience` frames begin only once it has confirmed.
`riskam_msgs/PersonDetection` carries no confidence field, so the streak cannot be weighted by
how sure the detector was.

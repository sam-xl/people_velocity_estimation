# Coasting

Every stage above is driven by a detection arriving, so the moment the detector stops reporting
somebody — the robot turns and clips them out of frame, they walk out past `depth_max_m` — they
leave `/people` on the very next message. nav2's social layer has no memory of its own, so the
costmap bump vanishes with them and a path is free to be planned straight through where they are
still standing. The coaster therefore carries a track forward: for `coast_max_s` after the last
sighting, a person with no detection this frame is published anyway, at
`p(t) = p_last + v_last · (t − t_last)`. The anchor is always the last *measured* position, never
the previous extrapolation, so error grows in proportion to the gap rather than compounding.
Untracked detections are never coasted.

**Only for somebody the camera can no longer see.** Every coasted position is projected back
through the camera model, and one that lands inside the image is *dropped* rather than published:
the camera looked there and the detector reported nobody, which is evidence of absence in a way
that a person outside the frustum never is. Erased rather than withheld, too, so turning away
cannot bring the ghost back. This is a deliberate trade and it is the behaviour, not a setting —
it gives up the in-frame cases, **a person behind a pillar and a detector stuttering under load
are not coasted**, to avoid ever inventing a person the camera is pointed straight at. The test
insets by `coast_in_view_margin_m` at the person's own range, because a detection at the image
border is unreliable enough that absence there proves nothing; widen that margin past half the
image and nothing is ever dropped, which is how you turn the test off. When TF cannot say where
the camera points the test is skipped entirely and coasting runs ungated — not knowing where the
camera is looking is no reason to drop anybody.

Three more things are deliberate. **It runs after the map transform**, which is the whole reason
the extrapolation is one line: `target_frame` is world-fixed, so carrying a person forward needs
to know nothing about what the robot did meanwhile. It is also why a message that cannot be
re-framed leaves the coaster completely untouched — nothing is remembered or forgotten on the
strength of a frame that was never published. **The velocity is held, not decayed** — not seeing
somebody is no evidence that they stopped, and bleeding towards zero would park a walking person
at the edge of frame they were last seen crossing. **A coasted person is labelled as one**, with
`coast_s` and `coast_position_sigma_m` attached regardless of `publish_covariance_tags`, and
`reliability` scaled down linearly to zero at `coast_max_s` so they fade out of the social layer
rather than blinking out of it. `0` disables coasting entirely.

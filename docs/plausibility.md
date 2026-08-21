# Plausibility

A poster, roll-up banner or screen showing a person is a detection the pose model is right to
make and that a costmap must not plan around. **The test is whether anything is behind the
person.** For this, a strip across the top of the box is processed. The contents of that strip
would be mostly not the head, so for a real body most of it reads the scene behind: either no
return at all (out of range, too dark or too oblique), or a return past them by more than
`background_step_m`. A printed person has nothing behind them, being the same flat surface.

`background_min_far_frac` sets the threshold below which a person is "too flat" to be real. At
0.03 no real detection is rejected and all banner detections are, with no dependence on range.
The registration requirement from the [depth read-out](depth_readout.md) is doubly load-bearing
here: **against an unregistered depth image the test fails silently**, since the strip is read
somewhere other than above the person. Counting a dropout as background is the other half of the
test: a pixel with no return is not part of this surface.

Rejection is hysteretic — a track must fail `implausible_patience` consecutive frames before it
is suppressed, so a person walking past a pillar costs a few false-positive frames rather than
blinking out. The streak keeps its own `plausibility_reset_timeout`, longer than
`spike_reset_timeout`, because a suppressed track stops feeding `tracks_` and a shared clock
would re-admit the poster on a cycle; the measured fraction rides along on every published person
as `background_frac`, so the threshold can be re-tuned from a bag. Three known limits: a strip
too small to read is neither a rejection nor a reset, so it passes the frame through and leaves
the streak for the frames around it to carry — clearing it there would let a banner that
intermittently clips the frame edge never reach the patience at all; an untracked detection has
no identity to accumulate a streak against and is judged on one frame; and a real person standing
flat against a wall is suppressed, with `background_step_m` buying about a body's depth of margin.

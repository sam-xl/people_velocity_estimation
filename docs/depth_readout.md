# Depth read-out

`DepthReader::read` takes the median of small patches around the shoulder and hip keypoints, which
keeps the samples on the body so background inside the bounding box never enters the estimate.
A keypoint the pose model did not find reads as exactly `(0, 0)` and is treated as absent. With
no usable torso keypoint it falls back to a low percentile over the whole box — the closest
valid surface, as in RiskAM's proximity sub-score, minus its near-clip fallback: a constant is
not a measurement and must not be back-projected.

The depth is attributed to the **hip midpoint** when both hips are visible and the box centre
otherwise, since the hips track the centre of mass more stably than a box that shifts whenever a
limb extends. That pixel is what gets back-projected. A detection whose depth cannot be measured
is **dropped**, ahead of the spike gate, so a person whose position is being held is not
resurrected by a frame with no usable depth.

Do not go back to a single pixel at the box centre with a zero/NaN check, as `riskam_node` once
did: a sample landing on background is a perfectly valid number, and it placed people metres too
far away.

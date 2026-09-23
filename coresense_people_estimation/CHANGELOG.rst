^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for coresense_people_estimation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. 
    please read REP 132, it contains both requirements and elaborate examples
    of changelogs. 

    link: https://www.ros.org/reps/rep-0132.html

Forthcoming
===========
Created coresense_people_estimation and added the people_estimation node

Changes:
--------
- Add the people_estimation node, also available as a composable component
    - Read depth as the median of patches around the torso keypoints, attributed to the hip midpoint
    - Back-project through the camera model into the colour optical frame
    - Fit a per-track velocity over an adaptive window chosen by reduced chi-square
- Add launch file and a parameter file documenting every parameter inline
- Add bag_camera_shim.py for replays that carry no colour camera_info
- Add dummy_tf.py to fake the TF chain when there is no localisation stack
- Add debug marker visualisation, toggled by the launch file's debug argument
- Filter out implausible and spurious detections
    - Withhold a detection until its ByteTrack id has been seen confirm_frames frames running, and drop untracked detections outright
    - Reject people printed on posters, banners and screens by testing whether anything reads behind them
    - Discard a depth reading that implies more than max_person_speed and republish the last accepted position
- Publish in the map frame for the nav2 social costmap layer
    - Transform positions into target_frame and compensate velocities for the robot's own motion
    - Coast a track the camera has lost for coast_max_s, with reliability fading to zero
    - Drop a coasted track the moment its extrapolated position falls back inside the camera image, so an in-frame occlusion is never coasted through
    - Propagate position and velocity covariances, published as tags alongside a reliability score

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
- Add debug marker visualisation, toggled by the launch file's debug argument

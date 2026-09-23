# Map transform & motion compensation

Everything above is **robot-relative**: a person standing still while the robot drives at them
reads as approaching at the robot's speed. This stage re-frames the message into `target_frame`
and takes the robot's own motion back out. Positions get the full rigid-body transform;
velocities are rotated only — a velocity is a free vector, and translating it would inject the
robot's position into it — then corrected by the transport theorem:

```
v_map = R(camera→map) · v_relative  +  v_base  +  ω × (p_person − p_base_origin)
```

all in `target_frame`, with `v_base` and `ω` the linear and angular parts of `/cmd_vel` rotated
out of `robot_base_frame`. Adding `v_base` makes a person standing still while the robot drives
past read as stationary rather than closing; `ω × r` removes the sideways sweep a stationary
person picks up whenever the robot turns, an error that grows linearly with range. The lever arm
runs from the **base** origin, since the camera's own offset cancels between its linear and
angular contributions. This consumes the body-fixed derivative, which is why the adaptive window
is *chosen* de-rotated but *fitted* body-fixed. Two limits: `cmd_vel` is what the robot was
*told* to do, so slip shows up as error, and `Twist` is unstamped, so commands are timestamped
on arrival and anything older than `cmd_vel_timeout` counts as stationary.

Every degraded path logs a warning throttled to 5s.

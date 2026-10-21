#!/usr/bin/env python3

"""
Publish the TF frames people_estimation needs but nothing else provides.

Standalone (the default) brings up the whole chain, for testing the pipeline
without a robot:

    map -> odom -> base_link +-> chassis_link -> ... (the robot's structure)
                             +-> mid_mount -> ...
                             +-> camera_link -> camera_color_frame
                                                +-> camera_color_optical_frame

- ``<camera_parent> -> camera_link`` is the camera mount: a plain REP-103
  offset, no rotation. This is the only transform published in both modes,
  because it is the only one nobody else owns -- see ``--camera-only`` below.
- ``camera_link -> camera_color_frame -> camera_color_optical_frame`` is the
  camera's own chain, reproduced exactly as the RealSense driver publishes it
- ``odom -> base_link`` integrates /cmd_vel with a unicycle model, so the base
  actually drives and turns. A static transform would leave it motionless, and
  both velocity-compensation terms in people_estimation would stay zero, hiding
  a bug in either. Pass ``--static-base`` to pin it at the origin anyway --
  quicker, and it makes hand-computed checks easy, but it is not a test of the
  velocity path. ``map -> odom`` is a static identity: nothing here localises,
  so odom exists only to give the chain the shape it has on the robot.
- Everything below base_link is the robot's structure: chassis, covers, lidars,
  imu, mid_mount and its plates and table, axle, rockers and wheels. The node
  looks none of them up -- they are what gives rviz a robot to draw, and what
  lets ``--camera-parent`` name a real mounting frame. Unlike the camera mount,
  that geometry is not invented: it is the robot's own published geometry, so a
  standalone tree measures like the real thing.

Several /cmd_vel sources are followed at once -- by default /cmd_vel and the
joystick's /bluetooth_teleop/cmd_vel -- so the base can be driven from whichever
is to hand without restarting. They are not arbitrated between: the newest
message wins, whatever it says. That is fine for one active source, but a source
publishing zeros while idle will fight one that is actually commanding, which is
why the handover is logged.

Pass ``--camera-only`` when the robot's own TF comes from somewhere else -- a rosbag
replay or a live robot. Only the camera mount is then published, since that is
the one transform such a recording does not carry: the RealSense driver
publishes the camera's internal frames, but nothing ever attaches camera_link to
the robot.

``--camera-only`` assumes whatever owns the tree owns all of it. A robot that publishes
its own odometry but runs no localisation stack breaks that assumption: its tree
is rooted at odom, there is no map frame, and every lookup fails with "map ...
does not exist" while people_estimation silently stops publishing. Add
``--static-odom`` for that case, which puts the identity map -> odom back
without touching the odometry or the structure below it. Do not reach for
``--static-base`` instead: that pins odom -> base_link, making this a second
publisher of an edge the robot already broadcasts, and the pose then flips
between the two every few milliseconds.

Note that map -> odom is the only fake that is *never* integrated. It is the
localisation correction, and the robot's own odometry already accounts for the
motion in odom -> base_link, so driving it from /cmd_vel as well would move the
base twice as far and twice as fast in map -- and double v_base in
people_estimation's velocity compensation with it.

Publishing the rest on top of a replay is actively harmful, which is why this is
a mode and not a free-for-all. odom -> base_link is stamped with the wall clock,
so against an older recording every recorded odom -> base_link lands outside
tf2's 10 s cache window and is dropped with "TF_OLD_DATA ignoring data from the
past for frame base_link" -- the detections then silently stop being
transformed. base_link would also end up with two parents (the replay's odom and
this one), which tf2 does not allow; the same goes for every structural frame,
since chassis_link and mid_mount would be claimed twice over. The static camera
frames would fight the recording's own on top of that.

Note that the camera offsets are made-up defaults that only have to look
plausible in standalone mode. Replaying with the wrong ones puts people at the
wrong place in map, and nothing warns about it, so measure the real mount.

Test tooling only. On a real robot the localisation and odometry stacks own
map -> odom -> base_link, the robot description owns the structure below it, and
the camera driver owns the camera's internal frames.

Usage:

    ros2 run people_velocity_estimation dummy_tf.py
"""

from __future__ import annotations

import argparse
import math
from functools import partial

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# The robot's structure below base_link, as
# (parent, child, (x, y, z), (roll, pitch, yaw)) in metres and radians. This is
# the robot's own geometry, not plausible-looking invention.
ROBOT_TRANSFORMS = [
    ("base_link", "chassis_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("base_link", "mid_mount", (0.0, 0.0, 0.280), (0.0, 0.0, 0.0)),
    ("chassis_link", "axle_link", (0.0, 0.0, 0.050), (0.0, 0.0, 0.0)),
    # The covers, lights and top plate really do sit at the chassis origin: they
    # are mesh-origin links, and the shape lives in the mesh rather than in the
    # joint. These zeros are the robot's values, not placeholders to fill in.
    ("chassis_link", "front_cover_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("chassis_link", "rear_cover_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("chassis_link", "left_side_cover_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("chassis_link", "right_side_cover_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("chassis_link", "front_lights_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("chassis_link", "rear_lights_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("chassis_link", "top_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("chassis_link", "front_laser", (0.3999, 0.0, 0.2337), (0.0, 0.0, 0.0)),
    # The rear lidar faces backwards, hence the yaw of pi.
    ("chassis_link", "rear_laser", (-0.3999, 0.0, 0.2337), (0.0, 0.0, math.pi)),
    ("chassis_link", "imu_link", (0.2085, -0.2902, 0.1681), (0.0, 0.0, 0.0)),
    ("mid_mount", "hams_plate_link", (-0.27913, 0.0, 0.300), (0.0, 0.0, 0.0)),
    ("mid_mount", "arm_mount_hams_plate_link", (-0.27913, 0.0, 0.600), (0.0, 0.0, 0.0)),
    ("mid_mount", "table_legs_link", (0.0, 0.0, 0.60635), (0.0, 0.0, 0.0)),
    ("mid_mount", "table_top_link", (0.0, 0.0, 0.60635), (0.0, 0.0, 0.0)),
    # Dynamic on the robot -- rocker suspension travel and wheel spin -- pinned
    # here at the real translations with identity rotation. A wheel that never
    # turns is the intended simplification, not a bug.
    ("axle_link", "front_rocker_link", (0.319, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("axle_link", "rear_rocker_link", (-0.319, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("front_rocker_link", "front_left_wheel_link", (0.0, 0.2755, 0.0), (0.0, 0.0, 0.0)),
    ("front_rocker_link", "front_right_wheel_link", (0.0, -0.2755, 0.0), (0.0, 0.0, 0.0)),
    ("rear_rocker_link", "rear_left_wheel_link", (0.0, 0.2755, 0.0), (0.0, 0.0, 0.0)),
    ("rear_rocker_link", "rear_right_wheel_link", (0.0, -0.2755, 0.0), (0.0, 0.0, 0.0)),
]

# camera_color_frame -> camera_color_optical_frame: REP-103 to the optical
# convention.
OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)

# camera_link -> camera_color_frame: the colour sensor's offset from the depth
# sensor that camera_link is aligned with. Nominal D435/D435i value, the same one
# realsense2_description uses.
COLOR_OFFSET_Y = 0.015


def quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """Convert an rpy triple to an (x, y, z, w) quaternion, ZYX convention."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def make_transform(stamp, parent: str, child: str, xyz, rpy) -> TransformStamped:
    """Fill one TransformStamped from a table entry."""
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent
    tf.child_frame_id = child
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
    (
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w,
    ) = quaternion_from_rpy(*rpy)
    return tf


class DummyTf(Node):
    """Fake the TF tree people_estimation needs when there is no robot."""

    def __init__(self, args: argparse.Namespace) -> None:
        """Broadcast the static frames, and start the odometry timer if wanted."""
        super().__init__("dummy_tf")

        self.args = args
        # A static broadcaster latches, so one send is enough -- late joiners
        # still get the whole tree.
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(self._static_transforms())

        if args.camera_only or args.static_base:
            # Nothing left to do: the base is either someone else's or pinned.
            if args.camera_only:
                joined = "camera mount" + (" and identity map -> odom" if args.static_odom else "")
                self.get_logger().info(f"dummy_tf: {joined} only, joining an existing tree")
            else:
                self.get_logger().info("dummy_tf: base pinned")
            return

        self.x, self.y, self.yaw = args.initial_x, args.initial_y, args.initial_yaw
        self.twist = Twist()
        self.active_topic = None
        self.broadcaster = TransformBroadcaster(self)
        # One shared callback per topic, with the topic bound in so the handover
        # between sources can be reported.
        self.cmd_vel_subs = [
            self.create_subscription(Twist, topic, partial(self._on_cmd_vel, topic), 10)
            for topic in args.cmd_vel_topics
        ]
        self.last_tick = self.get_clock().now()
        self.create_timer(1.0 / args.publish_rate, self._tick)

        if not args.cmd_vel_topics:
            self.get_logger().warn(
                "cmd_vel_topics is empty; the base will sit at its initial pose forever"
            )
        self.get_logger().info(
            f"dummy_tf: odom -> base_link at {args.publish_rate:.0f} Hz, "
            f"integrating {', '.join(args.cmd_vel_topics)}"
        )

    def _static_transforms(self) -> list[TransformStamped]:
        """Every transform that never changes, for the mode being run."""
        stamp = self.get_clock().now().to_msg()
        args = self.args

        # The camera mount, the one transform nobody else ever owns. Rotation is
        # identity: the optical change of axes belongs further down the chain,
        # and applying it here as well would rotate the camera twice.
        entries = [
            (
                args.camera_parent,
                "camera_link",
                (args.camera_x, args.camera_y, args.camera_z),
                (0.0, 0.0, 0.0),
            )
        ]
        # map -> odom is the localisation correction: identity when nothing
        # localises. Implied standalone, and opt-in under --camera-only for a robot that
        # publishes its own odometry but runs no localisation stack.
        if not args.camera_only or args.static_odom:
            entries.append(("map", "odom", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))

        if not args.camera_only:
            # The driver's own two-step chain rather than a composed
            # camera_link -> camera_color_optical_frame.
            entries.append(
                ("camera_link", "camera_color_frame", (0.0, COLOR_OFFSET_Y, 0.0), (0.0, 0.0, 0.0))
            )
            entries.append(
                ("camera_color_frame", "camera_color_optical_frame", (0.0, 0.0, 0.0), OPTICAL_RPY)
            )
            entries.extend(ROBOT_TRANSFORMS)
            if args.static_base:
                entries.append(("odom", "base_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))

        return [make_transform(stamp, *entry) for entry in entries]

    def _on_cmd_vel(self, topic: str, msg: Twist) -> None:
        """Latch the newest command, whichever source it arrived from."""
        # Last writer wins -- see the module docstring on why nothing arbitrates
        # here. Logging the handover is what keeps two sources fighting over the
        # base from looking like a bug in the integrator below.
        if topic != self.active_topic:
            self.get_logger().info(f"following {topic}")
            self.active_topic = topic
        self.twist = msg

    def _tick(self) -> None:
        """Integrate one step and broadcast odom -> base_link."""
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now

        # Unicycle model: the commanded twist is in the base frame, so the
        # linear part has to be rotated by the current heading before it
        # accumulates into the odom-frame position.
        self.yaw += self.twist.angular.z * dt
        self.x += (
            self.twist.linear.x * math.cos(self.yaw) - self.twist.linear.y * math.sin(self.yaw)
        ) * dt
        self.y += (
            self.twist.linear.x * math.sin(self.yaw) + self.twist.linear.y * math.cos(self.yaw)
        ) * dt

        self.broadcaster.sendTransform(
            make_transform(
                now.to_msg(), "odom", "base_link", (self.x, self.y, 0.0), (0.0, 0.0, self.yaw)
            )
        )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse the command line, leaving ROS args to rclpy."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--camera-only",
        action="store_true",
        help="The robot TF tree already comes from a replay or a real robot. Publish only "
        "the camera mount, and leave the odometry, the robot's structure and the camera "
        "optical frames to it.",
    )
    parser.add_argument(
        "--static-base",
        action="store_true",
        help="Pin odom -> base_link at the origin instead of integrating /cmd_vel.",
    )
    parser.add_argument(
        "--static-odom",
        action="store_true",
        help="Publish an identity map -> odom, for a robot that has odometry but no "
        "localisation stack. Only has an effect with --camera-only, since standalone already "
        "publishes it. Unlike --static-base this selects whether the transform is "
        "published at all, not how it is produced: map -> odom is never integrated.",
    )
    parser.add_argument(
        "--camera-parent",
        default="base_link",
        help="Frame the camera is mounted on. Must exist in the tree being joined -- "
        "standalone that is any frame in ROBOT_TRANSFORMS, e.g. mid_mount.",
    )
    parser.add_argument(
        "--camera-x", type=float, default=0.3, help="Mount offset ahead of the parent, metres."
    )
    parser.add_argument(
        "--camera-y", type=float, default=0.0, help="Mount offset left of the parent, metres."
    )
    parser.add_argument(
        "--camera-z", type=float, default=1.5, help="Mount height above the parent, metres."
    )
    parser.add_argument(
        "--cmd-vel-topics", nargs="*", default=["/cmd_vel", "/bluetooth_teleop/cmd_vel"]
    )
    parser.add_argument("--publish-rate", type=float, default=50.0)
    parser.add_argument("--initial-x", type=float, default=0.0)
    parser.add_argument("--initial-y", type=float, default=0.0)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    return parser.parse_args(argv)


def main() -> None:
    """Run the dummy TF tree until interrupted."""
    args = parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = DummyTf(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT and SIGTERM respectively; neither is an error here.
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

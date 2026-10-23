#!/usr/bin/env python3

"""
Stand in for the colour camera_info a bag replay does not carry.

people_velocity_estimation needs `/camera/camera/color/camera_info` to
back-project a pixel into a 3-D ray. A recording can capture the colour images
and not their camera_info, and the node then sits there logging "No camera_info
received yet" and never publishes a person. This script fabricates one, stamped
to match the depth stream, so a replay can drive the pipeline end to end without
re-recording:

    /camera/camera/depth/image_rect_raw  ->  /camera/camera/color/camera_info

That CameraInfo is the only thing published. Depth is consumed exactly as it
arrives, on the assumption that it is already registered to the colour frame --
see docs/plausibility.md on why an unregistered depth stream fails this pipeline silently.

The default intrinsics are nominal D435i values for 640x480 (derived from the
sensor FOVs, and matching what these cameras typically report), scaled if the
stream turns out to be a different size. Per-unit calibration varies by a
percent or two. To use your camera's real numbers, plug it in and run:

    ros2 topic echo --once /camera/camera/color/camera_info

then pass the `k` entries as `--color-intrinsics fx fy cx cy`.

Usage:

    ros2 run people_velocity_estimation bag_camera_shim.py
"""

from __future__ import annotations

import argparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image

# Nominal D435i intrinsics at 640x480, as (fx, fy, cx, cy).
#
# Colour: the RGB sensor is 16:9 native with a 42.5 deg vertical FOV, and a 4:3
# mode keeps the full height, so fx = fy = 240 / tan(42.5/2) = 617.
COLOR_INTRINSICS_640x480 = (617.0, 617.0, 319.5, 239.5)
REFERENCE_SIZE = (640, 480)


def scale_intrinsics(
    intrinsics: tuple[float, float, float, float], width: int, height: int
) -> tuple[float, float, float, float]:
    """
    Rescale intrinsics quoted for REFERENCE_SIZE to an actual image size.

    Exact for a resolution that is a pure scaling of the reference one, and
    wrong for a different aspect ratio: a RealSense mode with a different aspect
    is a *crop* of the sensor, which changes the FOV without changing the focal
    length. Pass the real values in that case.
    """
    fx, fy, cx, cy = intrinsics
    sx = width / REFERENCE_SIZE[0]
    sy = height / REFERENCE_SIZE[1]
    return (fx * sx, fy * sy, cx * sx, cy * sy)


class BagCameraShim(Node):
    """Publishes a colour CameraInfo, paced by the depth stream."""

    def __init__(self, args: argparse.Namespace) -> None:
        """Wire up the publisher and the depth subscription that paces it."""
        super().__init__("bag_camera_shim")

        self.color_intrinsics = tuple(args.color_intrinsics)
        self.color_frame = args.color_frame
        # The grid the intrinsics describe. Taken as a parameter rather than
        # read off the colour stream: subscribing to 30 Hz of RGB only to read
        # two integers costs 27 MB/s for nothing.
        self.width = args.width
        self.height = args.height

        self.info_pub = self.create_publisher(CameraInfo, args.camera_info_topic, 10)
        # Subscribed only to borrow its stamp and its rate; the image itself
        # goes to people_estimation directly, untouched.
        self.create_subscription(Image, args.depth_topic, self._depth_callback, 10)

        self.get_logger().info(
            f"bag_camera_shim started | paced by={args.depth_topic} -> "
            f"out={args.camera_info_topic} | grid={self.width}x{self.height} "
            f"fx,fy,cx,cy={self.color_intrinsics}"
        )

    def _depth_callback(self, msg: Image) -> None:
        self.info_pub.publish(self._camera_info(msg.header.stamp))

    def _camera_info(self, stamp) -> CameraInfo:
        """
        The colour CameraInfo, stamped to match the depth frame going out.

        Nothing downstream reads this stamp — people_estimation keeps the
        latest info and re-uses it — but a CameraInfo published with a stamp
        that has nothing to do with the data is a trap for the next tool.
        """
        fx, fy, cx, cy = self.color_intrinsics
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.color_frame
        info.height = self.height
        info.width = self.width
        # The bag's images are already rectified, so distortion is zero and the
        # rectification rotation is identity.
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # image_geometry back-projects from P, not K, so P is the one that has
        # to be right. Tx = Ty = 0: this is the left camera of the pair.
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse the command line, leaving ROS args to rclpy."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    # Subscribed to only for its stamp and its rate. Deliberately the raw depth
    # topic: it is published whether or not align_depth is enabled, so pacing
    # off it works in both cases. Nothing about the depth image is republished.
    parser.add_argument("--depth-topic", default="/camera/camera/depth/image_rect_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    parser.add_argument("--color-frame", default="camera_color_optical_frame")
    parser.add_argument("--width", type=int, default=REFERENCE_SIZE[0])
    parser.add_argument("--height", type=int, default=REFERENCE_SIZE[1])
    parser.add_argument(
        "--color-intrinsics",
        type=float,
        nargs=4,
        metavar=("FX", "FY", "CX", "CY"),
        default=None,
        help="Colour intrinsics. Default: nominal D435i, scaled to --width/--height.",
    )
    args = parser.parse_args(argv)

    if args.color_intrinsics is None:
        args.color_intrinsics = list(
            scale_intrinsics(COLOR_INTRINSICS_640x480, args.width, args.height)
        )
    return args


def main() -> None:
    """Run the shim until interrupted."""
    args = parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = BagCameraShim(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT and SIGTERM respectively; neither is an error here.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

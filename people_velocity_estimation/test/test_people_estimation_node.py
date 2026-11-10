"""Drive people_estimation_node through its topics, the way its users do."""

import time
from typing import NamedTuple

import numpy as np
import pytest
from conftest import NODE, TIMEOUT_S, node_parameters, people_estimation, wait_until
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, Vector3
from people_msgs.msg import People
from rcl_interfaces.msg import Log
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from riskam_msgs.msg import PersonDetection, PersonDetections
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

WIDTH, HEIGHT = 640, 480
FX, CX, CY = 500.0, 320.0, 240.0
CAMERA_FRAME = "camera_color_optical_frame"
# Farther than depth_max_m, so the depth read-out ignores it, and far enough behind
# anybody in front of it for the plausibility filter to count it as background.
BACKGROUND_M = 4.0
# Bbox corners are rounded to whole pixels; at 2 m one pixel is 2.0 / FX = 4 mm.
TOLERANCE = 0.01
# Longer than any transform_timeout set here, which the node may spend blocked on
# a TF lookup before it publishes.
FRAME_TIMEOUT_S = 2.0
NOSE, LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP = 0, 5, 6, 11, 12
DIAGNOSTICS = ["depth_valid_frac", "depth_spread_m", "position_hold_frames", "background_frac"]

pytestmark = pytest.mark.launch(fixture=people_estimation)

# Robot at (1, 0, 0) in the map, facing +y; camera 1 m up and 0.2 m ahead, looking
# forward (optical x right, y down, z ahead).
MAP_TO_BASE_T = np.array([1.0, 0.0, 0.0])
MAP_TO_BASE_Q = Quaternion(z=np.sqrt(0.5), w=np.sqrt(0.5))
BASE_TO_CAMERA_T = np.array([0.2, 0.0, 1.0])
BASE_TO_CAMERA_Q = Quaternion(x=-0.5, y=0.5, z=-0.5, w=0.5)


def rotation_matrix(q):
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


MAP_TO_BASE_R = rotation_matrix(MAP_TO_BASE_Q)
BASE_TO_CAMERA_R = rotation_matrix(BASE_TO_CAMERA_Q)


def pixel(x, y, z):
    return CX + FX * x / z, CY + FX * y / z


class Sighting(NamedTuple):
    """
    Somebody standing upright at (x, 0, z) in the camera frame, 0.5 m wide, 1.2 m tall.

    ``hip_dx`` moves the hips sideways from the box centre, so the two anchors tell apart.
    """

    track_id: int
    x: float
    z: float
    torso_keypoints: bool = True
    hip_dx: float = 0.0


def detection(sighting):
    x, z = sighting.x, sighting.z
    left, top = pixel(x - 0.25, -0.6, z)
    right, bottom = pixel(x + 0.25, 0.6, z)
    keypoints = [0.0] * 34
    placed = {NOSE: (x, -0.5)}
    if sighting.torso_keypoints:
        placed[LEFT_SHOULDER] = (x + 0.15, -0.2)
        placed[RIGHT_SHOULDER] = (x - 0.15, -0.2)
        placed[LEFT_HIP] = (x + sighting.hip_dx + 0.1, 0.1)
        placed[RIGHT_HIP] = (x + sighting.hip_dx - 0.1, 0.1)
    for k, (kx, ky) in placed.items():
        keypoints[2 * k : 2 * k + 2] = pixel(kx, ky, z)
    bbox = [round(left), round(top), round(right), round(bottom)]
    return PersonDetection(track_id=sighting.track_id, bbox=bbox, keypoints=keypoints)


def depth_image(detections, depths_m, header, encoding):
    """Paint each detection's box at its depth, over the background."""
    depth_m = np.full((HEIGHT, WIDTH), BACKGROUND_M, dtype=np.float32)
    for person, z in zip(detections, depths_m, strict=True):
        left, top, right, bottom = person.bbox
        # A real person leaves the top quarter of their box, around the head, reading
        # the background; that strip is what the plausibility filter inspects.
        # Clamped, as a negative start would wrap around the image.
        top = max(top + (bottom - top) // 4, 0)
        depth_m[top : bottom + 1, max(left, 0) : right + 1] = z
    if encoding == "16UC1":
        image = (depth_m * 1000.0).astype(np.uint16)
    elif encoding == "bgr8":
        image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    else:
        image = depth_m
    message = CvBridge().cv2_to_imgmsg(image, encoding=encoding)
    message.header = header
    return message


def stamp(seconds):
    return Time(nanoseconds=round(seconds * 1e9)).to_msg()


def xyz(vector):
    return [vector.x, vector.y, vector.z]


def vector3(array):
    return Vector3(x=array[0], y=array[1], z=array[2])


def tags(person):
    return dict(zip(person.tagnames, person.tags, strict=True))


class Rig:
    """Plays the camera, the detector, the robot and nav2 around the node."""

    def __init__(self, test_node, make_test_publisher, make_test_subscriber):
        """Connect to the node's topics; the fixtures wait until each one matches."""
        self.detections = make_test_publisher(PersonDetections, "test/detections")
        self.depth = make_test_publisher(Image, "test/depth")
        self.camera_info = make_test_publisher(CameraInfo, "test/camera_info")
        self.cmd_vel = make_test_publisher(Twist, "test/cmd_vel")
        self.people = make_test_subscriber(People, "test/people")
        # Transient local, like the node's /rosout publisher, so a warning logged
        # before the two connect still arrives.
        rosout_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.rosout = make_test_subscriber(Log, "/rosout", qos_profile=rosout_qos)
        self.tf = StaticTransformBroadcaster(test_node)

    @staticmethod
    def send(publisher, message):
        """
        Publish ``message`` and wait until the node has acknowledged it.

        The node's subscription can discover the publisher after the publisher has
        discovered it. A message sent in between reaches the node only with the
        publisher's next heartbeat, seconds later, unless this waits for it.
        """
        publisher.publish(message)
        assert publisher.wait_for_all_acked(Duration(seconds=TIMEOUT_S))

    def send_camera_info(self, fx):
        info = CameraInfo(width=WIDTH, height=HEIGHT, distortion_model="plumb_bob")
        info.header.frame_id = CAMERA_FRAME
        info.d = [0.0] * 5
        info.k = [fx, 0.0, CX, 0.0, fx, CY, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, CX, 0.0, 0.0, fx, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.send(self.camera_info, info)
        time.sleep(0.1)

    def send_cmd_vel(self, linear, angular):
        self.send(self.cmd_vel, Twist(linear=vector3(linear), angular=vector3(angular)))
        time.sleep(0.1)

    def send_robot_tf(self):
        transforms = []
        for parent, child, translation, rotation in [
            ("map", "base_link", MAP_TO_BASE_T, MAP_TO_BASE_Q),
            ("base_link", CAMERA_FRAME, BASE_TO_CAMERA_T, BASE_TO_CAMERA_Q),
        ]:
            transform = TransformStamped(child_frame_id=child)
            transform.header.frame_id = parent
            transform.transform.translation = vector3(translation)
            transform.transform.rotation = rotation
            transforms.append(transform)
        self.tf.sendTransform(transforms)
        time.sleep(0.1)

    def publish_frame(self, seconds, people, encoding="32FC1", depth_offset_s=0.0, paint_z=None):
        """Publish one frame of ``Sighting`` field tuples."""
        sightings = [Sighting(*person) for person in people]
        header = Header(frame_id=CAMERA_FRAME, stamp=stamp(seconds))
        detections = [detection(sighting) for sighting in sightings]
        self.send(self.detections, PersonDetections(header=header, detections=detections))
        depth_header = Header(frame_id=CAMERA_FRAME, stamp=stamp(seconds + depth_offset_s))
        depths_m = [sighting.z if paint_z is None else paint_z for sighting in sightings]
        self.send(self.depth, depth_image(detections, depths_m, depth_header, encoding))

    def published(self, seconds):
        """Whether People were received for the frame at ``seconds``."""
        return any(m.header.stamp == stamp(seconds) for m in self.people.received_messages)

    def publish_and_receive(self, seconds, people, **options):
        """Publish one frame, as ``publish_frame``, and wait for its People."""
        self.publish_frame(seconds, people, **options)
        return self.people.wait_for_message(
            lambda m: m.header.stamp == stamp(seconds), FRAME_TIMEOUT_S
        )

    def wait_for_log(self, text):
        """Wait until the node logs a message containing ``text``."""
        self.rosout.wait_for_message(
            lambda log: log.name == NODE and text in log.msg, FRAME_TIMEOUT_S
        )


@pytest.fixture
def run(test_node, make_test_publisher, make_test_subscriber):
    """Set up a rig around the launched node."""

    def run(send_camera_info=True, fx=FX, robot_tf=False):
        # Only once the node is up, so it discovers the rig's publishers as they are
        # created, not at their next heartbeat, which would delay the first send.
        assert wait_until(lambda: NODE in test_node.get_node_names()), f"{NODE} never came up"
        rig = Rig(test_node, make_test_publisher, make_test_subscriber)
        if robot_tf:
            rig.send_robot_tf()
        if send_camera_info:
            rig.send_camera_info(fx)
        return rig

    return run


@pytest.mark.parametrize("encoding", ["16UC1", "32FC1"])
@pytest.mark.parametrize(
    ("people_estimation", "tag_count"),
    [({"publish_covariance_tags": False}, 4), ({"publish_covariance_tags": True}, 8)],
    indirect=["people_estimation"],
    ids=["diagnostic_tags", "covariance_tags"],
)
def test_centred_person_is_published_with_its_diagnostics(run, encoding, tag_count):
    people = run().publish_and_receive(1.0, [(7, 0.0, 2.0)], encoding=encoding)
    [person] = people.people
    assert person.name == "person_7"
    # The hip midpoint, 0.1 m below the optical axis.
    assert xyz(person.position) == pytest.approx([0.0, 0.1, 2.0], abs=TOLERANCE)
    assert person.tagnames[:4] == DIAGNOSTICS
    assert len(person.tagnames) == tag_count
    assert 0.0 < person.reliability <= 1.0


def test_frame_without_detections_publishes_an_empty_people(run):
    people = run().publish_and_receive(1.0, [])
    assert people.people == []
    assert people.header.frame_id == CAMERA_FRAME


## Inputs that must result in failure to publish
# Each case breaks either the node's setup (its parameters, or the rig's ``setup``)
# or its first frame, then checks that a normal frame still gets through. The broken
# frame comes out as ``published``: None when the node drops it, [] when it publishes
# nobody. A drop shows up as the warning ``logged`` on /rosout, so the test need not
# wait out FRAME_TIMEOUT_S to see it. A broken setup gets fixed by ``repair`` before
# the normal frame; a broken frame needs no repair, since the normal frame is sent
# without its options.
@pytest.mark.parametrize(
    ("people_estimation", "setup", "frame_options", "published", "logged", "repair"),
    [
        pytest.param(
            {},
            {"send_camera_info": False},
            {},
            None,
            "No camera_info received yet",
            lambda rig: rig.send_camera_info(FX),
            id="no_camera_info",
        ),
        pytest.param(
            {},
            {"fx": 0.0},
            {},
            None,
            "No camera_info received yet",
            lambda rig: rig.send_camera_info(FX),
            id="zero_focal_length",
        ),
        pytest.param(
            {},
            {},
            {"encoding": "bgr8"},
            None,
            "Unsupported depth encoding",
            None,
            id="unsupported_encoding",
        ),
        pytest.param(
            {}, {}, {"depth_offset_s": 0.5}, None, None, None, id="depth_outside_sync_slop"
        ),
        pytest.param({}, {}, {"paint_z": 3.5}, [], None, None, id="depth_out_of_band"),
        pytest.param(
            {"target_frame": "map", "transform_timeout": 0.05},
            {},
            {},
            None,
            "Could not transform",
            Rig.send_robot_tf,
            id="missing_tf",
        ),
        pytest.param(
            {
                "target_frame": "map",
                "transform_timeout": 0.5,
                "require_cmd_vel": True,
                # Off, so the repair's cmd_vel cannot go stale before the next frame.
                "cmd_vel_timeout": 0.0,
            },
            {"robot_tf": True},
            {},
            None,
            "require_cmd_vel",
            lambda rig: rig.send_cmd_vel(np.zeros(3), np.zeros(3)),
            id="cmd_vel_required_but_missing",
        ),
    ],
    indirect=["people_estimation"],
)
def test_inputs_that_publish_nobody(run, setup, frame_options, published, logged, repair):
    rig = run(**setup)
    if published is None:
        rig.publish_frame(1.0, [(7, 0.0, 2.0)], **frame_options)
        # Check now before the repair, which could otherwise reach the node ahead of the frame.
        if logged:
            rig.wait_for_log(logged)
    else:
        assert rig.publish_and_receive(1.0, [(7, 0.0, 2.0)], **frame_options).people == published
    if repair:
        repair(rig)
    # The same person once the fault is gone, so the node is alive and listening.
    # More than one second later (2.0), out of sync_slop of the depth image left behind by e.g. the
    # slop case.
    people = rig.publish_and_receive(2.0, [(7, 0.0, 2.0)])
    assert len(people.people) == 1
    if published is None:
        # The node publishes frames in order, so a published first frame would be here.
        assert not rig.published(1.0)


def test_walking_person_is_published_at_their_speed(run):
    rig = run()
    for i in range(10):
        people = rig.publish_and_receive(1.0 + 0.1 * i, [(7, -0.5 + 0.1 * i, 2.0)])
        [person] = people.people
        # From the fifth frame the history spans more than velocity_min_span_s.
        if i >= 4:
            assert person.velocity.x == pytest.approx(1.0, abs=0.05), f"frame {i}"


@node_parameters(target_frame="map", transform_timeout=0.5, cmd_vel_timeout=0.0)
def test_cmd_vel_and_tf_reframe_into_map(run):
    rig = run(robot_tf=True)
    cmd_linear, cmd_angular = np.array([0.5, 0.0, 0.0]), np.array([0.0, 0.0, 0.4])
    rig.send_cmd_vel(cmd_linear, cmd_angular)

    # Walking at 1 m/s along optical x, so the person has a velocity of their own.
    for i in range(10):
        person_x = -0.5 + 0.1 * i
        people = rig.publish_and_receive(1.0 + 0.1 * i, [(7, person_x, 2.0)])
    assert people.header.frame_id == "map"
    [person] = people.people
    # The hip midpoint, 0.1 m below the optical axis.
    hips_in_camera = [person_x, 0.1, 2.0]
    hips_in_base = BASE_TO_CAMERA_R @ hips_in_camera + BASE_TO_CAMERA_T
    hips_in_map = MAP_TO_BASE_R @ hips_in_base + MAP_TO_BASE_T
    walking_in_map = MAP_TO_BASE_R @ BASE_TO_CAMERA_R @ [1.0, 0.0, 0.0]
    base_to_hips_in_map = hips_in_map - MAP_TO_BASE_T
    robot_motion_at_hips = MAP_TO_BASE_R @ cmd_linear + np.cross(
        MAP_TO_BASE_R @ cmd_angular, base_to_hips_in_map
    )
    assert xyz(person.position) == pytest.approx(hips_in_map.tolist(), abs=TOLERANCE)
    expected_velocity = walking_in_map + robot_motion_at_hips
    assert xyz(person.velocity) == pytest.approx(expected_velocity.tolist(), abs=0.05)


def test_position_comes_from_hip_midpoint_else_bounding_box(run):
    rig = run()
    [hips] = rig.publish_and_receive(1.0, [(7, 0.0, 2.0, True, 0.1)]).people
    assert xyz(hips.position) == pytest.approx([0.1, 0.1, 2.0], abs=TOLERANCE)
    [box] = rig.publish_and_receive(2.0, [(8, 0.3, 2.0, False)]).people
    assert xyz(box.position) == pytest.approx([0.3, 0.0, 2.0], abs=TOLERANCE)


@node_parameters(confirm_frames=3)
def test_track_is_published_only_after_confirm_frames(run):
    rig = run()
    for i in (1, 2, 3):
        people = rig.publish_and_receive(0.1 * i, [(7, -0.5, 2.0), (-1, 0.5, 2.0)])
        names = [person.name for person in people.people]
        assert names == (["person_7"] if i == 3 else []), f"frame {i}"


def test_untracked_detection_is_published_while_confirmation_is_off(run):
    people = run().publish_and_receive(1.0, [(-1, 0.0, 2.0)])
    assert [person.name for person in people.people] == ["person_untracked_0"]


def test_depth_spike_holds_the_previous_position(run):
    rig = run()
    for i in range(5):
        [person] = rig.publish_and_receive(1.0 + 0.1 * i, [(7, -0.5 + 0.1 * i, 2.0)]).people
    # 0.5 m deeper in 0.1 s is faster than max_person_speed.
    [held] = rig.publish_and_receive(1.5, [(7, 0.0, 2.0)], paint_z=2.5).people
    assert tags(held)["position_hold_frames"] == "1"
    assert xyz(held.position) == xyz(person.position)


VELOCITY_WINDOW_MAX_S = 2.0


@node_parameters(publish_covariance_tags=True, velocity_window_max_s=VELOCITY_WINDOW_MAX_S)
def test_reversal_shortens_the_velocity_window(run):
    frame_period_s, walking_speed = 0.1, 1.0
    step_m = walking_speed * frame_period_s
    rig = run()

    def walk(i, x):
        # From -1 m to 1 m at 2.5 m the box stays inside the image.
        [person] = rig.publish_and_receive(1.0 + frame_period_s * i, [(7, x, 2.5)]).people
        return person, float(tags(person)["velocity_window_s"])

    for i in range(21):
        person, window_s = walk(i, -1.0 + step_m * i)
    # The sample exactly velocity_window_max_s old is kept or dropped depending on
    # float rounding, so allow up to one frame short, but not two.
    assert window_s > VELOCITY_WINDOW_MAX_S - 1.5 * frame_period_s
    assert person.velocity.x == pytest.approx(walking_speed, abs=0.05)

    for back in range(1, 6):
        person, window_s = walk(20 + back, 1.0 - step_m * back)
    assert window_s < 1.0
    assert person.velocity.x < 0.0

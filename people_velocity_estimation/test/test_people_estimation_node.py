"""Drive people_estimation_node through its topics, the way its users do."""

import time
from typing import NamedTuple

import numpy as np
import pytest
from conftest import NODE, TIMEOUT_S, node_parameters, people_estimation, wait_until
from cv_bridge import CvBridge
from people_msgs.msg import People
from rclpy.duration import Duration
from rclpy.time import Time
from riskam_msgs.msg import PersonDetection, PersonDetections
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

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


class Rig:
    """Plays the camera, the detector and nav2 around the node."""

    def __init__(self, test_node, make_test_publisher, make_test_subscriber):
        """Connect to the node's topics; the fixtures wait until each one matches."""
        self.detections = make_test_publisher(PersonDetections, "test/detections")
        self.depth = make_test_publisher(Image, "test/depth")
        self.camera_info = make_test_publisher(CameraInfo, "test/camera_info")
        self.people = make_test_subscriber(People, "test/people")

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


@pytest.fixture
def run(test_node, make_test_publisher, make_test_subscriber):
    """Set up a rig around the launched node."""

    def run(send_camera_info=True, fx=FX):
        # Only once the node is up, so it discovers the rig's publishers as they are
        # created, not at their next heartbeat, which would delay the first send.
        assert wait_until(lambda: NODE in test_node.get_node_names()), f"{NODE} never came up"
        rig = Rig(test_node, make_test_publisher, make_test_subscriber)
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


def test_walking_person_is_published_at_their_speed(run):
    rig = run()
    for i in range(10):
        people = rig.publish_and_receive(1.0 + 0.1 * i, [(7, -0.5 + 0.1 * i, 2.0)])
        [person] = people.people
        # From the fifth frame the history spans more than velocity_min_span_s.
        if i >= 4:
            assert person.velocity.x == pytest.approx(1.0, abs=0.05), f"frame {i}"


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

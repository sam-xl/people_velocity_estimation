"""Launch the people_estimation node for each test."""

import time

import launch_pytest
import pytest
from launch import LaunchDescription
from launch_pytest.actions import ReadyToTest
from launch_ros.actions import Node

NODE = "people_estimation"
TIMEOUT_S = 10.0
TEST_PARAMETERS = {
    "detections_topic": "test/detections",
    "depth_topic": "test/depth",
    "camera_info_topic": "test/camera_info",
    "cmd_vel_topic": "test/cmd_vel",
    "people_topic": "test/people",
    "confirm_frames": 1,
    "target_frame": "",
}


def wait_until(done, timeout_s=TIMEOUT_S):
    """Poll until ``done()`` holds; False when ``timeout_s`` runs out first."""
    deadline = time.monotonic() + timeout_s
    while not done():
        if time.monotonic() > deadline:
            return False
        time.sleep(0.01)
    return True


def node_parameters(**overrides):
    """Run the decorated test with ``overrides`` on top of ``TEST_PARAMETERS``."""
    return pytest.mark.parametrize("people_estimation", [overrides], indirect=True)


@pytest.fixture
def node_absent(test_node):
    """
    Wait until the node of the previous test has left the graph.

    Every test starts a node of the same name, so a stale graph entry could let the
    readiness checks pass before this test's node is up.
    """
    gone = wait_until(lambda: NODE not in test_node.get_node_names())
    assert gone, f"{NODE} is still in the graph after its process ended"


@launch_pytest.fixture
def people_estimation(request, node_absent):
    """
    Start people_estimation_node with ``TEST_PARAMETERS`` plus the overrides in ``request.param``.

    Launch stops it with SIGINT, so the node shuts down cleanly and coverage data
    gets written.
    """
    node = Node(
        package="people_velocity_estimation",
        executable="people_estimation_node",
        name=NODE,
        parameters=[{**TEST_PARAMETERS, **getattr(request, "param", {})}],
        # Has the kernel SIGTERM the node should the test runner die without tearing down.
        prefix="setpriv --pdeathsig TERM",
    )
    return LaunchDescription([node, ReadyToTest()])

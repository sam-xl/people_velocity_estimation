"""
Check the shipped launch file and parameter file against the node's interface.

The node silently ignores a parameter file keyed on the wrong node name, and any
key it does not declare. The file is read here under the node's name, and every
key is checked against what the running node reports.
"""

import launch_pytest
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from conftest import NODE, TIMEOUT_S, wait_until
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_pytest.actions import ReadyToTest
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient

SHARE = get_package_share_directory("people_velocity_estimation")
with open(f"{SHARE}/config/people_estimation.yml") as config:
    PARAMETERS = yaml.safe_load(config)[f"/{NODE}"]["ros__parameters"]
SUBSCRIPTIONS = {
    "detections_topic": "riskam_msgs/msg/PersonDetections",
    "depth_topic": "sensor_msgs/msg/Image",
    "camera_info_topic": "sensor_msgs/msg/CameraInfo",
    "cmd_vel_topic": "geometry_msgs/msg/Twist",
}
PUBLICATIONS = {"people_topic": "people_msgs/msg/People"}


@launch_pytest.fixture
def shipped_launch_file(node_absent):
    """
    Include the shipped launch file with ``debug:=true``.

    No config argument, so it applies the shipped file from its default.
    """
    launch_file = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(f"{SHARE}/launch/people_estimation.launch"),
        launch_arguments={"debug": "true"}.items(),
    )
    return LaunchDescription([launch_file, ReadyToTest()])


@pytest.mark.launch(fixture=shipped_launch_file)
def test_launched_node_applies_the_parameter_file(test_node):
    parameters = AsyncParameterClient(test_node, NODE)
    assert parameters.wait_for_services(timeout_sec=TIMEOUT_S)

    def result(future):
        assert wait_until(future.done)
        return future.result()

    declared = result(parameters.list_parameters()).result
    assert set(PARAMETERS) - set(declared.names) == set()
    response = result(parameters.get_parameters(list(PARAMETERS)))
    values = [parameter_value_to_python(value) for value in response.values]
    # The launch file's debug argument overrides the file's key by design.
    assert dict(zip(PARAMETERS, values, strict=True)) == {**PARAMETERS, "debug": True}

    expected_subscriptions = {PARAMETERS[key]: [kind] for key, kind in SUBSCRIPTIONS.items()}
    expected_publications = {PARAMETERS[key]: [kind] for key, kind in PUBLICATIONS.items()}

    def graph():
        return (
            dict(test_node.get_subscriber_names_and_types_by_node(NODE, "/")),
            dict(test_node.get_publisher_names_and_types_by_node(NODE, "/")),
        )

    def topics_match():
        subscriptions, publications = graph()
        return (
            expected_subscriptions.items() <= subscriptions.items()
            and expected_publications.items() <= publications.items()
        )

    wait_until(topics_match)
    subscriptions, publications = graph()
    assert expected_subscriptions.items() <= subscriptions.items()
    assert expected_publications.items() <= publications.items()

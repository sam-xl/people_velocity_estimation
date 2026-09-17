"""
Bring up the people_estimation node.

Turns riskam_msgs/PersonDetections into people_msgs/People, in the colour
optical frame. The node name must stay "people_estimation":
config/people_estimation.yml keys its parameters on the fully-qualified node
name, and renaming either side silently stops them from being applied.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Declare the launch arguments and the node."""
    default_config = os.path.join(
        get_package_share_directory("coresense_people_estimation"),
        "config",
        "people_estimation.yml",
    )

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Parameter file for the people_estimation node.",
    )

    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="false",
        description=(
            "Publish debug markers and per-person TF frames. This always "
            'overrides the "debug" key in the parameter file.'
        ),
    )

    # The debug override comes after the file so it wins. LaunchConfiguration
    # substitutes to a string, so value_type is required or the node rejects
    # "false" as the wrong type.
    people_estimation_node = Node(
        package="coresense_people_estimation",
        executable="people_estimation_node",
        name="people_estimation",
        parameters=[
            LaunchConfiguration("config"),
            {"debug": ParameterValue(LaunchConfiguration("debug"), value_type=bool)},
        ],
        output="screen",
    )

    return LaunchDescription([config_arg, debug_arg, people_estimation_node])

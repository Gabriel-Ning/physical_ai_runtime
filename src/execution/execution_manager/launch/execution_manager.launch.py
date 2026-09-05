"""Launch the graph-wide Execution Manager from one embodiment profile.

No site defaults here: ``profile`` and ``max_command_age_s`` must be supplied
by the caller (normally a workstation aggregate launch that loaded apps/profiles).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                description="Absolute path to apps/profiles/<embodiment>.yaml.",
            ),
            DeclareLaunchArgument(
                "max_command_age_s",
                description="Maximum age of a streaming command before rejection.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock (/clock).",
            ),
            Node(
                package="execution_manager",
                executable="execution_manager",
                name="execution_manager",
                output="screen",
                parameters=[
                    {
                        "profile": LaunchConfiguration("profile"),
                        "max_command_age_s": ParameterValue(
                            LaunchConfiguration("max_command_age_s"),
                            value_type=float,
                        ),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )

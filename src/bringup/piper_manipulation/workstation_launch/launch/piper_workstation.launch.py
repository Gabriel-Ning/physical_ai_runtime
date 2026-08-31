"""Launch Piper workstation services and optional peripherals."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    share = get_package_share_directory("piper_manipulation_workstation_launch")
    launch_dir = os.path.join(share, "launch")
    def include(name, condition, arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=(arguments or {}).items(),
            condition=IfCondition(LaunchConfiguration(condition)))
    return LaunchDescription([
        DeclareLaunchArgument("em_config", default_value=os.path.join(share, "config", "execution_manager.yaml")),
        DeclareLaunchArgument("with_execution_manager", default_value="true"),
        DeclareLaunchArgument("with_recorder", default_value="true"),
        DeclareLaunchArgument("with_orbbec", default_value="true"),
        DeclareLaunchArgument("with_realsense", default_value="true"),
        DeclareLaunchArgument("with_leaders", default_value="true"),
        DeclareLaunchArgument("left_leader_can", default_value=""),
        DeclareLaunchArgument("right_leader_can", default_value=""),
        DeclareLaunchArgument("leader_publish_rate_hz", default_value=""),
        include("execution_manager.launch.py", "with_execution_manager", {"config": LaunchConfiguration("em_config")}),
        include("recorder.launch.py", "with_recorder"),
        include("piper_orbbec.launch.py", "with_orbbec"),
        include("piper_realsense.launch.py", "with_realsense"),
        include("piper_leaders.launch.py", "with_leaders", {
            "left_can_interface": LaunchConfiguration("left_leader_can"),
            "right_can_interface": LaunchConfiguration("right_leader_can"),
            "publish_rate_hz": LaunchConfiguration("leader_publish_rate_hz")}),
    ])

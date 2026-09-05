"""Small graph builders intended to be copied by new teleop applications."""

from isaacteleop.retargeting_engine.deviceio_source_nodes import ControllersSource
from isaacteleop.retargeting_engine.interface import OutputCombiner
from isaacteleop.teleop_session_manager import TeleopSessionConfig


def build_session_config(
    *, app_name: str, mode, mcap_config, source, retargeter, connections, outputs
):
    """Connect one DeviceIO source, one retargeter, and named public outputs."""
    connected_retargeter = retargeter.connect(
        {
            name: source.output(source_output)
            for name, source_output in connections.items()
        }
    )
    pipeline = OutputCombiner(
        {name: connected_retargeter.output(name) for name in outputs}
    )
    return TeleopSessionConfig(
        app_name=app_name,
        pipeline=pipeline,
        mode=mode,
        mcap_config=mcap_config,
    )


def build_controllers_session_config(*, app_name: str, mode, mcap_config, retargeter):
    """Quest bimanual example built with the generic one-source template."""
    controllers = ControllersSource(name="controllers")
    return build_session_config(
        app_name=app_name,
        mode=mode,
        mcap_config=mcap_config,
        source=controllers,
        retargeter=retargeter,
        connections={
            "controller_left": ControllersSource.LEFT,
            "controller_right": ControllersSource.RIGHT,
        },
        outputs=(
            "left_ee_pose",
            "right_ee_pose",
            "left_gripper",
            "right_gripper",
            "active",
            "left_active",
            "right_active",
        ),
    )

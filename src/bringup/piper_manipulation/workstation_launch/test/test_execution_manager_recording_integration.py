"""No-motion DDS integration for typed leases and stale-lease fencing."""

from __future__ import annotations

import os
import subprocess
import threading
import time
from pathlib import Path

import rclpy
import pytest
from ament_index_python.packages import get_package_prefix
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from execution_manager_interfaces.msg import LeasedJointReference
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rmi import EmbodimentConfig, ExecutionManagerClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

REPOSITORY = Path(__file__).resolve().parents[5]


class FakeControllerManager(Node):
    def __init__(self, *, context) -> None:
        super().__init__("fake_controller_manager", context=context)
        self.states = {
            "left_arm_jspc": "inactive",
            "left_arm_jtc": "inactive",
            "left_arm_tskpc": "inactive",
        }
        self.create_service(
            ListControllers,
            "/controller_manager/list_controllers",
            self._list,
        )
        self.create_service(
            SwitchController,
            "/controller_manager/switch_controller",
            self._switch,
        )

    def _list(self, request, response):
        del request
        response.controller = [
            ControllerState(name=name, state=state, type="test/Fake")
            for name, state in self.states.items()
        ]
        return response

    def _switch(self, request, response):
        for name in request.deactivate_controllers:
            if name in self.states:
                self.states[name] = "inactive"
        for name in request.activate_controllers:
            if name in self.states:
                self.states[name] = "active"
        response.ok = True
        return response


def _wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("integration condition timed out")


def _leased(node, lease_id, marker, joint_name):
    message = LeasedJointReference()
    message.header.stamp = node.get_clock().now().to_msg()
    message.lease_id = lease_id
    message.command.joint_names = [joint_name]
    message.command.points = [JointTrajectoryPoint(positions=[marker])]
    return message


@pytest.mark.parametrize(
    ("profile_name", "joint_name", "em_profile"),
    [
        ("piper_bimanual.yaml", "left_joint1", None),
        (
            "marvin_bimanual.yaml",
            "Joint1_L",
            "src/bringup/marvin_manipulation/workstation_launch/config/execution_manager.yaml",
        ),
    ],
)
def test_explicit_takeover_fences_old_lease_without_motion(
    monkeypatch, profile_name, joint_name, em_profile
):
    profile_path = REPOSITORY / "apps/profiles" / profile_name
    em_profile_path = REPOSITORY / em_profile if em_profile else profile_path
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv(
        "CYCLONEDDS_URI",
        "<CycloneDDS><Domain><General><Interfaces>"
        '<NetworkInterface address="127.0.0.1"/>'
        "</Interfaces></General></Domain></CycloneDDS>",
    )
    monkeypatch.setenv("ROS_LOG_DIR", "/tmp/physical_ai_runtime_ros_logs")
    monkeypatch.setenv("ROS_DOMAIN_ID", str(180 + os.getpid() % 40))
    context = rclpy.context.Context()
    context.init()
    manager = FakeControllerManager(context=context)
    helper = Node("typed_lease_test_client", context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(manager)
    executor.add_node(helper)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    process = None
    selection = None
    try:
        executable = (
            Path(get_package_prefix("execution_manager"))
            / "lib/execution_manager/execution_manager"
        )
        process = subprocess.Popen(
            [
                str(executable),
                "--ros-args",
                "-p",
                f"profile:={em_profile_path}",
                "-p",
                "max_command_age_s:=0.25",
            ],
            env=os.environ.copy(),
        )
        selection = ExecutionManagerClient(
            EmbodimentConfig.from_yaml(profile_path), helper, timeout_sec=5.0
        )
        selection.require_execution_manager()

        output = []
        trace = []
        helper.create_subscription(
            JointTrajectory,
            "/execution/left_arm/joint_reference",
            output.append,
            10,
        )
        helper.create_subscription(
            LeasedJointReference,
            "/execution_trace/policy/left_arm/joint_reference",
            trace.append,
            10,
        )
        policy = selection.claim(
            "POLICY", "policy-v2", {"left_arm": "joint_reference"}
        )
        policy_pub = helper.create_publisher(
            LeasedJointReference,
            policy.endpoints[("left_arm", "joint_reference")].endpoint,
            10,
        )
        policy_pub.publish(_leased(helper, policy.lease_id, 1.0, joint_name))
        _wait_for(lambda: len(output) == 1 and len(trace) == 1)

        teleop = selection.claim(
            "TELEOP",
            "operator-left",
            {"left_arm": "joint_reference"},
            preempt=True,
        )
        # A displaced client's late cleanup is idempotent and must not emit a
        # false FAULT or disturb the current owner.
        selection.release(policy.lease_id)
        policy_pub.publish(_leased(helper, policy.lease_id, 2.0, joint_name))
        time.sleep(0.1)
        assert len(output) == 1

        teleop_pub = helper.create_publisher(
            LeasedJointReference,
            teleop.endpoints[("left_arm", "joint_reference")].endpoint,
            10,
        )
        teleop_pub.publish(_leased(helper, teleop.lease_id, 3.0, joint_name))
        _wait_for(lambda: len(output) == 2)
        assert output[-1].points[-1].positions[-1] == 3.0

        selection.release(teleop.lease_id)
        teleop_pub.publish(_leased(helper, teleop.lease_id, 4.0, joint_name))
        time.sleep(0.1)
        assert len(output) == 2
    finally:
        if selection is not None:
            selection.close()
        executor.shutdown(timeout_sec=5.0)
        thread.join(timeout=5.0)
        helper.destroy_node()
        manager.destroy_node()
        if process is not None:
            process.terminate()
            process.wait(timeout=5.0)
        if context.ok():
            context.shutdown()

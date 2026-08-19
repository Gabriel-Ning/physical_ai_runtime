#!/usr/bin/env python3
"""Minimal learned-planner example: ROS state -> full trajectory -> EM/JTC."""

from __future__ import annotations

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ExampleTrajectoryPolicy:
    """Replace this class with a model that returns a complete trajectory."""

    def __init__(self, steps: int = 50, duration_s: float = 5.0) -> None:
        self.steps = steps
        self.duration_s = duration_s
        # Load a learned planner, diffusion planner, or trajectory model here.

    def predict(
        self, observation: dict[str, np.ndarray]
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return positions [T,D] and strictly increasing times [T]."""
        state = observation["observation.state"]

        # Replace with the real planner call, for example:
        # positions = self.model.plan(observation)  # [T,D]
        positions = np.repeat(state[None, :], self.steps, axis=0)  # safe hold
        times = np.linspace(
            self.duration_s / self.steps,
            self.duration_s,
            self.steps,
            dtype=np.float64,
        )
        return positions, times


class TrajectoryPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_policy_example")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3"])
        self.declare_parameter("joint_state_topic", "/joint_states")

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.policy = ExampleTrajectoryPolicy()
        self.state: np.ndarray | None = None
        self.submitted = False

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self.on_joint_state,
            qos_profile_sensor_data,
        )
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/action_sources/policy/joint_trajectory",
        )
        self.timer = self.create_timer(0.1, self.plan_once)

    def on_joint_state(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in self.joint_names):
            return
        self.state = np.asarray(
            [positions[name] for name in self.joint_names], dtype=np.float32
        )

    def plan_once(self) -> None:
        if (
            self.submitted
            or self.state is None
            or not self.trajectory_client.server_is_ready()
        ):
            return

        observation = {"observation.state": self.state.copy()}
        positions, times = self.policy.predict(observation)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.trajectory_to_ros(positions, times)
        self.trajectory_client.send_goal_async(
            goal,
            feedback_callback=self.on_feedback,
        ).add_done_callback(self.on_goal_response)
        self.submitted = True

    def on_feedback(self, _feedback) -> None:
        """Hook for recording policy execution progress."""

    def on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("EM rejected the trajectory")
            return
        goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future) -> None:
        wrapped = future.result()
        result = wrapped.result
        succeeded = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        )
        log = self.get_logger().info if succeeded else self.get_logger().error
        log(
            f"trajectory completed: status={wrapped.status}, "
            f"error_code={result.error_code}"
        )

    def trajectory_to_ros(
        self, positions: np.ndarray, times: np.ndarray
    ) -> JointTrajectory:
        positions = np.asarray(positions, dtype=np.float64)
        times = np.asarray(times, dtype=np.float64)
        if positions.ndim != 2 or positions.shape[1] != len(self.joint_names):
            raise ValueError("positions must have shape [T,D]")
        if times.shape != (positions.shape[0],) or np.any(np.diff(times) <= 0.0):
            raise ValueError("times must have shape [T] and be strictly increasing")
        if not np.isfinite(positions).all() or not np.isfinite(times).all():
            raise ValueError("trajectory contains NaN or Inf")

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        for joint_positions, time_s in zip(positions, times, strict=True):
            point = JointTrajectoryPoint()
            point.positions = joint_positions.tolist()
            nanoseconds = round(float(time_s) * 1e9)
            point.time_from_start.sec = nanoseconds // 1_000_000_000
            point.time_from_start.nanosec = nanoseconds % 1_000_000_000
            msg.points.append(point)
        return msg


def main() -> None:
    rclpy.init()
    node = TrajectoryPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

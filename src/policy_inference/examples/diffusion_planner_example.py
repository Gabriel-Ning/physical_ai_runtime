#!/usr/bin/env python3
"""Diffusion planner migration example: start/goal -> full trajectory -> EM.

Replace ``DiffusionPlanner.plan()`` with your model. Keep the request/result
dicts below unchanged so the ROS node does not need edits.

Request (dict):
  robot_model, planning_frame, joint_names
  q_pos_start / q_pos_goal     [7]
  q_vel_start / q_vel_goal     [7]
  q_acc_start / q_acc_goal     [7]

Result (dict):
  positions        [T, D]   rad
  velocities       [T, D]   rad/s
  accelerations    [T, D]   rad/s^2
  time_from_start  [T]      s, strictly increasing
  joint_names      [D]

T = number of points in the trajectory (model-defined).
D = number of joints (Panda: 7).
"""

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

PANDA_JOINTS = [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
]


class DiffusionPlanner:
    """Thin wrapper around your diffusion planner checkpoint."""

    def __init__(self) -> None:
        # checkpoint = torch.load(...)
        # self.model = ...
        pass

    def plan(self, request: dict) -> dict:
        """Run one start->goal plan. Swap the body for your real inference."""
        # ------------------------------------------------------------------
        # Migration point: call your model with ``request`` and return the
        # same result keys/shapes. Example:
        #
        #   return self.model.sample(request)
        # ------------------------------------------------------------------
        start = np.asarray(request['q_pos_start'], dtype=np.float64)
        goal = np.asarray(request['q_pos_goal'], dtype=np.float64)
        # Stub only. Your model chooses T (= number of trajectory points).
        num_points = 128
        duration_s = 10.0
        alpha = np.linspace(0.0, 1.0, num_points, dtype=np.float64)[:, None]
        positions = (1.0 - alpha) * start + alpha * goal
        return {
            'positions': positions,                    # [T, D]
            'velocities': np.zeros_like(positions),    # [T, D]
            'accelerations': np.zeros_like(positions),  # [T, D]
            'time_from_start': np.linspace(0.0, duration_s, num_points, dtype=np.float64),
            'joint_names': list(request['joint_names']),
        }


class DiffusionPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('diffusion_planner_example')
        self.declare_parameter('joint_names', PANDA_JOINTS)
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('robot_model', 'franka_panda')
        self.declare_parameter('planning_frame', 'panda_link0')
        self.declare_parameter(
            'q_pos_goal', [0.2, -0.3, 0.1, -1.8, 0.2, 1.6, 0.1]
        )

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.planner = DiffusionPlanner()
        self.state: np.ndarray | None = None
        self.submitted = False

        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_state_topic').value),
            self.on_joint_state,
            qos_profile_sensor_data,
        )
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/action_sources/policy/joint_trajectory',
        )
        self.timer = self.create_timer(0.1, self.plan_once)

    def on_joint_state(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in self.joint_names):
            return
        self.state = np.asarray(
            [positions[name] for name in self.joint_names], dtype=np.float64
        )

    def plan_once(self) -> None:
        if (
            self.submitted
            or self.state is None
            or not self.trajectory_client.server_is_ready()
        ):
            return

        zeros = np.zeros(len(self.joint_names), dtype=np.float64)
        request = {
            'robot_model': str(self.get_parameter('robot_model').value),
            'planning_frame': str(self.get_parameter('planning_frame').value),
            'joint_names': self.joint_names,
            'q_pos_start': self.state.copy(),
            'q_pos_goal': np.asarray(
                self.get_parameter('q_pos_goal').value, dtype=np.float64
            ),
            'q_vel_start': zeros.copy(),
            'q_vel_goal': zeros.copy(),
            'q_acc_start': zeros.copy(),
            'q_acc_goal': zeros.copy(),
        }
        result = self.planner.plan(request)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.result_to_ros(result)
        self.trajectory_client.send_goal_async(
            goal,
            feedback_callback=self.on_feedback,
        ).add_done_callback(self.on_goal_response)
        self.submitted = True
        self.get_logger().info(
            f'Submitted diffusion plan with {result["positions"].shape[0]} waypoints'
        )

    def on_feedback(self, _feedback) -> None:
        """Hook for recording policy execution progress."""

    def on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('EM rejected the trajectory')
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
            f'trajectory completed: status={wrapped.status}, '
            f'error_code={result.error_code}'
        )

    def result_to_ros(self, result: dict) -> JointTrajectory:
        positions = np.asarray(result['positions'], dtype=np.float64)
        velocities = np.asarray(result['velocities'], dtype=np.float64)
        accelerations = np.asarray(result['accelerations'], dtype=np.float64)
        times = np.asarray(result['time_from_start'], dtype=np.float64)
        joint_names = list(result['joint_names'])

        if joint_names != self.joint_names:
            raise ValueError('result joint_names must match the configured order')
        if positions.ndim != 2 or positions.shape[1] != len(self.joint_names):
            raise ValueError('positions must have shape [T,D]')
        if velocities.shape != positions.shape or accelerations.shape != positions.shape:
            raise ValueError('velocities/accelerations must match positions shape')
        if times.shape != (positions.shape[0],) or np.any(np.diff(times) <= 0.0):
            raise ValueError('time_from_start must be shape [T] and strictly increasing')
        if not all(
            np.isfinite(array).all()
            for array in (positions, velocities, accelerations, times)
        ):
            raise ValueError('trajectory contains NaN or Inf')

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = joint_names
        for position, velocity, acceleration, time_s in zip(
            positions, velocities, accelerations, times, strict=True
        ):
            point = JointTrajectoryPoint()
            point.positions = position.tolist()
            point.velocities = velocity.tolist()
            point.accelerations = acceleration.tolist()
            nanoseconds = round(float(time_s) * 1e9)
            point.time_from_start.sec = nanoseconds // 1_000_000_000
            point.time_from_start.nanosec = nanoseconds % 1_000_000_000
            msg.points.append(point)
        return msg


def main() -> None:
    rclpy.init()
    node = DiffusionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

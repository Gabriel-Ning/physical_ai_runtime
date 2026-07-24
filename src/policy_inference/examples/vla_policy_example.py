#!/usr/bin/env python3
"""Minimal VLA/ACT/diffusion/flow example: observation -> action chunk -> EM."""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ExampleChunkPolicy:
    """Replace this class with a model that returns an action chunk [T,D]."""

    def __init__(self, horizon: int, action_dt_s: float) -> None:
        self.horizon = horizon
        self.action_dt_s = action_dt_s
        # Load the VLA, ACT, Diffusion Policy, or Flow Matching model here.

    def predict(self, observation: dict) -> np.ndarray:
        """Return a receding-horizon joint-position action with shape [T,D]."""
        state = observation['observation.state']
        image = observation['observation.images.camera']
        task = observation['task']

        # Typical frameworks differ only inside this method:
        # action = self.model.select_action(...)  # ACT / VLA
        # action = self.model.sample_actions(...) # diffusion / flow matching
        del image, task
        return np.repeat(state[None, :], self.horizon, axis=0)  # safe hold


class VlaPolicyExample(Node):
    def __init__(self) -> None:
        super().__init__('vla_policy_example')
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3'])
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('task', 'hold position')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('horizon', 5)
        self.declare_parameter('action_dt_s', 0.1)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.task = str(self.get_parameter('task').value)
        self.horizon = int(self.get_parameter('horizon').value)
        self.action_dt_s = float(self.get_parameter('action_dt_s').value)
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.policy = ExampleChunkPolicy(self.horizon, self.action_dt_s)

        self.state: np.ndarray | None = None
        self.image: np.ndarray | None = None
        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_state_topic').value),
            self.on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self.on_image,
            qos_profile_sensor_data,
        )
        self.publisher = self.create_publisher(
            JointTrajectory, '/action_sources/policy/joint_chunk', 1
        )
        self.timer = self.create_timer(1.0 / rate_hz, self.infer)

    def on_joint_state(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in self.joint_names):
            return
        self.state = np.asarray(
            [positions[name] for name in self.joint_names], dtype=np.float32
        )

    def on_image(self, msg: Image) -> None:
        if msg.encoding.lower() not in {'rgb8', 'bgr8'}:
            self.get_logger().warning(f'Expected rgb8/bgr8, got {msg.encoding}')
            return
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        image = rows[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding.lower() == 'bgr8':
            image = image[..., ::-1]
        self.image = image.copy()

    def infer(self) -> None:
        if self.state is None or self.image is None:
            return

        # A common VLA input structure: proprioception, RGB image, language task.
        observation = {
            'observation.state': self.state.copy(),                 # [D], float32
            'observation.images.camera': self.image.copy(),         # [H,W,3], uint8
            'task': self.task,
        }

        # Optional PyTorch conversion used by many VLA models:
        # import torch
        # model_input = {
        #     'observation.state': torch.from_numpy(observation['observation.state'])
        #         .unsqueeze(0).cuda(),                               # [1,D]
        #     'observation.images.camera': torch.from_numpy(
        #         observation['observation.images.camera']
        #     ).permute(2, 0, 1).unsqueeze(0).float().cuda() / 255.0, # [1,3,H,W]
        #     'task': [observation['task']],
        # }
        # action = self.policy.model.select_action(model_input)       # [T,D]
        # action = action.detach().cpu().numpy()

        action = self.policy.predict(observation)
        self.publisher.publish(self.action_chunk_to_ros(action))

    def action_chunk_to_ros(self, action: np.ndarray) -> JointTrajectory:
        if action.shape != (self.horizon, len(self.joint_names)):
            raise ValueError(f'Expected action shape [T,D], got {action.shape}')
        if not np.isfinite(action).all():
            raise ValueError('Action contains NaN or Inf')
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        for index, positions in enumerate(action, start=1):
            point = JointTrajectoryPoint()
            point.positions = positions.astype(float).tolist()
            nanoseconds = round(index * self.action_dt_s * 1e9)
            point.time_from_start.sec = nanoseconds // 1_000_000_000
            point.time_from_start.nanosec = nanoseconds % 1_000_000_000
            msg.points.append(point)
        return msg


def main() -> None:
    rclpy.init()
    node = VlaPolicyExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

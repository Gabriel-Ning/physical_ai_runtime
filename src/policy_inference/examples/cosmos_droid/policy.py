#!/usr/bin/env python3
"""Cosmos-DROID 动作策略的 physical_ai_runtime 适配层。"""

from __future__ import annotations

import asyncio
import threading
from dataclasses import dataclass
from typing import Any

import numpy as np


_MISSING = object()


@dataclass(frozen=True)
class CosmosDroidPolicyConfig:
    """Cosmos-DROID policy client 配置。"""

    server_url: str = "ws://127.0.0.1:8000/"
    connect_timeout_s: float = 10.0
    response_timeout_s: float = 30.0
    use_concat_view: bool = True
    connect_on_init: bool = True
    return_arm_only: bool = True
    default_gripper: float = 0.0


class CosmosDroidChunkPolicy:
    """把 ROS observation 转成 Cosmos-DROID server 请求，并返回动作 chunk。"""

    def __init__(
        self,
        horizon: int,
        action_dt_s: float,
        server_url: str = "ws://127.0.0.1:8000/",
        *,
        connect_timeout_s: float = 10.0,
        response_timeout_s: float = 30.0,
        use_concat_view: bool = True,
        connect_on_init: bool = True,
        return_arm_only: bool = True,
        default_gripper: float = 0.0,
    ) -> None:
        if horizon <= 0:
            raise ValueError("horizon 必须为正数")
        self.horizon = int(horizon)
        self.action_dt_s = float(action_dt_s)
        self.config = CosmosDroidPolicyConfig(
            server_url=server_url,
            connect_timeout_s=connect_timeout_s,
            response_timeout_s=response_timeout_s,
            use_concat_view=use_concat_view,
            connect_on_init=connect_on_init,
            return_arm_only=return_arm_only,
            default_gripper=default_gripper,
        )
        self.last_raw_action_chunk: np.ndarray | None = None
        self.last_gripper_chunk: np.ndarray | None = None

        self._client: Any | None = None
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, name="cosmos-droid-client", daemon=True)
        self._thread.start()
        self._closed = False

        if self.config.connect_on_init:
            self.connect()

    def predict(self, observation: dict[str, Any]) -> np.ndarray:
        """同步预测一个 receding-horizon 动作 chunk。

        输入 observation 来自 ROS node，常用 key：

        - ``observation.state``: Franka 7 维关节位置，形状 [7] 或 [T,7]。
        - ``observation.gripper``: 夹爪标量，形状标量、[1]、[T] 或 [T,1]。
        - ``observation.images.wrist``: 腕部相机 RGB 图像，[H,W,3] uint8。
        - ``observation.images.exterior_1``: 外部相机 1 RGB 图像。
        - ``observation.images.exterior_2``: 外部相机 2 RGB 图像。
        - ``task``: 语言任务。

        返回值默认是 [horizon, 7]，用于发布 arm joint trajectory。
        Cosmos 原始 [T,8] 动作会保存在 ``last_raw_action_chunk``。
        """
        self._ensure_connected()
        cosmos_observation = self._build_cosmos_observation(observation)
        future = asyncio.run_coroutine_threadsafe(
            self._client.predict(cosmos_observation, use_concat_view=self.config.use_concat_view),
            self._loop,
        )
        result = future.result(timeout=self.config.response_timeout_s + 1.0)
        raw_action = self._extract_action_chunk(result)
        action = self._fit_horizon(raw_action, self.horizon)

        self.last_raw_action_chunk = action.copy()
        self.last_gripper_chunk = action[:, 7:8].copy()
        if self.config.return_arm_only:
            return np.ascontiguousarray(action[:, :7], dtype=np.float32)
        return np.ascontiguousarray(action, dtype=np.float32)

    def connect(self) -> None:
        """连接 Cosmos policy server。"""
        if self._closed:
            raise RuntimeError("CosmosDroidChunkPolicy 已关闭，不能重新连接")
        if self._client is not None:
            return
        future = asyncio.run_coroutine_threadsafe(self._connect_async(), self._loop)
        future.result(timeout=self.config.connect_timeout_s + 1.0)

    def close(self) -> None:
        """关闭 WebSocket client 和后台 event loop。"""
        if self._closed:
            return
        self._closed = True
        if self._client is not None and hasattr(self._client, "close"):
            future = asyncio.run_coroutine_threadsafe(self._client.close(), self._loop)
            future.result(timeout=2.0)
        self._client = None
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=2.0)

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    async def _connect_async(self) -> None:
        from tools.franka_cosmos_policy.policy_client import CosmosPolicyClient, PolicyClientConfig

        self._client = CosmosPolicyClient(
            PolicyClientConfig(
                server_url=self.config.server_url,
                connect_timeout_s=self.config.connect_timeout_s,
                response_timeout_s=self.config.response_timeout_s,
            )
        )
        await self._client.connect()

    def _ensure_connected(self) -> None:
        if self._client is None:
            self.connect()

    def _build_cosmos_observation(self, observation: dict[str, Any]) -> Any:
        from tools.franka_cosmos_policy.observation import FrankaObservation

        task = self._get_required(
            observation,
            "task",
            "prompt",
        )
        joint = self._as_joint_history(
            self._get_required(
                observation,
                "observation.state",
                "observation/joint_position",
                "joint_position",
            )
        )
        gripper = self._as_gripper_history(
            self._get_optional(
                observation,
                "observation.gripper",
                "observation/gripper_position",
                "gripper_position",
                default=_MISSING,
            ),
            fallback_from_joint=joint,
        )

        observation_image = self._get_optional(
            observation,
            "observation.image",
            "observation/image",
            "observation.images.camera",
            default=_MISSING,
        )
        if observation_image is not _MISSING:
            return FrankaObservation(
                prompt=str(task),
                joint_position=joint[:, :7],
                gripper_position=gripper,
                observation_image=observation_image,
            )

        wrist = self._get_required(
            observation,
            "observation.images.wrist",
            "observation.images.wrist_image_left",
            "observation/wrist_image_left",
            "wrist_image_left",
        )
        exterior_1 = self._get_required(
            observation,
            "observation.images.exterior_1",
            "observation.images.exterior_image_1_left",
            "observation/exterior_image_1_left",
            "exterior_image_1_left",
        )
        exterior_2 = self._get_required(
            observation,
            "observation.images.exterior_2",
            "observation.images.exterior_image_2_left",
            "observation/exterior_image_2_left",
            "exterior_image_2_left",
        )
        return FrankaObservation(
            prompt=str(task),
            joint_position=joint[:, :7],
            gripper_position=gripper,
            wrist_image_left=wrist,
            exterior_image_1_left=exterior_1,
            exterior_image_2_left=exterior_2,
        )

    def _extract_action_chunk(self, result: dict[str, Any]) -> np.ndarray:
        action_value = self._get_required(result, "action", "actions", "action_chunk")
        action = np.asarray(action_value, dtype=np.float32)
        if action.ndim == 1:
            action = action[None, :]
        if action.ndim != 2 or action.shape[1] < 8:
            raise ValueError(f"Cosmos action 必须是 [T,8+]，实际得到 {action.shape}")
        if not np.isfinite(action).all():
            raise ValueError("Cosmos action 含 NaN 或 Inf，拒绝发布")
        return np.ascontiguousarray(action[:, :8], dtype=np.float32)

    @staticmethod
    def _fit_horizon(action: np.ndarray, horizon: int) -> np.ndarray:
        if action.shape[0] <= 0:
            raise ValueError("Cosmos action chunk 为空")
        if action.shape[0] >= horizon:
            return action[:horizon].copy()
        pad = np.repeat(action[-1:, :], horizon - action.shape[0], axis=0)
        return np.concatenate([action, pad], axis=0)

    @staticmethod
    def _as_joint_history(value: Any) -> np.ndarray:
        joint = np.asarray(value, dtype=np.float32)
        if joint.ndim == 1:
            joint = joint[None, :]
        if joint.ndim != 2 or joint.shape[1] not in (7, 8):
            raise ValueError(f"observation.state 必须是 [7]、[8]、[T,7] 或 [T,8]，实际得到 {joint.shape}")
        return np.ascontiguousarray(joint, dtype=np.float32)

    def _as_gripper_history(self, value: Any, *, fallback_from_joint: np.ndarray) -> np.ndarray:
        if value is _MISSING or value is None:
            if fallback_from_joint.shape[1] >= 8:
                value = fallback_from_joint[:, 7:8]
            else:
                value = self.config.default_gripper
        gripper = np.asarray(value, dtype=np.float32)
        if gripper.ndim == 0:
            gripper = np.full((fallback_from_joint.shape[0], 1), float(gripper), dtype=np.float32)
        elif gripper.ndim == 1:
            gripper = gripper[:, None]
        if gripper.ndim != 2 or gripper.shape[1] != 1:
            raise ValueError(f"observation.gripper 必须是标量、[T] 或 [T,1]，实际得到 {gripper.shape}")
        if gripper.shape[0] == 1 and fallback_from_joint.shape[0] > 1:
            gripper = np.repeat(gripper, fallback_from_joint.shape[0], axis=0)
        if gripper.shape[0] != fallback_from_joint.shape[0]:
            raise ValueError(
                f"gripper history 长度 {gripper.shape[0]} 与 joint history 长度 {fallback_from_joint.shape[0]} 不一致"
            )
        return np.ascontiguousarray(gripper, dtype=np.float32)

    @staticmethod
    def _get_required(mapping: dict[str, Any], *keys: str) -> Any:
        value = CosmosDroidChunkPolicy._get_optional(mapping, *keys, default=_MISSING)
        if value is _MISSING:
            raise KeyError(f"缺少必要 observation/result key，候选 keys: {keys}")
        return value

    @staticmethod
    def _get_optional(mapping: dict[str, Any], *keys: str, default: Any = None) -> Any:
        for key in keys:
            if key in mapping:
                return mapping[key]
        return default

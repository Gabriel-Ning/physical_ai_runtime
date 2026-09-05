#!/usr/bin/env python3
"""Marvin dual-arm and Pika-gripper joint slider through an EM-managed Node."""

from __future__ import annotations

import argparse
import signal
import threading
import time
import tkinter as tk
from tkinter import ttk

import rmi

LEFT_ARM_JOINTS = tuple(f"Joint{index}_L" for index in range(1, 8))
RIGHT_ARM_JOINTS = tuple(f"Joint{index}_R" for index in range(1, 8))
LEFT_GRIPPER_JOINT = "left_gripper_left_joint"
RIGHT_GRIPPER_JOINT = "right_gripper_left_joint"

ARM_LIMITS = (
    (-2.9671, 2.9671),
    (-2.0944, 2.0944),
    (-2.9671, 2.9671),
    (-2.5307, 1.0472),
    (-2.9671, 2.9671),
    (-1.0472, 1.0472),
    (-1.5708, 1.5708),
)
GRIPPER_LIMITS = (0.0, 0.04)


def _clamp_step(current: float, target: float, max_step: float) -> float:
    return current + max(-max_step, min(max_step, target - current))


class SliderTeleop:
    """Tk joint target source, independent of RMI and EM lifecycle."""

    def __init__(self, *, rate_hz: float) -> None:
        self.arm_max_step = 0.3 / rate_hz
        self.gripper_max_step = 0.02 / rate_hz
        self.commanded: dict[str, float] = {}
        self.scales: dict[str, tk.Scale] = {}
        self.sync_requested = True
        self.is_open = True

        self.root = tk.Tk()
        self.root.title("Marvin Joint Slider — EM TeleopJoint")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.enabled = tk.BooleanVar(value=False)
        self.status = tk.StringVar(value="DISABLED — robot command output is off")
        self._build_ui()

    def _build_ui(self) -> None:
        header = ttk.Frame(self.root, padding=8)
        header.pack(fill=tk.X)
        ttk.Checkbutton(
            header,
            text="Enable control",
            variable=self.enabled,
            command=self._toggle,
        ).pack(side=tk.LEFT)
        ttk.Button(header, text="Sync current pose", command=self._request_sync).pack(
            side=tk.LEFT, padx=8
        )
        ttk.Label(header, textvariable=self.status).pack(side=tk.LEFT, padx=8)

        body = ttk.Frame(self.root, padding=8)
        body.pack(fill=tk.BOTH, expand=True)
        self._build_side(body, "Left", LEFT_ARM_JOINTS, LEFT_GRIPPER_JOINT, 0)
        self._build_side(body, "Right", RIGHT_ARM_JOINTS, RIGHT_GRIPPER_JOINT, 1)

    def _build_side(
        self,
        parent,
        title: str,
        arm_joints: tuple[str, ...],
        gripper_joint: str,
        column: int,
    ) -> None:
        frame = ttk.LabelFrame(parent, text=title, padding=8)
        frame.grid(row=0, column=column, padx=6, sticky="nsew")
        parent.columnconfigure(column, weight=1)
        for row, (joint, limits) in enumerate(zip(arm_joints, ARM_LIMITS, strict=True)):
            self._add_scale(frame, joint, limits, row, resolution=0.001)
        self._add_scale(
            frame, gripper_joint, GRIPPER_LIMITS, len(arm_joints), resolution=0.0001
        )

    def _add_scale(self, parent, joint, limits, row, *, resolution) -> None:
        ttk.Label(parent, text=joint, width=25).grid(row=row, column=0, sticky="w")
        scale = tk.Scale(
            parent,
            from_=limits[0],
            to=limits[1],
            resolution=resolution,
            orient=tk.HORIZONTAL,
            length=420,
            digits=5,
        )
        scale.grid(row=row, column=1, sticky="ew")
        self.scales[joint] = scale

    @staticmethod
    def _positions(observation: rmi.Observation) -> dict[str, float]:
        return dict(
            zip(
                observation.data["joint_names"],
                observation.data["joint_positions"],
                strict=False,
            )
        )

    def sync(self, observation: rmi.Observation) -> bool:
        positions = self._positions(observation)
        missing = [joint for joint in self.scales if joint not in positions]
        if missing:
            self.status.set(f"WAITING FOR JOINTS: {', '.join(missing)}")
            return False
        for joint, scale in self.scales.items():
            value = float(positions[joint])
            scale.set(value)
            self.commanded[joint] = value
        if not self.enabled.get():
            self.status.set("DISABLED — synchronized to current robot pose")
        self.sync_requested = False
        return True

    def _request_sync(self) -> None:
        self.sync_requested = True

    def _toggle(self) -> None:
        if self.enabled.get():
            self.sync_requested = True
            self.status.set("ENABLING — waiting for EM authority")
        else:
            self.status.set("DISABLED — EM authority released")

    def set_active(self) -> None:
        self.status.set("ACTIVE — slider commands own both manipulators")

    def select_action(self, observation: rmi.Observation) -> list[float] | None:
        """Return one profile-ordered 16-DoF joint-reference action."""
        if not self.enabled.get():
            return None
        if self.sync_requested and not self.sync(observation):
            return None
        for joint in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS:
            self.commanded[joint] = _clamp_step(
                self.commanded[joint],
                float(self.scales[joint].get()),
                self.arm_max_step,
            )
        for joint in (LEFT_GRIPPER_JOINT, RIGHT_GRIPPER_JOINT):
            self.commanded[joint] = _clamp_step(
                self.commanded[joint],
                float(self.scales[joint].get()),
                self.gripper_max_step,
            )
        joint_order = (
            LEFT_ARM_JOINTS
            + (LEFT_GRIPPER_JOINT,)
            + RIGHT_ARM_JOINTS
            + (RIGHT_GRIPPER_JOINT,)
        )
        return [self.commanded[joint] for joint in joint_order]

    def spin_once(self) -> None:
        self.root.update_idletasks()
        self.root.update()

    def close(self) -> None:
        self.is_open = False
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile", default="marvin_bimanual.yaml", help="Marvin RMI profile"
    )
    parser.add_argument("--rate-hz", type=float, default=30.0)
    args = parser.parse_args()
    if args.rate_hz <= 0.0:
        parser.error("--rate-hz must be positive")

    with rmi.Context.from_profile(args.profile, timeout_sec=15.0) as context:
        robot = context.robot
        slider_teleop = SliderTeleop(rate_hz=args.rate_hz)
        teleop_node = context.make_node("TeleopJoint", slider_teleop)
        context.wait_until_ready(timeout=30.0, require_execution_manager=True)
        period_s = 1.0 / args.rate_hz
        activation = None
        stop_requested = threading.Event()

        def request_stop(_signum, _frame) -> None:
            stop_requested.set()

        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        try:
            while slider_teleop.is_open and not stop_requested.is_set():
                tick_started = time.monotonic()
                slider_teleop.spin_once()
                if not slider_teleop.is_open:
                    break
                observation = robot["dual_manipulator"].get_observation()

                if slider_teleop.enabled.get() and activation is None:
                    if slider_teleop.sync(observation):
                        try:
                            activation = teleop_node.activate(preempt=True)
                            slider_teleop.set_active()
                        except Exception as exc:
                            slider_teleop.enabled.set(False)
                            slider_teleop.status.set(f"ENABLE FAILED: {exc}")
                elif not slider_teleop.enabled.get() and activation is not None:
                    activation.close()
                    activation = None

                if activation is not None:
                    action = slider_teleop.select_action(observation)
                    if action is not None:
                        teleop_node["dual_manipulator"].submit(action)

                sleep_s = period_s - (time.monotonic() - tick_started)
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
        finally:
            if activation is not None:
                try:
                    activation.close()
                except Exception as exc:
                    print(f"[SHUTDOWN WARNING] Failed to release TeleopJoint: {exc}")
            if slider_teleop.is_open:
                slider_teleop.close()


if __name__ == "__main__":
    main()

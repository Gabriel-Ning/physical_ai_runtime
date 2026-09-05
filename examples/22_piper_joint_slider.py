#!/usr/bin/env python3
"""Control both Piper arms and grippers with EM-managed Tk sliders."""

from __future__ import annotations

import argparse
import time
import tkinter as tk

import rmi

ARM_LIMITS = (
    (-2.61799, 2.61799),
    (0.0, 3.14159),
    (-2.96706, 0.0),
    (-1.74533, 1.74533),
    (-1.22173, 1.22173),
    (-6.28319, 6.28319),
)
GRIPPER_LIMITS = (0.0, 0.04)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="piper_bimanual.yaml")
    parser.add_argument("--rate-hz", type=float, default=30.0)
    args = parser.parse_args()
    if args.rate_hz <= 0.0:
        parser.error("--rate-hz must be positive")

    with rmi.Context.from_profile(args.profile, timeout_sec=15.0) as context:
        context.wait_until_ready(timeout=30.0, require_execution_manager=True)
        root = tk.Tk()
        root.title("Piper bimanual joint slider")
        enabled = tk.BooleanVar(value=False)
        tk.Checkbutton(root, text="Enable EM control", variable=enabled).grid(
            row=0, column=0, columnspan=4
        )
        scales: dict[str, tk.Scale] = {}
        for column, side in enumerate(("left", "right")):
            names = [f"{side}_joint{i}" for i in range(1, 7)] + [
                f"{side}_gripper_joint1"
            ]
            limits = ARM_LIMITS + (GRIPPER_LIMITS,)
            for row, (name, bounds) in enumerate(zip(names, limits, strict=True), 1):
                tk.Label(root, text=name).grid(row=row, column=column * 2)
                scale = tk.Scale(
                    root,
                    from_=bounds[0],
                    to=bounds[1],
                    resolution=0.001,
                    orient=tk.HORIZONTAL,
                    length=360,
                )
                scale.grid(row=row, column=column * 2 + 1)
                scales[name] = scale

        observation = context.robot["dual_manipulator"].get_observation()
        current = dict(
            zip(
                observation.data["joint_names"],
                observation.data["joint_positions"],
                strict=False,
            )
        )
        for name, scale in scales.items():
            scale.set(float(current[name]))

        nodes = {
            side: context.make_node(f"TeleopJoint_{side.capitalize()}")
            for side in ("left", "right")
        }
        activations = []
        try:
            while True:
                started = time.monotonic()
                root.update_idletasks()
                root.update()
                if enabled.get() and not activations:
                    observation = context.robot["dual_manipulator"].get_observation()
                    current = dict(
                        zip(
                            observation.data["joint_names"],
                            observation.data["joint_positions"],
                            strict=False,
                        )
                    )
                    for name, scale in scales.items():
                        scale.set(float(current[name]))
                    for node in nodes.values():
                        activation = node.activate(preempt=True)
                        activation.__enter__()
                        activations.append(activation)
                elif not enabled.get() and activations:
                    while activations:
                        activations.pop().__exit__(None, None, None)
                if activations:
                    for side, node in nodes.items():
                        arm = [scales[f"{side}_joint{i}"].get() for i in range(1, 7)]
                        gripper = [scales[f"{side}_gripper_joint1"].get()]
                        node[f"{side}_manipulator"].submit(arm + gripper)
                time.sleep(max(0.0, 1.0 / args.rate_hz - (time.monotonic() - started)))
        finally:
            while activations:
                activations.pop().__exit__(None, None, None)


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, tk.TclError):
        pass

"""Join a frozen, native-action chunk without resetting the inference producer.

This owns only an execution cursor. All inputs/outputs are unleased intents;
RMI binds each selected output to the authority captured before selection.
"""

from __future__ import annotations

import itertools
from copy import deepcopy
from dataclasses import replace
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def _rotation(value):
    return Rotation.from_quat(value, scalar_first=True)


def _slerp(a, b, u):
    return Slerp([0.0, 1.0], Rotation.concatenate([a, b]))(u)


def _position(action):
    value = action.value['position'] if action.command == 'pose_reference' else action.value
    result = np.asarray(value, dtype=float)
    if result.ndim != 1 or not np.isfinite(result).all():
        raise ValueError('handover requires finite position vectors')
    return result


class ChunkHandover:
    """Select a forward join point, bridge to it, then play the remaining chunk.

    Translation/joints use quintic Hermite interpolation with measured initial
    velocity and the chunk's outgoing tangent. Orientation uses spherical cubic
    Bezier interpolation with corresponding angular tangents. These are reference
    trajectories; controller/robot tracking and limits remain separate concerns.
    """

    def __init__(
        self,
        layout: Any,
        *,
        duration: float = 1.0,
        max_joint_gap: float = 0.6,
        max_position_gap: float = 0.25,
        max_orientation_gap: float = np.pi / 2,
    ):
        if any(
            not np.isfinite(x) or x <= 0
            for x in (
                duration,
                layout.frequency,
                max_joint_gap,
                max_position_gap,
                max_orientation_gap,
            )
        ):
            raise ValueError(
                'handover duration, frequency and gap limits must be positive and finite'
            )
        self.layout = layout
        self.duration = duration
        self.dt = 1.0 / layout.frequency
        self.max_joint_gap = max_joint_gap
        self.max_position_gap = max_position_gap
        self.max_orientation_gap = max_orientation_gap
        self.frames = ()
        self.cursor = 0
        self.join_index = None
        self.started = None
        self.reason = 'no_chunk'
        self._last_tcp = None
        self._tcp_velocity = np.zeros(3)
        self._angular_velocity = np.zeros(3)

    def observe(self, observation, now):
        sample = observation.sensors.get(self.layout.pose_part)
        if sample is None:
            return
        pose = sample.value
        position = np.asarray(pose.position_xyz, dtype=float)
        rotation = _rotation(pose.orientation_wxyz)
        self._tcp_velocity = np.zeros(3)
        self._angular_velocity = np.zeros(3)
        if self._last_tcp is not None:
            t, p, r = self._last_tcp
            dt = now - t
            if 1e-6 < dt <= 0.5:
                self._tcp_velocity = (position - p) / dt
                self._angular_velocity = -(rotation.inv() * r).as_rotvec() / dt
        self._last_tcp = (now, position, rotation)

    def _measured(self, template, observation):
        joints = dict(zip(observation.joint_names, observation.joint_positions))
        speeds = dict(zip(observation.joint_names, observation.joint_velocities))
        result, velocities = [], []
        for action in template:
            if action.command == 'joint_reference':
                group = next(g for g in self.layout.joints.groups if g.part == action.part)
                value = [joints[j] for j in group.joint_names]
                velocities.append(np.asarray([speeds.get(j, 0.0) for j in group.joint_names]))
            elif action.command == 'pose_reference':
                pose = observation.sensors[action.part].value
                value = dict(
                    action.value,
                    position=list(pose.position_xyz),
                    orientation=list(pose.orientation_wxyz),
                )
                velocities.append(self._tcp_velocity.copy())
            else:
                raise ValueError(f'unsupported handover command: {action.command}')
            result.append(replace(action, value=value))
        return tuple(result), velocities

    def start(self, frames, observation, now):
        """False means no feasible join; caller can await the next natural chunk."""
        self.frames = ()
        self.started = None
        self.join_index = None
        if not frames:
            self.reason = 'no_chunk'
            return False
        frames = deepcopy(tuple(tuple(frame) for frame in frames))
        keys = [(a.part, a.command) for a in frames[0]]
        if not keys or len(set(keys)) != len(keys):
            raise ValueError('chunk must contain a nonempty, unique action batch')
        for frame in frames:
            if [(a.part, a.command) for a in frame] != keys:
                raise ValueError('all chunk frames must use the same ordered resources')
            for action in frame:
                if action._lease_id is not None:
                    raise ValueError('handover accepts intents, not previously leased commands')
                _position(action)
                if action.command == 'pose_reference':
                    _rotation(action.value['orientation'])
        measured, velocities = self._measured(frames[0], observation)
        best = None
        grippers = set(self.layout.gripper_parts)
        for index, frame in enumerate(frames):
            score = index * 1e-4  # Prefer earlier forward points when equally suitable.
            feasible = True
            for slot, (actual, target) in enumerate(zip(measured, frame)):
                delta = _position(target) - _position(actual)
                limit = (
                    self.max_position_gap
                    if target.command == 'pose_reference'
                    else self.max_joint_gap
                )
                gap = (
                    np.linalg.norm(delta)
                    if target.command == 'pose_reference'
                    else np.max(np.abs(delta))
                )
                if gap > limit:
                    feasible = False
                    break
                if target.part not in grippers:
                    score += float(np.mean((delta / limit) ** 2))
                if target.command == 'pose_reference':
                    angle = (
                        _rotation(actual.value['orientation']).inv()
                        * _rotation(target.value['orientation'])
                    ).magnitude()
                    if angle > self.max_orientation_gap:
                        feasible = False
                        break
                    score += (angle / self.max_orientation_gap) ** 2
                if index + 1 < len(frames) and target.part not in grippers:
                    tangent = _position(frames[index + 1][slot]) - _position(target)
                    speed = velocities[slot]
                    if np.linalg.norm(delta) > 1e-6 and np.linalg.norm(tangent) > 1e-6:
                        cosine = np.dot(delta, tangent) / (
                            np.linalg.norm(delta) * np.linalg.norm(tangent)
                        )
                        score += 0.25 * (1.0 - np.clip(cosine, -1.0, 1.0))
                    if np.linalg.norm(speed) > 1e-3 and np.linalg.norm(tangent) > 1e-6:
                        cosine = np.dot(speed, tangent) / (
                            np.linalg.norm(speed) * np.linalg.norm(tangent)
                        )
                        score += 0.25 * (1.0 - np.clip(cosine, -1.0, 1.0))
            if feasible and (best is None or score < best[0]):
                best = (score, index)
        if best is None:
            self.reason = 'no_feasible_join'
            return False
        self.join_index = best[1]
        self.cursor = self.join_index + 1
        self.frames = frames
        self.started = now
        self.anchor, self.velocities = measured, velocities
        self.initial_angular_velocity = self._angular_velocity.copy()
        self.bridge_duration = self.duration
        following = frames[self.cursor] if self.cursor < len(frames) else frames[self.join_index]
        for slot, (actual, target, later) in enumerate(
            zip(measured, frames[self.join_index], following)
        ):
            distance = np.linalg.norm(_position(target) - _position(actual))
            speed = (
                np.linalg.norm(velocities[slot])
                + np.linalg.norm(_position(later) - _position(target)) / self.dt
            )
            if speed > 1e-6:
                self.bridge_duration = min(
                    self.bridge_duration, max(self.dt, 2.0 * distance / speed)
                )
        self.reason = 'joining'
        return True

    def select_action(self, now):
        if self.started is None:
            return None
        u = np.clip((now - self.started) / self.bridge_duration, 0.0, 1.0)
        if self.reason == 'joining':
            target = self.frames[self.join_index]
            following = (
                self.frames[self.join_index + 1]
                if self.join_index + 1 < len(self.frames)
                else target
            )
            alpha = 10 * u**3 - 15 * u**4 + 6 * u**5
            h0 = self.bridge_duration * (u - 6 * u**3 + 8 * u**4 - 3 * u**5)
            h1 = self.bridge_duration * (-4 * u**3 + 7 * u**4 - 3 * u**5)
            output = []
            for slot, (a, b, c) in enumerate(zip(self.anchor, target, following)):
                p0, p1 = _position(a), _position(b)
                end_velocity = (_position(c) - p1) / self.dt
                position = p0 + alpha * (p1 - p0) + h0 * self.velocities[slot] + h1 * end_velocity
                if b.command == 'pose_reference':
                    r0, r1 = _rotation(a.value['orientation']), _rotation(b.value['orientation'])
                    w1 = (r1.inv() * _rotation(c.value['orientation'])).as_rotvec() / self.dt
                    controls = [
                        r0,
                        r0
                        * Rotation.from_rotvec(
                            self.bridge_duration * self.initial_angular_velocity / 3.0
                        ),
                        r1 * Rotation.from_rotvec(-self.bridge_duration * w1 / 3.0),
                        r1,
                    ]
                    while len(controls) > 1:
                        controls = [_slerp(x, y, u) for x, y in itertools.pairwise(controls)]
                    value = dict(
                        b.value,
                        position=position.tolist(),
                        orientation=controls[0].as_quat(scalar_first=True).tolist(),
                    )
                else:
                    value = position.tolist()
                output.append(replace(b, value=value))
            if u >= 1.0:
                self.reason = 'playing_tail'
            return tuple(output)
        if self.cursor < len(self.frames):
            frame = self.frames[self.cursor]
            self.cursor += 1
            return deepcopy(frame)
        self.started = None
        self.reason = 'finished'
        return None

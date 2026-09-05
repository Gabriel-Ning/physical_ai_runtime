"""One Euro Filter implementation for 1D, Vec3, and SE(3) manifold (R^3 x SO(3))."""

from __future__ import annotations

import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def _smoothing_factor(dt: float, cutoff: float) -> float:
    if dt <= 0.0 or cutoff <= 0.0:
        return 1.0
    r = 2.0 * math.pi * cutoff * dt
    return r / (r + 1.0)


class OneEuroFilter1D:
    """1D velocity-adaptive low-pass filter (Casiez et al., CHI 2012)."""

    def __init__(
        self,
        min_cutoff: float = 1.0,
        beta: float = 0.007,
        d_cutoff: float = 1.0,
    ) -> None:
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.x_prev: float | None = None
        self.dx_prev: float = 0.0

    def reset(self) -> None:
        self.x_prev = None
        self.dx_prev = 0.0

    def filter(self, x: float, dt: float = 1.0 / 60.0) -> float:
        if self.x_prev is None or dt <= 0.0:
            self.x_prev = float(x)
            self.dx_prev = 0.0
            return float(x)

        # Estimate derivative
        dx = (float(x) - self.x_prev) / dt
        a_d = _smoothing_factor(dt, self.d_cutoff)
        dx_hat = a_d * dx + (1.0 - a_d) * self.dx_prev

        # Dynamic cutoff frequency based on speed
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = _smoothing_factor(dt, cutoff)
        x_hat = a * float(x) + (1.0 - a) * self.x_prev

        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return x_hat


class OneEuroFilterVec3:
    """3D position adaptive filter."""

    def __init__(
        self,
        min_cutoff: float = 1.0,
        beta: float = 0.007,
        d_cutoff: float = 1.0,
    ) -> None:
        self._filters = [
            OneEuroFilter1D(min_cutoff, beta, d_cutoff) for _ in range(3)
        ]

    def reset(self) -> None:
        for f in self._filters:
            f.reset()

    def filter(self, vec: np.ndarray, dt: float = 1.0 / 60.0) -> np.ndarray:
        return np.array(
            [self._filters[i].filter(float(vec[i]), dt=dt) for i in range(3)],
            dtype=vec.dtype,
        )


class OneEuroFilterSE3:
    """SE(3) One Euro Filter preserving the SO(3) rotation manifold via Slerp."""

    def __init__(
        self,
        min_cutoff_pos: float = 1.0,
        beta_pos: float = 0.007,
        min_cutoff_rot: float = 1.0,
        beta_rot: float = 0.007,
        d_cutoff: float = 1.0,
    ) -> None:
        self.pos_filter = OneEuroFilterVec3(
            min_cutoff_pos, beta_pos, d_cutoff
        )
        self.min_cutoff_rot = float(min_cutoff_rot)
        self.beta_rot = float(beta_rot)
        self.d_cutoff = float(d_cutoff)
        self.rot_prev: Rotation | None = None
        self.omega_prev: float = 0.0

    def reset(self) -> None:
        self.pos_filter.reset()
        self.rot_prev = None
        self.omega_prev = 0.0

    def filter(
        self, pos: np.ndarray, rot: Rotation, dt: float = 1.0 / 60.0
    ) -> tuple[np.ndarray, Rotation]:
        dt = max(1e-4, dt)
        pos_hat = self.pos_filter.filter(pos, dt=dt)

        if self.rot_prev is None:
            self.rot_prev = rot
            self.omega_prev = 0.0
            return pos_hat, rot

        # Angular difference on SO(3)
        delta_rot = rot * self.rot_prev.inv()
        angle = float(np.linalg.norm(delta_rot.as_rotvec()))
        omega = angle / dt

        a_d = _smoothing_factor(dt, self.d_cutoff)
        omega_hat = a_d * omega + (1.0 - a_d) * self.omega_prev

        cutoff_rot = self.min_cutoff_rot + self.beta_rot * omega_hat
        alpha_rot = _smoothing_factor(dt, cutoff_rot)

        if alpha_rot >= 0.999:
            rot_hat = rot
        elif alpha_rot <= 0.001:
            rot_hat = self.rot_prev
        else:
            rot_hat = Slerp(
                [0.0, 1.0], Rotation.concatenate([self.rot_prev, rot])
            )([alpha_rot])[0]

        self.rot_prev = rot_hat
        self.omega_prev = omega_hat
        return pos_hat, rot_hat

"""Eval/debug bookkeeping for action-chunk consumption (not part of RMI)."""

from __future__ import annotations

from typing import Any

import numpy as np


class ChunkTelemetry:
    """Tracks replan indices and raw chunk snapshots for logging."""

    def __init__(self) -> None:
        self.chunk_index = -1
        self.clear()

    def clear(self) -> None:
        self.last_raw_action: np.ndarray | None = None
        self.last_was_replan = False
        self.last_chunk_index = -1
        self.last_queue_remaining = 0
        self.last_chunk_raw: np.ndarray | None = None
        self.last_chunk0_raw: np.ndarray | None = None
        self.prev_chunk0_raw: np.ndarray | None = None

    def record_empty(self) -> None:
        self.last_raw_action = None
        self.last_was_replan = False

    def record(
        self,
        values: np.ndarray,
        *,
        was_replan: bool,
        remaining: list[Any] | None,
    ) -> None:
        self.last_raw_action = values.copy()
        self.last_was_replan = was_replan
        if was_replan:
            self.chunk_index = 0
            leftover = []
            if remaining is not None:
                for item in remaining:
                    if hasattr(item, "detach"):
                        item = item.detach().float().cpu().numpy()
                    leftover.append(np.asarray(item, dtype=np.float64).reshape(-1))
            self.last_queue_remaining = len(leftover)
            if leftover:
                self.last_chunk_raw = np.stack([values, *leftover], axis=0)
            else:
                self.last_chunk_raw = values.reshape(1, -1)
            self.prev_chunk0_raw = self.last_chunk0_raw
            self.last_chunk0_raw = values.copy()
        else:
            self.chunk_index += 1
            self.last_queue_remaining = 0 if remaining is None else len(remaining)
        self.last_chunk_index = self.chunk_index

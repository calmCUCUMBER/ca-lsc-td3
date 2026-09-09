"""Simulation-time exploration processes for ROS/PX4 training."""

from __future__ import annotations

import math

import numpy as np


class PiecewiseConstantExplorer:
    """Hold one exploration sample for a fixed interval of simulation time.

    A held sample is intentionally shared across RL control steps.  This makes
    the exploration time scale compatible with actuators that cannot follow
    independent 10 Hz action noise.  ``source`` separates warm-up actions from
    post-warm-up policy noise, so crossing ``learning_starts`` always begins a
    fresh segment.
    """

    def __init__(self, rng: np.random.Generator, *, hold_s: float) -> None:
        if not math.isfinite(hold_s) or hold_s < 0.0:
            raise ValueError("exploration hold_s must be finite and non-negative")
        self.rng = rng
        self.hold_s = float(hold_s)
        self.segment_count = 0
        self.reset()

    def reset(self) -> None:
        self._value: np.ndarray | None = None
        self._source = ""
        self._deadline_sim_s: float | None = None
        self._last_sim_time_s: float | None = None

    def _sample(
        self,
        *,
        sim_time_s: float,
        source: str,
        sampler: object,
    ) -> np.ndarray:
        if not math.isfinite(sim_time_s):
            raise ValueError("exploration sim_time_s must be finite")
        clock_rollback = (
            self._last_sim_time_s is not None
            and sim_time_s < self._last_sim_time_s - 1.0e-9
        )
        expired = (
            self._deadline_sim_s is None
            or sim_time_s >= self._deadline_sim_s - 1.0e-9
        )
        if (
            self.hold_s <= 0.0
            or self._value is None
            or self._source != source
            or clock_rollback
            or expired
        ):
            self._value = np.asarray(sampler(), dtype=np.float32)
            self._source = source
            self._deadline_sim_s = sim_time_s + self.hold_s
            self.segment_count += 1
        self._last_sim_time_s = sim_time_s
        return self._value.copy()

    def uniform_action(
        self,
        *,
        sim_time_s: float,
        shape: tuple[int, ...],
    ) -> np.ndarray:
        return self._sample(
            sim_time_s=sim_time_s,
            source="warmup_uniform_action",
            sampler=lambda: self.rng.uniform(-1.0, 1.0, size=shape),
        )

    def gaussian_noise(
        self,
        *,
        sim_time_s: float,
        shape: tuple[int, ...],
        standard_deviation: float,
    ) -> np.ndarray:
        if not math.isfinite(standard_deviation) or standard_deviation < 0.0:
            raise ValueError(
                "exploration standard_deviation must be finite and non-negative"
            )
        return self._sample(
            sim_time_s=sim_time_s,
            source="policy_gaussian_noise",
            sampler=lambda: self.rng.normal(
                0.0,
                standard_deviation,
                size=shape,
            ),
        )

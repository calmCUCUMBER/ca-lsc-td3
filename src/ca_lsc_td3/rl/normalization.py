"""Observation preprocessing used by TD3 training harnesses."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np


@dataclass(frozen=True)
class NormalizerState:
    count: int
    mean: list[float]
    variance: list[float]
    clip: float
    epsilon: float


class RunningObservationNormalizer:
    """Running mean/variance normalizer with an explicit frozen state."""

    def __init__(
        self,
        observation_dim: int,
        *,
        epsilon: float = 1.0e-6,
        clip: float = 5.0,
    ) -> None:
        if observation_dim <= 0:
            raise ValueError("observation_dim must be positive")
        if epsilon <= 0.0 or clip <= 0.0:
            raise ValueError("epsilon and clip must be positive")
        self.observation_dim = int(observation_dim)
        self.epsilon = float(epsilon)
        self.clip = float(clip)
        self.count = 0
        self.mean = np.zeros(self.observation_dim, dtype=np.float64)
        self._m2 = np.zeros(self.observation_dim, dtype=np.float64)

    def update(self, observations: np.ndarray) -> None:
        values = np.asarray(observations, dtype=np.float64)
        if values.ndim == 1:
            values = values[None, :]
        if values.shape[1] != self.observation_dim:
            raise ValueError("observation dimension mismatch")
        if not np.all(np.isfinite(values)):
            raise ValueError("normalizer observations must be finite")
        for value in values:
            self.count += 1
            delta = value - self.mean
            self.mean += delta / self.count
            delta2 = value - self.mean
            self._m2 += delta * delta2

    @property
    def variance(self) -> np.ndarray:
        if self.count < 2:
            return np.ones(self.observation_dim, dtype=np.float64)
        return np.maximum(self._m2 / (self.count - 1), self.epsilon)

    def normalize(self, observation: np.ndarray) -> np.ndarray:
        value = np.asarray(observation, dtype=np.float64)
        if value.shape[-1] != self.observation_dim:
            raise ValueError("observation dimension mismatch")
        if not np.all(np.isfinite(value)):
            raise ValueError("observation must be finite")
        if self.count < 2:
            normalized = value
        else:
            normalized = (value - self.mean) / np.sqrt(self.variance + self.epsilon)
        normalized = np.clip(normalized, -self.clip, self.clip)
        return normalized.astype(np.float32)

    def normalize_batch(self, observations: np.ndarray) -> np.ndarray:
        return self.normalize(observations)

    def state_dict(self) -> dict[str, Any]:
        return {
            "count": self.count,
            "mean": self.mean.tolist(),
            "variance": self.variance.tolist(),
            "m2": self._m2.tolist(),
            "clip": self.clip,
            "epsilon": self.epsilon,
        }

    def load_state_dict(self, payload: dict[str, Any]) -> None:
        count = int(payload["count"])
        mean = np.asarray(payload["mean"], dtype=np.float64)
        if "m2" in payload:
            m2 = np.asarray(payload["m2"], dtype=np.float64)
        else:
            variance = np.asarray(payload["variance"], dtype=np.float64)
            m2 = variance * max(count - 1, 0)
        if mean.shape != (self.observation_dim,) or m2.shape != (self.observation_dim,):
            raise ValueError("normalizer state dimension mismatch")
        if count < 0 or not math.isfinite(float(payload["clip"])):
            raise ValueError("invalid normalizer state")
        self.count = count
        self.mean = mean
        self._m2 = m2
        self.clip = float(payload["clip"])
        self.epsilon = float(payload["epsilon"])

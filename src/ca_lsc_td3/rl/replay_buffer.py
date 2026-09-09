"""Simple finite-transition replay buffer for A0 TD3 smoke."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class ReplayBatch:
    observations: np.ndarray
    actions: np.ndarray
    rewards: np.ndarray
    next_observations: np.ndarray
    terminated: np.ndarray
    truncated: np.ndarray

    @property
    def dones(self) -> np.ndarray:
        """Bellman terminal mask.

        Time-limit truncation is intentionally excluded.  If a transition
        timeout should be treated as task failure, the environment must return
        ``terminated=True`` instead of relying on the Gymnasium truncation bit.
        """
        return self.terminated


class ReplayBuffer:
    def __init__(
        self,
        *,
        capacity: int,
        observation_dim: int,
        action_dim: int,
        seed: int = 0,
    ) -> None:
        if capacity <= 0 or observation_dim <= 0 or action_dim <= 0:
            raise ValueError("capacity and dimensions must be positive")
        self.capacity = int(capacity)
        self.observation_dim = int(observation_dim)
        self.action_dim = int(action_dim)
        self._rng = np.random.default_rng(seed)
        self._observations = np.zeros((capacity, observation_dim), dtype=np.float32)
        self._actions = np.zeros((capacity, action_dim), dtype=np.float32)
        self._rewards = np.zeros((capacity, 1), dtype=np.float32)
        self._next_observations = np.zeros((capacity, observation_dim), dtype=np.float32)
        self._terminated = np.zeros((capacity, 1), dtype=np.float32)
        self._truncated = np.zeros((capacity, 1), dtype=np.float32)
        self._index = 0
        self._size = 0

    @property
    def size(self) -> int:
        return self._size

    def add(
        self,
        observation: np.ndarray,
        action: np.ndarray,
        reward: float,
        next_observation: np.ndarray,
        done: bool | None = None,
        *,
        terminated: bool | None = None,
        truncated: bool = False,
    ) -> None:
        observation = np.asarray(observation, dtype=np.float32)
        next_observation = np.asarray(next_observation, dtype=np.float32)
        action = np.asarray(action, dtype=np.float32)
        if observation.shape != (self.observation_dim,):
            raise ValueError("observation shape mismatch")
        if next_observation.shape != (self.observation_dim,):
            raise ValueError("next observation shape mismatch")
        if action.shape != (self.action_dim,):
            raise ValueError("action shape mismatch")
        if not np.all(np.isfinite(observation)):
            raise ValueError("observation must be finite")
        if not np.all(np.isfinite(next_observation)):
            raise ValueError("next observation must be finite")
        if not np.all(np.isfinite(action)):
            raise ValueError("action must be finite")
        if terminated is None:
            terminated = bool(done)
        elif done is not None and bool(done) != bool(terminated):
            raise ValueError("done and terminated disagree")
        self._observations[self._index] = observation
        self._actions[self._index] = action
        self._rewards[self._index, 0] = float(reward)
        self._next_observations[self._index] = next_observation
        self._terminated[self._index, 0] = float(bool(terminated))
        self._truncated[self._index, 0] = float(bool(truncated))
        self._index = (self._index + 1) % self.capacity
        self._size = min(self._size + 1, self.capacity)

    def sample(self, batch_size: int) -> ReplayBatch:
        if batch_size <= 0:
            raise ValueError("batch_size must be positive")
        if self._size < batch_size:
            raise ValueError("not enough samples in replay buffer")
        indices = self._rng.integers(0, self._size, size=batch_size)
        return ReplayBatch(
            observations=self._observations[indices],
            actions=self._actions[indices],
            rewards=self._rewards[indices],
            next_observations=self._next_observations[indices],
            terminated=self._terminated[indices],
            truncated=self._truncated[indices],
        )

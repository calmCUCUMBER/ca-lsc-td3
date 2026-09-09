"""Configuration contracts for the first TD3 baselines."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class A0TD3Config:
    """Frozen A0 Vanilla MLP-TD3 smoke configuration."""

    observation_dim: int = 10
    action_dim: int = 1
    actor_hidden: tuple[int, int] = (64, 64)
    critic_hidden: tuple[int, int] = (128, 128)
    gamma: float = 0.99
    tau: float = 0.005
    actor_learning_rate: float = 3.0e-4
    critic_learning_rate: float = 1.0e-3
    policy_noise: float = 0.2
    noise_clip: float = 0.5
    policy_delay: int = 2
    exploration_noise: float = 0.1
    batch_size: int = 64
    learning_starts: int = 256
    replay_capacity: int = 100_000
    max_episode_steps: int = 300
    rl_frequency_hz: float = 10.0
    energy_reward_weight: float = 0.0
    failure_penalty: float = 20.0


def action_to_lambda(action: float) -> float:
    """Map the A0 actor's tanh action in [-1, 1] to lambda_d in [0, 1]."""
    return min(1.0, max(0.0, 0.5 * (float(action) + 1.0)))

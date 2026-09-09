"""MLP actor and twin critics for A0 Vanilla TD3."""

from __future__ import annotations

import torch
from torch import nn


def _mlp(input_dim: int, hidden: tuple[int, ...], output_dim: int) -> nn.Sequential:
    layers: list[nn.Module] = []
    previous = input_dim
    for width in hidden:
        layers.extend([nn.Linear(previous, width), nn.ReLU()])
        previous = width
    layers.append(nn.Linear(previous, output_dim))
    return nn.Sequential(*layers)


class MLPActor(nn.Module):
    def __init__(
        self,
        observation_dim: int,
        action_dim: int = 1,
        hidden: tuple[int, ...] = (64, 64),
    ) -> None:
        super().__init__()
        self.net = _mlp(observation_dim, hidden, action_dim)

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        return torch.tanh(self.net(observation))


class MLPCritic(nn.Module):
    def __init__(
        self,
        observation_dim: int,
        action_dim: int = 1,
        hidden: tuple[int, ...] = (128, 128),
    ) -> None:
        super().__init__()
        self.net = _mlp(observation_dim + action_dim, hidden, 1)

    def forward(self, observation: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        return self.net(torch.cat([observation, action], dim=-1))


class TwinCritic(nn.Module):
    def __init__(
        self,
        observation_dim: int,
        action_dim: int = 1,
        hidden: tuple[int, ...] = (128, 128),
    ) -> None:
        super().__init__()
        self.q1 = MLPCritic(observation_dim, action_dim, hidden)
        self.q2 = MLPCritic(observation_dim, action_dim, hidden)

    def forward(
        self,
        observation: torch.Tensor,
        action: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        return self.q1(observation, action), self.q2(observation, action)

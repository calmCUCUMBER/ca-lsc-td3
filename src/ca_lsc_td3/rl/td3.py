"""Minimal A0 Vanilla TD3 implementation used for smoke testing."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch.nn import functional as F

from .config import A0TD3Config
from .networks import MLPActor, TwinCritic
from .replay_buffer import ReplayBatch


@dataclass(frozen=True)
class TD3Losses:
    critic1_loss: float
    critic2_loss: float
    actor_loss: float | None


class TD3Agent:
    def __init__(
        self,
        config: A0TD3Config = A0TD3Config(),
        *,
        seed: int = 0,
        device: str | torch.device = "cpu",
    ) -> None:
        self.config = config
        self.device = torch.device(device)
        torch.manual_seed(seed)
        self._rng = np.random.default_rng(seed)
        self.actor = MLPActor(
            config.observation_dim,
            config.action_dim,
            config.actor_hidden,
        ).to(self.device)
        self.actor_target = MLPActor(
            config.observation_dim,
            config.action_dim,
            config.actor_hidden,
        ).to(self.device)
        self.critic = TwinCritic(
            config.observation_dim,
            config.action_dim,
            config.critic_hidden,
        ).to(self.device)
        self.critic_target = TwinCritic(
            config.observation_dim,
            config.action_dim,
            config.critic_hidden,
        ).to(self.device)
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.actor_optimizer = torch.optim.Adam(
            self.actor.parameters(), lr=config.actor_learning_rate
        )
        self.critic_optimizer = torch.optim.Adam(
            self.critic.parameters(), lr=config.critic_learning_rate
        )
        self.total_updates = 0

    def select_action(
        self,
        observation: np.ndarray,
        *,
        explore: bool = True,
    ) -> np.ndarray:
        observation = np.asarray(observation, dtype=np.float32)
        if observation.shape != (self.config.observation_dim,):
            raise ValueError("observation shape mismatch")
        with torch.no_grad():
            tensor = torch.as_tensor(
                observation[None, :], dtype=torch.float32, device=self.device
            )
            action = self.actor(tensor).cpu().numpy()[0]
        if explore:
            action = action + self._rng.normal(
                0.0, self.config.exploration_noise, size=action.shape
            )
        return np.clip(action, -1.0, 1.0).astype(np.float32)

    def train_step(self, batch: ReplayBatch) -> TD3Losses:
        obs = torch.as_tensor(batch.observations, dtype=torch.float32, device=self.device)
        actions = torch.as_tensor(batch.actions, dtype=torch.float32, device=self.device)
        rewards = torch.as_tensor(batch.rewards, dtype=torch.float32, device=self.device)
        next_obs = torch.as_tensor(
            batch.next_observations, dtype=torch.float32, device=self.device
        )
        dones = torch.as_tensor(batch.dones, dtype=torch.float32, device=self.device)

        with torch.no_grad():
            noise = (
                torch.randn_like(actions) * self.config.policy_noise
            ).clamp(-self.config.noise_clip, self.config.noise_clip)
            next_actions = (self.actor_target(next_obs) + noise).clamp(-1.0, 1.0)
            target_q1, target_q2 = self.critic_target(next_obs, next_actions)
            target_q = torch.min(target_q1, target_q2)
            backup = rewards + self.config.gamma * (1.0 - dones) * target_q

        current_q1, current_q2 = self.critic(obs, actions)
        critic1_loss = F.mse_loss(current_q1, backup)
        critic2_loss = F.mse_loss(current_q2, backup)
        critic_loss = critic1_loss + critic2_loss
        self.critic_optimizer.zero_grad(set_to_none=True)
        critic_loss.backward()
        self.critic_optimizer.step()

        actor_loss_value: float | None = None
        if self.total_updates % self.config.policy_delay == 0:
            actor_loss = -self.critic.q1(obs, self.actor(obs)).mean()
            self.actor_optimizer.zero_grad(set_to_none=True)
            actor_loss.backward()
            self.actor_optimizer.step()
            self._soft_update(self.actor_target, self.actor)
            self._soft_update(self.critic_target, self.critic)
            actor_loss_value = float(actor_loss.detach().cpu().item())

        self.total_updates += 1
        return TD3Losses(
            critic1_loss=float(critic1_loss.detach().cpu().item()),
            critic2_loss=float(critic2_loss.detach().cpu().item()),
            actor_loss=actor_loss_value,
        )

    def _soft_update(self, target: torch.nn.Module, source: torch.nn.Module) -> None:
        tau = self.config.tau
        with torch.no_grad():
            for target_param, source_param in zip(target.parameters(), source.parameters()):
                target_param.data.mul_(1.0 - tau).add_(source_param.data, alpha=tau)

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        torch.save(
            {
                "config": self.config.__dict__,
                "actor": self.actor.state_dict(),
                "actor_target": self.actor_target.state_dict(),
                "critic": self.critic.state_dict(),
                "critic_target": self.critic_target.state_dict(),
                "actor_optimizer": self.actor_optimizer.state_dict(),
                "critic_optimizer": self.critic_optimizer.state_dict(),
                "total_updates": self.total_updates,
            },
            path,
        )

    def load(self, path: Path) -> dict[str, Any]:
        payload = torch.load(path, map_location=self.device)
        self.actor.load_state_dict(payload["actor"])
        self.actor_target.load_state_dict(payload["actor_target"])
        self.critic.load_state_dict(payload["critic"])
        self.critic_target.load_state_dict(payload["critic_target"])
        self.actor_optimizer.load_state_dict(payload["actor_optimizer"])
        self.critic_optimizer.load_state_dict(payload["critic_optimizer"])
        self.total_updates = int(payload["total_updates"])
        return payload

from __future__ import annotations

import tempfile
from pathlib import Path
import unittest

import numpy as np
import torch

from ca_lsc_td3.rl.a0_smoke import run_smoke
from ca_lsc_td3.rl.a0_smoke import run_oracle_episode
from ca_lsc_td3.rl.config import A0TD3Config, action_to_lambda
from ca_lsc_td3.rl.env import ToyA0TransitionEnv
from ca_lsc_td3.rl.networks import MLPActor, TwinCritic
from ca_lsc_td3.rl.normalization import RunningObservationNormalizer
from ca_lsc_td3.rl.replay_buffer import ReplayBuffer
from ca_lsc_td3.rl.td3 import TD3Agent


class A0TD3Tests(unittest.TestCase):
    def test_action_to_lambda_maps_tanh_range(self) -> None:
        self.assertEqual(action_to_lambda(-1.0), 0.0)
        self.assertEqual(action_to_lambda(1.0), 1.0)
        self.assertAlmostEqual(action_to_lambda(0.0), 0.5)
        self.assertEqual(action_to_lambda(-2.0), 0.0)
        self.assertEqual(action_to_lambda(2.0), 1.0)

    def test_network_shapes(self) -> None:
        config = A0TD3Config()
        obs = torch.zeros((4, config.observation_dim))
        act = torch.zeros((4, config.action_dim))
        actor = MLPActor(config.observation_dim, config.action_dim, config.actor_hidden)
        critic = TwinCritic(config.observation_dim, config.action_dim, config.critic_hidden)
        self.assertEqual(actor(obs).shape, (4, 1))
        q1, q2 = critic(obs, act)
        self.assertEqual(q1.shape, (4, 1))
        self.assertEqual(q2.shape, (4, 1))

    def test_environment_reset_and_step_are_finite(self) -> None:
        env = ToyA0TransitionEnv(seed=1)
        obs, _ = env.reset()
        self.assertEqual(obs.shape, (10,))
        self.assertTrue(np.all(np.isfinite(obs)))
        next_obs, reward, terminated, truncated, info = env.step(np.array([0.0], dtype=np.float32))
        self.assertTrue(np.all(np.isfinite(next_obs)))
        self.assertTrue(np.isfinite(reward))
        self.assertFalse(terminated and truncated)
        self.assertIn("lambda_exec", info)

    def test_replay_sample_and_td3_update_are_finite(self) -> None:
        config = A0TD3Config(batch_size=8)
        replay = ReplayBuffer(
            capacity=64,
            observation_dim=config.observation_dim,
            action_dim=config.action_dim,
            seed=2,
        )
        env = ToyA0TransitionEnv(seed=2)
        obs, _ = env.reset()
        for _ in range(16):
            action = np.array([0.0], dtype=np.float32)
            next_obs, reward, terminated, truncated, _ = env.step(action)
            replay.add(obs, action, reward, next_obs, terminated or truncated)
            obs = next_obs
        agent = TD3Agent(config, seed=2, device="cpu")
        losses = agent.train_step(replay.sample(config.batch_size))
        self.assertTrue(np.isfinite(losses.critic1_loss))
        self.assertTrue(np.isfinite(losses.critic2_loss))
        self.assertIsNotNone(losses.actor_loss)
        self.assertTrue(np.isfinite(losses.actor_loss))

    def test_replay_bootstrap_mask_ignores_truncation(self) -> None:
        replay = ReplayBuffer(capacity=4, observation_dim=10, action_dim=1, seed=7)
        obs = np.zeros(10, dtype=np.float32)
        action = np.zeros(1, dtype=np.float32)
        replay.add(obs, action, 0.0, obs, terminated=False, truncated=True)
        batch = replay.sample(1)
        self.assertEqual(float(batch.terminated[0, 0]), 0.0)
        self.assertEqual(float(batch.truncated[0, 0]), 1.0)
        self.assertEqual(float(batch.dones[0, 0]), 0.0)

    def test_running_normalizer_freezes_large_raw_scales(self) -> None:
        normalizer = RunningObservationNormalizer(10)
        observations = np.asarray([
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1800.0, 0.0],
            [15.0, 0.1, 0.0, 1.0, 0.2, 0.1, 0.1, 1.0, 0.0, 60.0],
        ], dtype=np.float32)
        normalizer.update(observations)
        scaled = normalizer.normalize(observations[0])
        self.assertEqual(scaled.shape, (10,))
        self.assertTrue(np.all(np.isfinite(scaled)))
        self.assertLessEqual(float(np.max(np.abs(scaled))), normalizer.clip)

    def test_oracle_policy_can_solve_toy_transition(self) -> None:
        result = run_oracle_episode(seed=6)
        self.assertTrue(result["success"])
        self.assertEqual(result["termination_reason"], "success")

    def test_checkpoint_save_and_load(self) -> None:
        config = A0TD3Config()
        agent = TD3Agent(config, seed=3, device="cpu")
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "agent.pt"
            agent.save(path)
            other = TD3Agent(config, seed=4, device="cpu")
            payload = other.load(path)
        self.assertIn("actor", payload)
        self.assertEqual(other.total_updates, agent.total_updates)

    def test_a0_smoke_runner_writes_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            summary = run_smoke(Path(temp), episodes=5, seed=5, device="cpu")
            root = Path(temp)
            self.assertTrue(summary["a0_td3_smoke_pass"])
            self.assertEqual(summary["energy_reward_weight"], 0.0)
            self.assertEqual(summary["learning_starts"], 256)
            self.assertEqual(summary["observation_dim"], 10)
            self.assertTrue(summary["oracle_success"])
            self.assertGreater(summary["critic_update_count"], 0)
            self.assertGreater(summary["actor_update_count"], 0)
            self.assertTrue((root / "training_metrics.csv").exists())
            self.assertTrue((root / "evaluation_telemetry.csv").exists())
            self.assertTrue((root / "a0_td3_smoke_summary.json").exists())


if __name__ == "__main__":
    unittest.main()

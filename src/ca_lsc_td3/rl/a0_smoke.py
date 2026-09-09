"""A0 Vanilla MLP-TD3 training-loop smoke runner.

The runner is intentionally lightweight: it validates the TD3 software chain
on a deterministic local transition environment before any expensive ROS/PX4
training environment is connected.  It is not a formal paper experiment.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import torch

from .config import A0TD3Config, action_to_lambda
from .env import ToyA0TransitionEnv
from .normalization import RunningObservationNormalizer
from .observations import A0_OBSERVATION_FIELDS
from .replay_buffer import ReplayBuffer
from .replay_buffer import ReplayBatch
from .td3 import TD3Agent
from .reward import COMMON_REWARD_VERSION


def _finite_or_none(value: float | None) -> float | None:
    if value is None:
        return None
    return float(value) if math.isfinite(float(value)) else None


def _run_eval_episode(
    env: ToyA0TransitionEnv,
    agent: TD3Agent,
    normalizer: RunningObservationNormalizer,
    *,
    output_csv: Path,
    seed: int,
) -> dict[str, Any]:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    observation, _ = env.reset(seed=seed)
    rows: list[dict[str, Any]] = []
    total_reward = 0.0
    success = False
    reason = "not_started"
    for step in range(agent.config.max_episode_steps):
        state_observation = observation
        action = agent.select_action(normalizer.normalize(state_observation), explore=False)
        lambda_d = action_to_lambda(float(action[0]))
        next_observation, reward, terminated, truncated, info = env.step(action)
        row = {
            "step": step,
            "reward": reward,
            "action": float(action[0]),
            "lambda_d": lambda_d,
            "lambda_exec_next": float(info["lambda_exec"]),
            "terminated": bool(terminated),
            "truncated": bool(truncated),
        }
        row.update({
            f"{field}_state": float(value)
            for field, value in zip(A0_OBSERVATION_FIELDS, state_observation)
        })
        row.update({
            f"{field}_next": float(value)
            for field, value in zip(A0_OBSERVATION_FIELDS, next_observation)
        })
        rows.append(row)
        total_reward += reward
        observation = next_observation
        success = bool(info.get("success", False))
        reason = str(info.get("termination_reason", "running"))
        if terminated or truncated:
            break
    fieldnames = [
        "step",
        "reward",
        "action",
        "lambda_d",
        "lambda_exec_next",
        "terminated",
        "truncated",
        *[f"{field}_state" for field in A0_OBSERVATION_FIELDS],
        *[f"{field}_next" for field in A0_OBSERVATION_FIELDS],
    ]
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return {
        "episode_return": total_reward,
        "episode_length": len(rows),
        "success": success,
        "termination_reason": reason,
        "telemetry_csv": str(output_csv),
    }


def _normalized_batch(
    batch: ReplayBatch,
    normalizer: RunningObservationNormalizer,
) -> ReplayBatch:
    return ReplayBatch(
        observations=normalizer.normalize_batch(batch.observations),
        actions=batch.actions,
        rewards=batch.rewards,
        next_observations=normalizer.normalize_batch(batch.next_observations),
        terminated=batch.terminated,
        truncated=batch.truncated,
    )


def run_smoke(
    output_dir: Path,
    *,
    episodes: int = 20,
    seed: int = 0,
    device: str | None = None,
) -> dict[str, Any]:
    if episodes <= 0:
        raise ValueError("episodes must be positive")
    output_dir.mkdir(parents=True, exist_ok=True)
    config = A0TD3Config()
    selected_device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    env = ToyA0TransitionEnv(seed=seed, max_steps=config.max_episode_steps)
    eval_env = ToyA0TransitionEnv(seed=seed + 10_000, max_steps=config.max_episode_steps)
    agent = TD3Agent(config, seed=seed, device=selected_device)
    normalizer = RunningObservationNormalizer(config.observation_dim)
    replay = ReplayBuffer(
        capacity=config.replay_capacity,
        observation_dim=config.observation_dim,
        action_dim=config.action_dim,
        seed=seed,
    )
    rng = np.random.default_rng(seed)

    episode_rows: list[dict[str, Any]] = []
    all_observations_finite = True
    all_losses_finite = True
    actor_update_count = 0
    critic_update_count = 0
    oracle_success = run_oracle_episode(seed=seed + 50_000)["success"]
    for episode in range(1, episodes + 1):
        observation, _ = env.reset(seed=seed + episode)
        normalizer.update(observation)
        episode_return = 0.0
        lambda_smoothness = 0.0
        max_altitude_error = 0.0
        max_abs_alpha = 0.0
        max_descent_rate = 0.0
        last_losses: tuple[float | None, float | None, float | None] = (None, None, None)
        previous_lambda = float(observation[7])
        success = False
        termination_reason = "time_limit"
        episode_length = 0

        for step in range(config.max_episode_steps):
            normalized_observation = normalizer.normalize(observation)
            if replay.size < config.learning_starts:
                action = rng.uniform(-1.0, 1.0, size=(config.action_dim,)).astype(np.float32)
            else:
                action = agent.select_action(normalized_observation, explore=True)
            next_observation, reward, terminated, truncated, info = env.step(action)
            replay.add(
                observation,
                action,
                reward,
                next_observation,
                terminated=bool(terminated),
                truncated=bool(truncated),
            )
            normalizer.update(next_observation)

            all_observations_finite = (
                all_observations_finite
                and np.all(np.isfinite(observation))
                and np.all(np.isfinite(next_observation))
            )
            lambda_exec = float(info["lambda_exec"])
            lambda_smoothness += (lambda_exec - previous_lambda) ** 2
            previous_lambda = lambda_exec
            max_altitude_error = max(max_altitude_error, float(info["max_altitude_error"]))
            max_abs_alpha = max(max_abs_alpha, float(info["max_abs_alpha"]))
            max_descent_rate = max(max_descent_rate, float(info["descent_rate"]))
            episode_return += float(reward)
            episode_length = step + 1

            if replay.size >= max(config.batch_size, config.learning_starts):
                losses = agent.train_step(
                    _normalized_batch(replay.sample(config.batch_size), normalizer)
                )
                critic_update_count += 1
                if losses.actor_loss is not None:
                    actor_update_count += 1
                loss_values = [
                    losses.critic1_loss,
                    losses.critic2_loss,
                    losses.actor_loss if losses.actor_loss is not None else 0.0,
                ]
                all_losses_finite = all_losses_finite and all(
                    math.isfinite(float(value)) for value in loss_values
                )
                last_losses = (
                    losses.critic1_loss,
                    losses.critic2_loss,
                    losses.actor_loss,
                )

            observation = next_observation
            success = bool(info.get("success", False))
            termination_reason = str(info.get("termination_reason", "running"))
            if terminated or truncated:
                break

        episode_rows.append({
            "episode": episode,
            "episode_return": episode_return,
            "episode_length": episode_length,
            "success": success,
            "termination_reason": termination_reason,
            "max_altitude_error": max_altitude_error,
            "max_abs_alpha": max_abs_alpha,
            "max_descent_rate": max_descent_rate,
            "lambda_smoothness": lambda_smoothness,
            "critic1_loss": _finite_or_none(last_losses[0]),
            "critic2_loss": _finite_or_none(last_losses[1]),
            "actor_loss": _finite_or_none(last_losses[2]),
            "replay_size": replay.size,
        })

    metrics_csv = output_dir / "training_metrics.csv"
    with metrics_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(episode_rows[0].keys()))
        writer.writeheader()
        writer.writerows(episode_rows)

    checkpoint = output_dir / "checkpoints" / "a0_td3_smoke.pt"
    agent.save(checkpoint)
    reload_agent = TD3Agent(config, seed=seed + 1, device=selected_device)
    reload_payload = reload_agent.load(checkpoint)
    checkpoint_reload_ok = int(reload_payload.get("total_updates", -1)) == agent.total_updates

    eval_result = _run_eval_episode(
        eval_env,
        reload_agent,
        normalizer,
        output_csv=output_dir / "evaluation_telemetry.csv",
        seed=seed + 20_000,
    )

    pass_failures: list[str] = []
    if not all_observations_finite:
        pass_failures.append("nonfinite_observation")
    if not all_losses_finite:
        pass_failures.append("nonfinite_loss")
    if critic_update_count <= 0:
        pass_failures.append("critic_not_updated")
    if actor_update_count <= 0:
        pass_failures.append("actor_not_updated")
    if replay.size < config.batch_size:
        pass_failures.append("replay_not_populated")
    if not oracle_success:
        pass_failures.append("oracle_policy_cannot_succeed")
    if not checkpoint.exists() or not checkpoint_reload_ok:
        pass_failures.append("checkpoint_save_load_failed")
    if len(episode_rows) != episodes:
        pass_failures.append("episode_loop_incomplete")

    summary = {
        "stage": "A0 Vanilla MLP-TD3 smoke",
        "a0_td3_smoke_pass": not pass_failures,
        "pass_failures": pass_failures,
        "episodes": episodes,
        "seed": seed,
        "device": selected_device,
        "observation_fields": list(A0_OBSERVATION_FIELDS),
        "observation_dim": config.observation_dim,
        "action_dim": config.action_dim,
        "actor_architecture": [config.observation_dim, *config.actor_hidden, config.action_dim],
        "critic_architecture": [config.observation_dim + config.action_dim, *config.critic_hidden, 1],
        "energy_reward_weight": config.energy_reward_weight,
        "reward_version": COMMON_REWARD_VERSION,
        "learning_starts": config.learning_starts,
        "critic_update_count": critic_update_count,
        "actor_update_count": actor_update_count,
        "replay_size": replay.size,
        "normalizer": normalizer.state_dict(),
        "oracle_success": oracle_success,
        "checkpoint": str(checkpoint),
        "training_metrics_csv": str(metrics_csv),
        "evaluation": eval_result,
        "note": "Local training-code smoke only; not a formal ROS/PX4 paper experiment.",
    }
    (output_dir / "a0_td3_smoke_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return summary


def oracle_action_for_observation(observation: np.ndarray) -> np.ndarray:
    """Return a simple reachable lambda schedule for the toy transition plant."""
    airspeed = float(observation[0])
    wing_support = np.clip((airspeed / 15.0) ** 2, 0.0, 1.0)
    lambda_target = float(np.clip(wing_support, 0.0, 1.0))
    return np.asarray([2.0 * lambda_target - 1.0], dtype=np.float32)


def run_oracle_episode(*, seed: int = 0) -> dict[str, Any]:
    env = ToyA0TransitionEnv(seed=seed, max_steps=A0TD3Config().max_episode_steps)
    observation, _ = env.reset(seed=seed)
    total_reward = 0.0
    reason = "time_limit"
    success = False
    for step in range(A0TD3Config().max_episode_steps):
        action = oracle_action_for_observation(observation)
        observation, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        success = bool(info.get("success", False))
        reason = str(info.get("termination_reason", "running"))
        if terminated or truncated:
            return {
                "success": success,
                "termination_reason": reason,
                "episode_length": step + 1,
                "episode_return": total_reward,
            }
    return {
        "success": success,
        "termination_reason": reason,
        "episode_length": A0TD3Config().max_episode_steps,
        "episode_return": total_reward,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default=None)
    args = parser.parse_args()
    summary = run_smoke(
        args.output_dir,
        episodes=args.episodes,
        seed=args.seed,
        device=args.device,
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["a0_td3_smoke_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

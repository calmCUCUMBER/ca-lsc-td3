"""A0 Vanilla TD3 online ROS/PX4 training smoke.

This is an integration smoke, not the formal paper training protocol.  It
starts one SITL transition per episode, publishes online A0 actions at 10 Hz
simulation time, stores real transitions, updates the existing TD3Agent, and
saves a checkpoint plus metrics.
"""

from __future__ import annotations

import argparse
from collections import Counter
import copy
import csv
from dataclasses import asdict, replace
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
from typing import Any

import numpy as np

from .config import A0TD3Config, action_to_lambda
from .exploration import PiecewiseConstantExplorer
from .normalization import RunningObservationNormalizer
from .replay_buffer import ReplayBuffer, ReplayBatch
from .td3 import TD3Agent
from .reward import (
    COMMON_REWARD_VERSION,
    DebugRewardWeights,
    common_reward_terms,
)


TASK_TERMINATION_REASONS = frozenset({
    "success",
    "forward_transition_timeout",
    "airspeed_hold_runaway",
    "va_hold_altitude_runaway",
    "va_hold_vertical_speed_runaway",
    "pre_measurement_altitude_transient",
    "px4_failsafe",
})

INFRASTRUCTURE_TERMINATION_REASONS = frozenset({
    "a0_rl_action_timeout",
    "initial_rl_state_pre_rl_readiness_failed",
    "initial_rl_state_pre_rl_readiness_unavailable",
    "terminal_not_observed",
    "runner_wall_timeout",
    "launch_process_error",
})

ACTION_SOURCE_ACTOR_DETERMINISTIC = "actor_deterministic"
ACTION_SOURCE_WARMUP_UNIFORM = "warmup_uniform"
ACTION_SOURCE_PERSISTENT_EXPLORATION = "persistent_exploration"
ACTION_SOURCE_FIXED = "fixed"
ACTION_SOURCE_FORCED = "forced"
ACTION_SOURCE_SAFE_KEEPALIVE = "safe_keepalive"
EPISODE_ACTION_SOURCES = (
    ACTION_SOURCE_ACTOR_DETERMINISTIC,
    ACTION_SOURCE_WARMUP_UNIFORM,
    ACTION_SOURCE_PERSISTENT_EXPLORATION,
    ACTION_SOURCE_FIXED,
    ACTION_SOURCE_FORCED,
)


def _terminal_semantics(
    *,
    reason: str,
    terminated: bool,
    truncated: bool,
) -> tuple[bool, bool, bool]:
    """Return Bellman-terminal, truncation, and task-failure semantics.

    Upstream ROS messages use ``terminated`` to mean that the process should
    stop.  That is broader than an MDP terminal: runtime/action transport
    failures must bootstrap and must not receive the task failure penalty.
    Unknown stop reasons are conservatively treated as infrastructure
    truncations until they are explicitly classified.
    """
    normalized = reason.strip().lower()
    if not terminated and not truncated:
        return False, False, False
    if normalized in TASK_TERMINATION_REASONS:
        task_failure = normalized != "success"
        return True, False, task_failure
    return False, True, False


def _attempt_infrastructure_failure(
    *,
    terminal_seen: bool,
    reason: str,
    rl_active_observation_count: int,
    launch_status: int,
    timed_out: bool,
    initialization_invalid_reason: str = "",
) -> str:
    if timed_out:
        return "runner_wall_timeout"
    if launch_status != 0:
        return "launch_process_error"
    if not terminal_seen:
        return "terminal_not_observed"
    if initialization_invalid_reason:
        return initialization_invalid_reason
    if reason in INFRASTRUCTURE_TERMINATION_REASONS:
        return reason
    if rl_active_observation_count <= 0:
        return "pre_rl_abort_before_rl_active"
    if reason not in TASK_TERMINATION_REASONS:
        return "unclassified_terminal_reason"
    return ""


def _finite(value: object, default: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _payload_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    text = str(value).strip().lower()
    if text in {"true", "yes", "on"}:
        return True
    if text in {"false", "no", "off", ""}:
        return False
    try:
        return bool(int(float(text)))
    except (TypeError, ValueError):
        return False


def _latched_initial_readiness(
    payload: dict[str, Any],
) -> tuple[bool, bool, bool, bool, bool, str]:
    """Decode the authoritative transition-entry readiness snapshot."""
    keys = (
        "a0_rl_initial_height_gate_ok",
        "a0_rl_initial_vz_gate_ok",
        "a0_rl_initial_groundspeed_gate_ok",
    )
    available = (
        _payload_bool(payload.get("a0_rl_initial_readiness_available"))
        and all(key in payload for key in keys)
    )
    height_ok = _payload_bool(payload.get(keys[0]))
    vz_ok = _payload_bool(payload.get(keys[1]))
    groundspeed_ok = _payload_bool(payload.get(keys[2]))
    ready = (
        available
        and _payload_bool(payload.get("a0_rl_initial_readiness_latched"))
        and height_ok
        and vz_ok
        and groundspeed_ok
    )
    reason = ""
    if not ready:
        reason = (
            "initial_rl_state_pre_rl_readiness_unavailable"
            if not available
            else "initial_rl_state_pre_rl_readiness_failed"
        )
    return available, height_ok, vz_ok, groundspeed_ok, ready, reason


def _json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


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


def _parse_env_file(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    if not path.exists():
        return result
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        key, value = stripped.split("=", 1)
        result[key.strip()] = value.strip().strip('"').strip("'")
    return result


def _mean_or_none(values: list[float]) -> float | None:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return None
    return float(np.mean(finite))


def _p95_or_none(values: list[float]) -> float | None:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return None
    return float(np.percentile(finite, 95.0))


def _metric_summary(rows: list[dict[str, Any]], key: str) -> dict[str, float | int | None]:
    finite = [
        value
        for row in rows
        if math.isfinite(value := _finite(row.get(key)))
    ]
    if not finite:
        return {"count": 0, "mean": None, "median": None, "p95": None}
    return {
        "count": len(finite),
        "mean": float(np.mean(finite)),
        "median": float(np.median(finite)),
        "p95": float(np.percentile(finite, 95.0)),
    }


def _agent_parameter_digest(agent: TD3Agent) -> str:
    """Hash all learned tensors so evaluation can prove the policy stayed frozen."""
    digest = hashlib.sha256()
    for module_name in ("actor", "actor_target", "critic", "critic_target"):
        module = getattr(agent, module_name)
        for tensor_name, tensor in sorted(module.state_dict().items()):
            digest.update(module_name.encode("utf-8"))
            digest.update(tensor_name.encode("utf-8"))
            digest.update(tensor.detach().cpu().contiguous().numpy().tobytes())
    return digest.hexdigest()


def _evaluation_decision(success_count: int, episode_count: int) -> tuple[str, str]:
    """Return the preregistered A0 decision band for a ten-episode evaluation."""
    if episode_count <= 0:
        return "unavailable", "obtain_valid_evaluation_episodes"
    success_rate = success_count / episode_count
    if success_rate <= 0.20:
        return "A_0_to_20_percent", "continue_a0_training_to_100_episodes"
    if success_rate <= 0.50:
        return "B_30_to_50_percent", "continue_a0_training_to_100_episodes"
    if success_rate <= 0.80:
        return "C_60_to_80_percent", "candidate_a0_nominal_baseline_then_review_safety"
    return "D_90_to_100_percent", "freeze_a0_nominal_baseline_and_start_a1"


def _advance_periodic_deadline(
    previous_deadline_s: float | None,
    *,
    current_time_s: float,
    period_s: float,
) -> float:
    if previous_deadline_s is None or not math.isfinite(previous_deadline_s):
        return current_time_s + period_s
    deadline = previous_deadline_s + period_s
    if deadline <= current_time_s + 1.0e-9:
        missed = math.floor((current_time_s - deadline) / period_s) + 1
        deadline += missed * period_s
    return deadline


def _episode_report(
    row: dict[str, Any],
    total_episodes: int,
    *,
    label: str = "TRAIN",
) -> str:
    def fmt(value: object, digits: int = 3) -> str:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return "nan"
        if not math.isfinite(number):
            return "nan"
        return f"{number:.{digits}f}"

    return (
        f"[{label} {int(row['episode']):03d}/{total_episodes:03d}] "
        f"R={fmt(row['episode_return'], 2)} "
        f"success={int(bool(row['success']))} "
        f"steps={int(row['episode_length'])} "
        f"reason={row['termination_reason']} | "
        f"actions={int(row['online_action_count'])} "
        f"rl_obs={int(row['rl_active_observation_count'])} "
        f"stale={fmt(row['rl_active_stale_fraction'])} "
        f"lambda_mean={fmt(row.get('lambda_exec_mean'))} "
        f"lambda90_dwell={fmt(row.get('lambda_exec_ge_0p9_max_dwell_s'))} "
        f"explore_segments={int(row.get('exploration_segment_count', 0))} "
        f"lambda_sat={fmt(row.get('lambda_saturation_fraction'))} "
        f"obs_dt={fmt(row.get('observation_sim_dt_mean'))} "
        f"rl_dt={fmt(row.get('rl_step_sim_dt_mean'))} "
        f"q1={fmt(row.get('critic1_loss_mean'))} "
        f"actor={fmt(row.get('actor_loss_mean'))} "
        f"wall={fmt(row.get('episode_wall_time_s'), 1)}s "
        f"rl_rtf={fmt(row.get('rl_active_real_time_factor'), 2)}"
    )


class A0OnlineTrainer:
    def __init__(
        self,
        *,
        output_dir: Path,
        seed: int,
        device: str | None,
        enable_learning: bool = True,
        fixed_action: float | None = None,
        deterministic_evaluation: bool = False,
        policy_checkpoint: Path | None = None,
        normalizer_checkpoint: Path | None = None,
        learning_starts: int | None = None,
        exploration_hold_s: float = 0.0,
    ) -> None:
        import rclpy
        from rclpy.node import Node
        from rosgraph_msgs.msg import Clock as ClockMsg
        from std_msgs.msg import String

        self.rclpy = rclpy
        self.String = String
        self.node: Node = rclpy.create_node("a0_td3_online_trainer")
        self.publisher = self.node.create_publisher(String, "/ca_lsc/a0_rl_action", 10)
        self.subscription = self.node.create_subscription(
            String, "/ca_lsc/a0_observation", self._observation_callback, 10
        )
        self.clock_subscription = self.node.create_subscription(
            ClockMsg, "/clock", self._clock_callback, 10
        )
        self.output_dir = output_dir
        self.requested_episodes = 0
        self.sim_speed_factor = 1.0
        self.enable_learning = enable_learning
        self.fixed_action = fixed_action
        self.deterministic_evaluation = deterministic_evaluation
        self.policy_checkpoint = policy_checkpoint
        self.normalizer_checkpoint = normalizer_checkpoint
        if deterministic_evaluation and enable_learning:
            raise ValueError("deterministic evaluation cannot enable learning")
        if deterministic_evaluation and fixed_action is not None:
            raise ValueError("deterministic evaluation cannot use a fixed action")
        if deterministic_evaluation and (
            policy_checkpoint is None or normalizer_checkpoint is None
        ):
            raise ValueError(
                "deterministic evaluation requires policy and normalizer checkpoints"
            )
        base_config = A0TD3Config()
        resolved_learning_starts = (
            base_config.learning_starts
            if learning_starts is None else int(learning_starts)
        )
        if resolved_learning_starts <= 0:
            raise ValueError("learning_starts must be positive")
        if not math.isfinite(exploration_hold_s) or exploration_hold_s < 0.0:
            raise ValueError(
                "exploration_hold_s must be finite and non-negative"
            )
        self.config = replace(
            base_config,
            learning_starts=resolved_learning_starts,
        )
        self.agent = TD3Agent(self.config, seed=seed, device=device or "cpu")
        self.normalizer = RunningObservationNormalizer(self.config.observation_dim)
        self._loaded_checkpoint_updates = 0
        if policy_checkpoint is not None:
            if not policy_checkpoint.is_file():
                raise ValueError(f"policy checkpoint not found: {policy_checkpoint}")
            payload = self.agent.load(policy_checkpoint)
            self._loaded_checkpoint_updates = int(payload.get("total_updates", -1))
        if normalizer_checkpoint is not None:
            if not normalizer_checkpoint.is_file():
                raise ValueError(
                    f"normalizer checkpoint not found: {normalizer_checkpoint}"
                )
            normalizer_state = json.loads(
                normalizer_checkpoint.read_text(encoding="utf-8")
            )
            self.normalizer.load_state_dict(normalizer_state)
        # The control actor is read-only on the ROS callback thread.  The
        # learner owns ``self.agent`` and periodically publishes a parameter
        # snapshot under a short lock, so gradient work cannot block action
        # serving or mutate a model during inference.
        self.control_actor = copy.deepcopy(self.agent.actor).eval()
        self._control_actor_lock = threading.Lock()
        self._metrics_lock = threading.Lock()
        self._learner_queue: queue.Queue[tuple[int, ReplayBatch] | None] = (
            queue.Queue()
        )
        self._learner_exception: str | None = None
        self._learner_scheduled_count = 0
        self._learner_completed_count = 0
        self._learner_thread: threading.Thread | None = None
        self.replay = ReplayBuffer(
            capacity=self.config.replay_capacity,
            observation_dim=self.config.observation_dim,
            action_dim=self.config.action_dim,
            seed=seed,
        )
        self.rng = np.random.default_rng(seed)
        self.explorer = PiecewiseConstantExplorer(
            self.rng,
            hold_s=exploration_hold_s,
        )
        self.exploration_hold_s = float(exploration_hold_s)
        self.sequence_id = 0
        self.critic_update_count = 0
        self.actor_update_count = 0
        self.nonfinite_observation_count = 0
        self.nonfinite_loss_count = 0
        self.action_publish_count = 0
        self.action_source_counts: Counter[str] = Counter()
        self.replay_write_count = 0
        self.received_observation_count = 0
        self.task_terminal_count = 0
        self.infrastructure_truncation_count = 0
        self.infrastructure_failure_penalty_count = 0
        self.last_losses: tuple[float | None, float | None, float | None] = (
            None, None, None
        )
        self.episode_rows: list[dict[str, Any]] = []
        self._active_episode = 0
        self._episode_return = 0.0
        self._episode_running_reward_return = 0.0
        self._episode_progress_shaping_return = 0.0
        self._episode_terminal_reward_return = 0.0
        self._episode_steps = 0
        self._episode_actions = 0
        self._episode_transition_observations = 0
        self._episode_transition_stale_observations = 0
        self._episode_pre_rl_observations = 0
        self._episode_rl_active_observations = 0
        self._episode_rl_active_stale_observations = 0
        self._episode_success = False
        self._episode_reason = "not_started"
        self._episode_terminal_command_state = ""
        self._episode_terminal_handover_ready = False
        self._episode_terminal_handover_ready_dwell_s = math.nan
        self._episode_terminal_actual_fw_confirmed = False
        self._episode_initial_readiness_available = False
        self._episode_initial_readiness_ok = False
        self._episode_initial_height_gate_ok = False
        self._episode_initial_vz_gate_ok = False
        self._episode_initial_groundspeed_gate_ok = False
        self._episode_initialization_invalid_reason = ""
        self._max_altitude_error = 0.0
        self._max_abs_alpha = 0.0
        self._max_descent_rate = 0.0
        self._max_abs_vertical_speed = 0.0
        self._lambda_smoothness = 0.0
        self._previous_observation: np.ndarray | None = None
        self._previous_action: np.ndarray | None = None
        self._previous_sim_time_s: float | None = None
        self._previous_lambda_exec: float | None = None
        self._next_action_sim_time_s: float | None = None
        self._next_keepalive_sim_time_s: float | None = None
        self._last_rl_observation: np.ndarray | None = None
        self._episode_terminal_seen = False
        self._episode_first_rl_sim_time_s: float | None = None
        self._episode_last_rl_sim_time_s: float | None = None
        self._episode_first_rl_wall_time_s: float | None = None
        self._episode_last_rl_wall_time_s: float | None = None
        self._episode_lambda_sum = 0.0
        self._episode_lambda_count = 0
        self._episode_lambda_max = 0.0
        self._episode_lambda_saturation_count = 0
        self._episode_exploration_segment_start_count = 0
        self._episode_warmup_action_count = 0
        self._episode_action_source_counts: Counter[str] = Counter()
        self._episode_lambda_ge_0p9_started_at: float | None = None
        self._episode_lambda_ge_0p9_max_dwell_s = 0.0
        self._episode_critic1_losses: list[float] = []
        self._episode_critic2_losses: list[float] = []
        self._episode_actor_losses: list[float] = []
        self._episode_observation_sim_dt_values: list[float] = []
        self._episode_observation_wall_dt_values: list[float] = []
        self._episode_rl_step_sim_dt_values: list[float] = []
        self._episode_rl_step_wall_dt_values: list[float] = []
        self._previous_rl_step_wall_time_s: float | None = None
        self._current_sim_time_s: float | None = None
        self._next_fixed_keepalive_sim_time_s: float | None = None
        self.attempt_rows: list[dict[str, Any]] = []
        self._active_attempt = 0
        self._evaluation_agent_digest_before = _agent_parameter_digest(self.agent)
        self._evaluation_normalizer_state_before = copy.deepcopy(
            self.normalizer.state_dict()
        )
        if self.enable_learning:
            self._learner_thread = threading.Thread(
                target=self._learner_loop,
                name="a0_td3_learner",
                daemon=True,
            )
            self._learner_thread.start()

    def begin_episode(self, episode: int, *, attempt: int | None = None) -> None:
        self._active_episode = episode
        self._active_attempt = int(attempt if attempt is not None else episode)
        self._episode_return = 0.0
        self._episode_running_reward_return = 0.0
        self._episode_progress_shaping_return = 0.0
        self._episode_terminal_reward_return = 0.0
        self._episode_steps = 0
        self._episode_actions = 0
        self._episode_transition_observations = 0
        self._episode_transition_stale_observations = 0
        self._episode_pre_rl_observations = 0
        self._episode_rl_active_observations = 0
        self._episode_rl_active_stale_observations = 0
        self._episode_success = False
        self._episode_reason = "running"
        self._episode_terminal_command_state = ""
        self._episode_terminal_handover_ready = False
        self._episode_terminal_handover_ready_dwell_s = math.nan
        self._episode_terminal_actual_fw_confirmed = False
        self._episode_initial_readiness_available = False
        self._episode_initial_readiness_ok = False
        self._episode_initial_height_gate_ok = False
        self._episode_initial_vz_gate_ok = False
        self._episode_initial_groundspeed_gate_ok = False
        self._episode_initialization_invalid_reason = ""
        self._max_altitude_error = 0.0
        self._max_abs_alpha = 0.0
        self._max_descent_rate = 0.0
        self._max_abs_vertical_speed = 0.0
        self._lambda_smoothness = 0.0
        self._previous_observation = None
        self._previous_action = None
        self._previous_sim_time_s = None
        self._previous_lambda_exec = None
        self._next_action_sim_time_s = None
        self._next_keepalive_sim_time_s = None
        self._last_rl_observation = None
        self._episode_terminal_seen = False
        self._episode_first_rl_sim_time_s = None
        self._episode_last_rl_sim_time_s = None
        self._episode_first_rl_wall_time_s = None
        self._episode_last_rl_wall_time_s = None
        self._episode_lambda_sum = 0.0
        self._episode_lambda_count = 0
        self._episode_lambda_max = 0.0
        self._episode_lambda_saturation_count = 0
        self.explorer.reset()
        self._episode_exploration_segment_start_count = self.explorer.segment_count
        self._episode_warmup_action_count = 0
        self._episode_action_source_counts = Counter()
        self._episode_lambda_ge_0p9_started_at = None
        self._episode_lambda_ge_0p9_max_dwell_s = 0.0
        self._episode_critic1_losses = []
        self._episode_critic2_losses = []
        self._episode_actor_losses = []
        self._episode_observation_sim_dt_values = []
        self._episode_observation_wall_dt_values = []
        self._episode_rl_step_sim_dt_values = []
        self._episode_rl_step_wall_dt_values = []
        self._previous_rl_step_wall_time_s = None
        # Every SITL launch creates a new Gazebo clock epoch.  Keeping the
        # previous episode's time suppresses the pre-RL keepalive until the
        # new clock catches up, which can leave PX4 without any A0 command.
        self._current_sim_time_s = None
        self._next_fixed_keepalive_sim_time_s = None

    def _clock_callback(self, message: Any) -> None:
        sec = float(getattr(message.clock, "sec", 0))
        nanosec = float(getattr(message.clock, "nanosec", 0))
        sim_time_s = sec + nanosec * 1.0e-9
        previous = self._current_sim_time_s
        if previous is not None and sim_time_s < previous - 1.0e-6:
            # A rollback means a fresh simulator epoch.  Drop all deadlines
            # and partial transition state from the old epoch.
            self._next_action_sim_time_s = None
            self._next_keepalive_sim_time_s = None
            self._next_fixed_keepalive_sim_time_s = None
            self._previous_observation = None
            self._previous_action = None
            self._previous_sim_time_s = None
            self._previous_lambda_exec = None
            self._last_rl_observation = None
            self._previous_rl_step_wall_time_s = None
        self._current_sim_time_s = sim_time_s

    def publish_fixed_action_keepalive(self) -> None:
        if self._current_sim_time_s is None or self._episode_terminal_seen:
            return
        forced_action: float | None = None
        if self.fixed_action is None:
            if self._active_episode <= 0 or self._last_rl_observation is not None:
                return
            forced_action = -1.0
        sim_time_s = self._current_sim_time_s
        if (
            self._next_fixed_keepalive_sim_time_s is not None
            and sim_time_s < self._next_fixed_keepalive_sim_time_s - 1.0e-9
        ):
            return
        self._publish_action(
            np.zeros(self.config.observation_dim, dtype=np.float32),
            sim_time_s,
            count_episode=False,
            forced_action=forced_action,
        )
        self._next_fixed_keepalive_sim_time_s = sim_time_s + 0.1

    def warm_up_learning_compute(self) -> None:
        if not self.enable_learning:
            return
        try:
            import torch

            # Warm CUDA/autograd/Adam kernels on a disposable agent.  Calling
            # step() on the real optimizer, even with zero gradients, advances
            # Adam's internal step counter and changes the training protocol.
            cpu_rng_state = torch.random.get_rng_state()
            cuda_rng_states = (
                torch.cuda.get_rng_state_all()
                if self.agent.device.type == "cuda" else None
            )
            scratch = copy.deepcopy(self.agent)
            batch_size = max(2, min(8, self.config.batch_size))
            zeros_obs = np.zeros(
                (batch_size, self.config.observation_dim), dtype=np.float32
            )
            zeros_action = np.zeros(
                (batch_size, self.config.action_dim), dtype=np.float32
            )
            zeros_scalar = np.zeros((batch_size, 1), dtype=np.float32)
            scratch.train_step(ReplayBatch(
                observations=zeros_obs,
                actions=zeros_action,
                rewards=zeros_scalar,
                next_observations=zeros_obs.copy(),
                terminated=zeros_scalar.copy(),
                truncated=zeros_scalar.copy(),
            ))
            if self.agent.device.type == "cuda":
                torch.cuda.synchronize(self.agent.device)
            del scratch
            torch.random.set_rng_state(cpu_rng_state)
            if cuda_rng_states is not None:
                torch.cuda.set_rng_state_all(cuda_rng_states)
        except Exception as exc:  # pragma: no cover - defensive ROS smoke logging
            self.node.get_logger().warning(
                f"A0 TD3 compute warm-up skipped: {exc}"
            )

    def warm_up_control_inference(self) -> None:
        """Prime frozen/control Actor inference without changing any state."""
        try:
            import torch

            observation = np.zeros(self.config.observation_dim, dtype=np.float32)
            normalized = self.normalizer.normalize(observation)
            with self._control_actor_lock, torch.no_grad():
                tensor = torch.as_tensor(
                    normalized[None, :],
                    dtype=torch.float32,
                    device=self.agent.device,
                )
                self.control_actor(tensor)
            if self.agent.device.type == "cuda":
                torch.cuda.synchronize(self.agent.device)
        except Exception as exc:  # pragma: no cover - defensive ROS logging
            self.node.get_logger().warning(
                f"A0 control Actor inference warm-up skipped: {exc}"
            )

    def _select_control_action(
        self,
        observation: np.ndarray,
        sim_time_s: float,
    ) -> np.ndarray:
        import torch

        normalized = self.normalizer.normalize(observation)
        with self._control_actor_lock, torch.no_grad():
            tensor = torch.as_tensor(
                normalized[None, :],
                dtype=torch.float32,
                device=self.agent.device,
            )
            action = self.control_actor(tensor).cpu().numpy()[0]
        if not self.deterministic_evaluation:
            action = action + self.explorer.gaussian_noise(
                sim_time_s=sim_time_s,
                shape=action.shape,
                standard_deviation=self.config.exploration_noise,
            )
        return np.clip(action, -1.0, 1.0).astype(np.float32)

    def _learner_loop(self) -> None:
        while True:
            task = self._learner_queue.get()
            try:
                if task is None:
                    return
                episode, batch = task
                losses = self.agent.train_step(batch)
                if losses.actor_loss is not None:
                    with self._control_actor_lock:
                        self.control_actor.load_state_dict(
                            self.agent.actor.state_dict()
                        )
                        self.control_actor.eval()
                values = [
                    losses.critic1_loss,
                    losses.critic2_loss,
                    losses.actor_loss
                    if losses.actor_loss is not None else 0.0,
                ]
                with self._metrics_lock:
                    self.critic_update_count += 1
                    if losses.actor_loss is not None:
                        self.actor_update_count += 1
                    self._learner_completed_count += 1
                    if not all(math.isfinite(float(value)) for value in values):
                        self.nonfinite_loss_count += 1
                    self.last_losses = (
                        losses.critic1_loss,
                        losses.critic2_loss,
                        losses.actor_loss,
                    )
                    if episode == self._active_episode:
                        self._episode_critic1_losses.append(
                            float(losses.critic1_loss)
                        )
                        self._episode_critic2_losses.append(
                            float(losses.critic2_loss)
                        )
                        if losses.actor_loss is not None:
                            self._episode_actor_losses.append(
                                float(losses.actor_loss)
                            )
            except Exception as exc:  # pragma: no cover - runtime safeguard
                with self._metrics_lock:
                    self._learner_exception = repr(exc)
                    self.nonfinite_loss_count += 1
            finally:
                self._learner_queue.task_done()

    def wait_for_learning_idle(self, timeout_s: float = 30.0) -> bool:
        if not self.enable_learning:
            return True
        deadline = time.monotonic() + timeout_s
        while self._learner_queue.unfinished_tasks:
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.01)
        return True

    def finish_episode(
        self,
        *,
        launch_status: int,
        timed_out: bool,
        wall_time_s: float,
    ) -> dict[str, Any]:
        if timed_out and not self._episode_terminal_seen:
            self._episode_reason = "runner_wall_timeout"
        elif launch_status != 0 and not self._episode_terminal_seen:
            self._episode_reason = "launch_process_error"
        elif not self._episode_terminal_seen:
            self._episode_reason = "terminal_not_observed"
        infrastructure_failure_reason = _attempt_infrastructure_failure(
            terminal_seen=self._episode_terminal_seen,
            reason=self._episode_reason,
            rl_active_observation_count=self._episode_rl_active_observations,
            launch_status=launch_status,
            timed_out=timed_out,
            initialization_invalid_reason=(
                self._episode_initialization_invalid_reason
            ),
        )
        valid_episode = not infrastructure_failure_reason
        action_step_ratio = (
            self._episode_actions / self._episode_steps
            if self._episode_steps > 0 else 0.0
        )
        transition_stale_fraction = (
            self._episode_transition_stale_observations
            / self._episode_transition_observations
            if self._episode_transition_observations > 0 else 0.0
        )
        rl_active_stale_fraction = (
            self._episode_rl_active_stale_observations
            / self._episode_rl_active_observations
            if self._episode_rl_active_observations > 0 else 0.0
        )
        rl_active_sim_time_s = (
            self._episode_last_rl_sim_time_s - self._episode_first_rl_sim_time_s
            if (
                self._episode_first_rl_sim_time_s is not None
                and self._episode_last_rl_sim_time_s is not None
            ) else math.nan
        )
        rl_active_wall_time_s = (
            self._episode_last_rl_wall_time_s - self._episode_first_rl_wall_time_s
            if (
                self._episode_first_rl_wall_time_s is not None
                and self._episode_last_rl_wall_time_s is not None
            ) else math.nan
        )
        rl_active_real_time_factor = (
            rl_active_sim_time_s / rl_active_wall_time_s
            if (
                math.isfinite(rl_active_sim_time_s)
                and math.isfinite(rl_active_wall_time_s)
                and rl_active_wall_time_s > 0.0
            ) else math.nan
        )
        lambda_exec_mean = (
            self._episode_lambda_sum / self._episode_lambda_count
            if self._episode_lambda_count > 0 else math.nan
        )
        exploration_segment_count = (
            self.explorer.segment_count
            - self._episode_exploration_segment_start_count
        )
        lambda_saturation_fraction = (
            self._episode_lambda_saturation_count / self._episode_lambda_count
            if self._episode_lambda_count > 0 else 0.0
        )
        row = {
            "episode": self._active_episode,
            "attempt": self._active_attempt,
            "valid_episode": valid_episode,
            "infrastructure_failure_reason": infrastructure_failure_reason,
            "terminal_semantics": (
                "infrastructure_invalid"
                if infrastructure_failure_reason else "task_terminal"
            ),
            "episode_return": self._episode_return,
            "running_reward_return": self._episode_running_reward_return,
            "progress_shaping_return": self._episode_progress_shaping_return,
            "terminal_reward_return": self._episode_terminal_reward_return,
            "episode_length": self._episode_steps,
            "online_action_count": self._episode_actions,
            "action_step_ratio": action_step_ratio,
            "transition_observation_count": self._episode_transition_observations,
            "transition_stale_count": self._episode_transition_stale_observations,
            "transition_stale_fraction": transition_stale_fraction,
            "pre_rl_observation_count": self._episode_pre_rl_observations,
            "rl_active_observation_count": self._episode_rl_active_observations,
            "rl_active_stale_count": self._episode_rl_active_stale_observations,
            "rl_active_stale_fraction": rl_active_stale_fraction,
            "success": self._episode_success,
            "terminal_seen": self._episode_terminal_seen,
            "termination_reason": self._episode_reason,
            "terminal_command_state": self._episode_terminal_command_state,
            "terminal_handover_ready": self._episode_terminal_handover_ready,
            "terminal_handover_ready_dwell_s": (
                self._episode_terminal_handover_ready_dwell_s
            ),
            "terminal_actual_fw_confirmed": (
                self._episode_terminal_actual_fw_confirmed
            ),
            "initial_readiness_available": (
                self._episode_initial_readiness_available
            ),
            "initial_readiness_ok": self._episode_initial_readiness_ok,
            "initial_height_gate_ok": self._episode_initial_height_gate_ok,
            "initial_vz_gate_ok": self._episode_initial_vz_gate_ok,
            "initial_groundspeed_gate_ok": (
                self._episode_initial_groundspeed_gate_ok
            ),
            "initialization_invalid_reason": (
                self._episode_initialization_invalid_reason
            ),
            "launch_status": launch_status,
            "timed_out": timed_out,
            "episode_wall_time_s": wall_time_s,
            "rl_active_sim_time_s": rl_active_sim_time_s,
            "rl_active_wall_time_s": rl_active_wall_time_s,
            "rl_active_real_time_factor": rl_active_real_time_factor,
            "observation_sim_dt_mean": _mean_or_none(
                self._episode_observation_sim_dt_values
            ),
            "observation_sim_dt_p95": _p95_or_none(
                self._episode_observation_sim_dt_values
            ),
            "observation_wall_dt_mean": _mean_or_none(
                self._episode_observation_wall_dt_values
            ),
            "observation_wall_dt_p95": _p95_or_none(
                self._episode_observation_wall_dt_values
            ),
            "rl_step_sim_dt_mean": _mean_or_none(
                self._episode_rl_step_sim_dt_values
            ),
            "rl_step_sim_dt_p95": _p95_or_none(
                self._episode_rl_step_sim_dt_values
            ),
            "rl_step_wall_dt_mean": _mean_or_none(
                self._episode_rl_step_wall_dt_values
            ),
            "rl_step_wall_dt_p95": _p95_or_none(
                self._episode_rl_step_wall_dt_values
            ),
            "max_altitude_error": self._max_altitude_error,
            "max_abs_alpha": self._max_abs_alpha,
            "max_descent_rate": self._max_descent_rate,
            "max_abs_vertical_speed": self._max_abs_vertical_speed,
            "lambda_smoothness": self._lambda_smoothness,
            "lambda_exec_mean": lambda_exec_mean,
            "lambda_exec_max": self._episode_lambda_max,
            "lambda_saturation_fraction": lambda_saturation_fraction,
            "lambda_exec_ge_0p9_max_dwell_s": (
                self._episode_lambda_ge_0p9_max_dwell_s
            ),
            "exploration_segment_count": exploration_segment_count,
            "warmup_action_count": self._episode_warmup_action_count,
            "actor_deterministic_action_count": (
                self._episode_action_source_counts[
                    ACTION_SOURCE_ACTOR_DETERMINISTIC
                ]
            ),
            "warmup_uniform_action_count": (
                self._episode_action_source_counts[ACTION_SOURCE_WARMUP_UNIFORM]
            ),
            "persistent_exploration_action_count": (
                self._episode_action_source_counts[
                    ACTION_SOURCE_PERSISTENT_EXPLORATION
                ]
            ),
            "fixed_action_count": (
                self._episode_action_source_counts[ACTION_SOURCE_FIXED]
            ),
            "forced_action_count": (
                self._episode_action_source_counts[ACTION_SOURCE_FORCED]
            ),
            "action_source_count": sum(
                self._episode_action_source_counts.values()
            ),
            "critic1_loss": self.last_losses[0],
            "critic2_loss": self.last_losses[1],
            "actor_loss": self.last_losses[2],
            "critic1_loss_mean": _mean_or_none(self._episode_critic1_losses),
            "critic1_loss_p95": _p95_or_none(self._episode_critic1_losses),
            "critic2_loss_mean": _mean_or_none(self._episode_critic2_losses),
            "critic2_loss_p95": _p95_or_none(self._episode_critic2_losses),
            "actor_loss_mean": _mean_or_none(self._episode_actor_losses),
            "actor_loss_p95": _p95_or_none(self._episode_actor_losses),
            "replay_size": self.replay.size,
        }
        return row

    def _publish_action(
        self,
        observation: np.ndarray,
        sim_time_s: float,
        *,
        count_episode: bool = True,
        forced_action: float | None = None,
    ) -> np.ndarray:
        if forced_action is not None:
            action = np.asarray([forced_action], dtype=np.float32)
            action_source = (
                ACTION_SOURCE_FORCED
                if count_episode else ACTION_SOURCE_SAFE_KEEPALIVE
            )
        elif self.deterministic_evaluation:
            # Evaluation replay is intentionally empty.  This override must
            # precede the warm-up gate so a frozen checkpoint is evaluated
            # directly instead of silently falling back to uniform actions.
            action = self._select_control_action(observation, sim_time_s)
            action_source = ACTION_SOURCE_ACTOR_DETERMINISTIC
        elif self.fixed_action is not None:
            action = np.asarray([self.fixed_action], dtype=np.float32)
            action_source = ACTION_SOURCE_FIXED
        elif self.replay.size < self.config.learning_starts:
            action = self.explorer.uniform_action(
                sim_time_s=sim_time_s,
                shape=(self.config.action_dim,),
            )
            action_source = ACTION_SOURCE_WARMUP_UNIFORM
            if count_episode:
                self._episode_warmup_action_count += 1
        else:
            action = self._select_control_action(observation, sim_time_s)
            action_source = ACTION_SOURCE_PERSISTENT_EXPLORATION
        lambda_d = action_to_lambda(float(action[0]))
        message = self.String()
        message.data = json.dumps({
            "sequence_id": self.sequence_id,
            "timestamp_sim_s": sim_time_s,
            "action": float(action[0]),
            "lambda_d": lambda_d,
            "action_source": action_source,
        })
        self.publisher.publish(message)
        self.sequence_id += 1
        self.action_publish_count += 1
        self.action_source_counts[action_source] += 1
        if count_episode:
            self._episode_actions += 1
            self._episode_action_source_counts[action_source] += 1
        return action

    def _publish_safe_keepalive(
        self,
        observation: np.ndarray,
        sim_time_s: float,
    ) -> None:
        if (
            self._next_keepalive_sim_time_s is None
            or sim_time_s >= self._next_keepalive_sim_time_s - 1.0e-9
        ):
            # A deterministic frozen policy can be evaluated before the
            # transition starts because HOLD_MC observations use the same A0
            # contract.  Pre-publishing it avoids a DDS/inference bootstrap
            # interval in which the aircraft would otherwise accelerate under
            # lambda=0 before its first evaluated Actor action arrives.
            forced_action = None if self.deterministic_evaluation else -1.0
            self._publish_action(
                observation,
                sim_time_s,
                count_episode=False,
                forced_action=forced_action,
            )
            self._next_keepalive_sim_time_s = sim_time_s + 0.1

    def _record_transition(
        self,
        *,
        observation: np.ndarray,
        sim_time_s: float,
        terminated: bool,
        truncated: bool,
        reason: str,
    ) -> None:
        if self._previous_observation is None or self._previous_action is None:
            return
        dt_s = 0.1
        if self._previous_sim_time_s is not None and math.isfinite(sim_time_s):
            dt_s = max(1.0e-3, min(0.2, sim_time_s - self._previous_sim_time_s))
        if math.isfinite(dt_s):
            self._episode_rl_step_sim_dt_values.append(dt_s)
        lambda_delta = float(observation[7] - self._previous_observation[7])
        success = terminated and reason == "success"
        failure = terminated and not success
        unsafe_failure = failure and reason != "forward_transition_timeout"
        if failure and reason in INFRASTRUCTURE_TERMINATION_REASONS:
            self.infrastructure_failure_penalty_count += 1
        reward_terms = common_reward_terms(
            dt_s=dt_s,
            previous_airspeed_mps=float(self._previous_observation[0]),
            airspeed_mps=float(observation[0]),
            previous_lambda_executed=float(self._previous_observation[7]),
            lambda_executed=float(observation[7]),
            altitude_error_m=float(observation[3]),
            pitch_rate_rps=float(observation[6]),
            alpha_rad=float(observation[1]),
            lambda_delta=lambda_delta,
            success=success,
            failure=failure,
            unsafe_failure=unsafe_failure,
        )
        reward = reward_terms.total
        initialization_invalid = bool(
            getattr(self, "_episode_initialization_invalid_reason", "")
        )
        if not self.deterministic_evaluation and not initialization_invalid:
            self.replay.add(
                self._previous_observation,
                self._previous_action,
                reward,
                observation,
                terminated=terminated,
                truncated=truncated,
            )
            self.replay_write_count += 1
        self._episode_return += reward
        self._episode_running_reward_return += reward_terms.running
        self._episode_progress_shaping_return += reward_terms.progress_shaping
        self._episode_terminal_reward_return += reward_terms.terminal
        self._episode_steps += 1
        self._max_altitude_error = max(self._max_altitude_error, abs(float(observation[3])))
        self._max_abs_alpha = max(self._max_abs_alpha, abs(float(observation[1])))
        self._max_descent_rate = max(self._max_descent_rate, max(0.0, -float(observation[4])))
        self._max_abs_vertical_speed = max(
            self._max_abs_vertical_speed,
            abs(float(observation[4])),
        )
        if self._previous_lambda_exec is not None:
            self._lambda_smoothness += (float(observation[7]) - self._previous_lambda_exec) ** 2
        self._previous_lambda_exec = float(observation[7])
        now_wall = time.monotonic()
        if self._previous_rl_step_wall_time_s is not None:
            self._episode_rl_step_wall_dt_values.append(
                now_wall - self._previous_rl_step_wall_time_s
            )
        self._previous_rl_step_wall_time_s = now_wall
        if self._episode_first_rl_sim_time_s is None:
            self._episode_first_rl_sim_time_s = (
                self._previous_sim_time_s
                if self._previous_sim_time_s is not None else sim_time_s
            )
            self._episode_first_rl_wall_time_s = now_wall
        self._episode_last_rl_sim_time_s = sim_time_s
        self._episode_last_rl_wall_time_s = now_wall
        lambda_exec = float(observation[7])
        self._episode_lambda_sum += lambda_exec
        self._episode_lambda_count += 1
        self._episode_lambda_max = max(self._episode_lambda_max, lambda_exec)
        if lambda_exec <= 1.0e-3 or lambda_exec >= 1.0 - 1.0e-3:
            self._episode_lambda_saturation_count += 1
        if lambda_exec >= 0.9:
            if self._episode_lambda_ge_0p9_started_at is None:
                self._episode_lambda_ge_0p9_started_at = sim_time_s
            self._episode_lambda_ge_0p9_max_dwell_s = max(
                self._episode_lambda_ge_0p9_max_dwell_s,
                max(0.0, sim_time_s - self._episode_lambda_ge_0p9_started_at),
            )
        else:
            self._episode_lambda_ge_0p9_started_at = None

    def _maybe_train_one_step(self) -> None:
        """Queue one update without blocking the ROS/action-serving thread."""
        if (
            self.enable_learning
            and not getattr(self, "_episode_initialization_invalid_reason", "")
            and self.replay.size >= max(
                self.config.batch_size,
                self.config.learning_starts,
            )
        ):
            batch = _normalized_batch(
                self.replay.sample(self.config.batch_size),
                self.normalizer,
            )
            self._learner_queue.put((self._active_episode, batch))
            self._learner_scheduled_count += 1

    def _observation_callback(self, message: Any) -> None:
        try:
            payload = json.loads(message.data)
            observation = np.asarray(payload["observation"], dtype=np.float32)
            sim_time_s = float(payload["sim_time_s"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.nonfinite_observation_count += 1
            return
        if observation.shape != (self.config.observation_dim,) or not np.all(np.isfinite(observation)):
            self.nonfinite_observation_count += 1
            return
        self.received_observation_count += 1

        command_state = str(payload.get("command_state", "")).split("|", 1)[0]
        in_transition = command_state == "TRANSITION_FW"
        terminated = bool(payload.get("terminated", False))
        truncated = bool(payload.get("truncated", False))
        reason = str(payload.get("termination_reason", "running"))
        stale_raw = payload.get("a0_rl_action_stale", 0)
        try:
            action_stale = bool(int(float(stale_raw)))
        except (TypeError, ValueError):
            action_stale = False
        control_phase = str(payload.get("a0_rl_control_phase", "")).strip()
        if not control_phase and in_transition:
            control_phase = "pre_rl" if action_stale else "rl_active"
        rl_active = in_transition and control_phase == "rl_active" and not action_stale
        obs_sim_dt = _finite(payload.get("a0_observation_sim_dt_s"))
        obs_wall_dt = _finite(payload.get("a0_observation_wall_dt_s"))
        if in_transition and math.isfinite(obs_sim_dt) and obs_sim_dt > 0.0:
            self._episode_observation_sim_dt_values.append(obs_sim_dt)
        if in_transition and math.isfinite(obs_wall_dt) and obs_wall_dt > 0.0:
            self._episode_observation_wall_dt_values.append(obs_wall_dt)

        if terminated or truncated:
            # The recorder deliberately repeats terminal payloads briefly so
            # DDS can deliver them before process exit.  The first payload is
            # authoritative.  Once action publication stops, a later repeat
            # may observe an old action age; it must never overwrite the
            # original physical terminal with a synthetic action timeout.
            if self._episode_terminal_seen:
                return
            self._episode_terminal_seen = True
            self._episode_success = reason == "success"
            self._episode_reason = reason
            self._episode_terminal_command_state = command_state
            self._episode_terminal_handover_ready = _payload_bool(
                payload.get("a0_rl_handover_ready")
            )
            self._episode_terminal_handover_ready_dwell_s = _finite(
                payload.get("a0_rl_handover_ready_dwell_s")
            )
            self._episode_terminal_actual_fw_confirmed = _payload_bool(
                payload.get("a0_rl_actual_fw_confirmed")
            )
            if self._previous_observation is not None and self._previous_action is not None:
                replay_terminated, replay_truncated, _ = _terminal_semantics(
                    reason=reason,
                    terminated=terminated,
                    truncated=truncated,
                )
                if replay_terminated:
                    self.task_terminal_count += 1
                if replay_truncated:
                    self.infrastructure_truncation_count += 1
                if (
                    not self.deterministic_evaluation
                    and not self._episode_initialization_invalid_reason
                ):
                    self.normalizer.update(observation)
                self._last_rl_observation = observation
                self._record_transition(
                    observation=observation,
                    sim_time_s=sim_time_s,
                    terminated=replay_terminated,
                    truncated=replay_truncated,
                    reason=reason,
                )
                self._maybe_train_one_step()
                self._previous_observation = None
                self._previous_action = None
                self._previous_sim_time_s = None
                self._previous_lambda_exec = None
                self._next_action_sim_time_s = None
                return
            return

        if not in_transition:
            self._publish_safe_keepalive(observation, sim_time_s)
            self._previous_observation = None
            self._previous_action = None
            self._previous_sim_time_s = None
            self._previous_lambda_exec = None
            self._next_action_sim_time_s = None
            return

        self._episode_transition_observations += 1
        if action_stale:
            self._episode_transition_stale_observations += 1

        if not rl_active:
            self._episode_pre_rl_observations += 1
            self._publish_safe_keepalive(observation, sim_time_s)
            self._previous_observation = None
            self._previous_action = None
            self._previous_sim_time_s = None
            self._previous_lambda_exec = None
            self._next_action_sim_time_s = None
            return

        self._episode_rl_active_observations += 1
        if action_stale:
            self._episode_rl_active_stale_observations += 1

        if self._episode_rl_active_observations == 1:
            # Admission uses the command-node snapshot latched exactly at the
            # HOLD_MC -> TRANSITION_FW edge.  The live gates in this callback
            # describe post-entry dynamics and may already be false by the
            # time an x4 DDS subscriber receives its first RL-active sample.
            (
                self._episode_initial_readiness_available,
                self._episode_initial_height_gate_ok,
                self._episode_initial_vz_gate_ok,
                self._episode_initial_groundspeed_gate_ok,
                self._episode_initial_readiness_ok,
                self._episode_initialization_invalid_reason,
            ) = _latched_initial_readiness(payload)

        if self._episode_initialization_invalid_reason:
            # Keep the aircraft at the safe MC endpoint during the short
            # terminal-delivery grace period.  Schema 34 aborts a genuinely
            # invalid transition-entry snapshot immediately upstream.
            # No observation, action, reward, normalizer sample, or gradient
            # from this attempt may enter the learning process.
            self._publish_action(
                observation,
                sim_time_s,
                count_episode=False,
                forced_action=-1.0,
            )
            self._previous_observation = None
            self._previous_action = None
            self._previous_sim_time_s = None
            self._previous_lambda_exec = None
            self._next_action_sim_time_s = None
            return

        if not self.deterministic_evaluation:
            self.normalizer.update(observation)
        self._last_rl_observation = observation
        if self._previous_observation is None or self._previous_action is None:
            action = self._publish_action(observation, sim_time_s)
            self._previous_observation = observation
            self._previous_action = action
            self._previous_sim_time_s = sim_time_s
            self._previous_lambda_exec = float(observation[7])
            self._next_action_sim_time_s = _advance_periodic_deadline(
                None,
                current_time_s=sim_time_s,
                period_s=0.1,
            )
            return

        should_step = (
            terminated
            or truncated
            or self._next_action_sim_time_s is None
            or sim_time_s >= self._next_action_sim_time_s - 1.0e-9
        )
        if not should_step:
            return

        self._record_transition(
            observation=observation,
            sim_time_s=sim_time_s,
            terminated=terminated,
            truncated=truncated,
            reason=reason,
        )
        if terminated or truncated:
            self._previous_observation = None
            self._previous_action = None
            self._previous_sim_time_s = None
            self._next_action_sim_time_s = None
            return

        action = self._publish_action(observation, sim_time_s)
        self._previous_observation = observation
        self._previous_action = action
        self._previous_sim_time_s = sim_time_s
        self._next_action_sim_time_s = _advance_periodic_deadline(
            self._next_action_sim_time_s,
            current_time_s=sim_time_s,
            period_s=0.1,
        )
        self._maybe_train_one_step()

    def write_evaluation_outputs(self) -> dict[str, Any]:
        """Write a frozen-policy evaluation without creating a new checkpoint."""
        metrics_csv = self.output_dir / "evaluation_metrics.csv"
        attempt_metrics_csv = self.output_dir / "evaluation_attempt_metrics.csv"
        metrics_csv.parent.mkdir(parents=True, exist_ok=True)
        if self.episode_rows:
            with metrics_csv.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=list(self.episode_rows[0].keys()),
                )
                writer.writeheader()
                writer.writerows(self.episode_rows)
        if self.attempt_rows:
            with attempt_metrics_csv.open(
                "w", newline="", encoding="utf-8"
            ) as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=list(self.attempt_rows[0].keys()),
                )
                writer.writeheader()
                writer.writerows(self.attempt_rows)

        agent_digest_after = _agent_parameter_digest(self.agent)
        normalizer_state_after = self.normalizer.state_dict()
        policy_frozen = (
            agent_digest_after == self._evaluation_agent_digest_before
            and self.agent.total_updates == self._loaded_checkpoint_updates
        )
        normalizer_frozen = (
            normalizer_state_after == self._evaluation_normalizer_state_before
        )
        success_rows = [row for row in self.episode_rows if bool(row["success"])]
        success_count = len(success_rows)
        episode_count = len(self.episode_rows)
        success_rate = success_count / episode_count if episode_count else 0.0
        decision_band, recommended_next_step = _evaluation_decision(
            success_count,
            episode_count,
        )

        pass_failures: list[str] = []
        if self.enable_learning:
            pass_failures.append("learning_enabled_during_evaluation")
        if self.fixed_action is not None:
            pass_failures.append("fixed_action_used_instead_of_loaded_policy")
        if self.received_observation_count <= 0:
            pass_failures.append("no_online_observations")
        if self.action_publish_count <= 0:
            pass_failures.append("no_online_actions")
        if self.nonfinite_observation_count:
            pass_failures.append("nonfinite_or_malformed_observation")
        if self.nonfinite_loss_count:
            pass_failures.append("nonfinite_loss")
        if self._learner_exception is not None:
            pass_failures.append("learner_worker_failed")
        if len(self.episode_rows) != self.requested_episodes:
            pass_failures.append("requested_valid_episode_count_not_reached")
        if any(not row["valid_episode"] for row in self.attempt_rows):
            pass_failures.append("infrastructure_invalid_attempt_observed")
        if self.replay.size != 0:
            pass_failures.append("replay_mutated_during_evaluation")
        if self.replay_write_count != 0:
            pass_failures.append("replay_write_during_evaluation")
        if self.critic_update_count or self.actor_update_count:
            pass_failures.append("network_update_during_evaluation")
        if self._learner_scheduled_count or self._learner_completed_count:
            pass_failures.append("learner_work_scheduled_during_evaluation")
        if not policy_frozen:
            pass_failures.append("loaded_policy_changed_during_evaluation")
        if not normalizer_frozen:
            pass_failures.append("normalizer_changed_during_evaluation")
        if any(row["online_action_count"] <= 0 for row in self.episode_rows):
            pass_failures.append("episode_without_online_actions")
        if any(row["episode_length"] <= 0 for row in self.episode_rows):
            pass_failures.append("episode_without_transitions")
        if any(not row["terminal_seen"] for row in self.episode_rows):
            pass_failures.append("episode_terminal_not_observed")
        if any(row["termination_reason"] == "running" for row in self.episode_rows):
            pass_failures.append("episode_terminal_reason_still_running")
        if any(
            row["success"] and not row["terminal_actual_fw_confirmed"]
            for row in self.episode_rows
        ):
            pass_failures.append("success_without_actual_fw_confirmation")
        if any(row["rl_active_observation_count"] <= 0 for row in self.episode_rows):
            pass_failures.append("episode_without_rl_active_observations")
        if any(row["action_step_ratio"] < 0.9 for row in self.episode_rows):
            pass_failures.append("online_action_not_aligned_with_evaluation_steps")
        if any(
            row["actor_deterministic_action_count"] != row["episode_length"]
            for row in self.episode_rows
        ):
            pass_failures.append("not_all_evaluation_steps_used_deterministic_actor")
        if any(
            row["warmup_uniform_action_count"] != 0
            or row["warmup_action_count"] != 0
            for row in self.episode_rows
        ):
            pass_failures.append("warmup_action_used_during_evaluation")
        if any(
            row["persistent_exploration_action_count"] != 0
            or row["exploration_segment_count"] != 0
            for row in self.episode_rows
        ):
            pass_failures.append("exploration_action_used_during_evaluation")
        if any(
            row["fixed_action_count"] != 0 or row["forced_action_count"] != 0
            for row in self.episode_rows
        ):
            pass_failures.append("non_actor_control_action_used_during_evaluation")
        if any(
            row["action_source_count"] != row["online_action_count"]
            for row in self.episode_rows
        ):
            pass_failures.append("evaluation_action_source_count_mismatch")
        if any(row["rl_active_stale_fraction"] > 0.01 for row in self.episode_rows):
            pass_failures.append("rl_active_action_stale_fraction_high")

        termination_counts = dict(sorted(Counter(
            str(row["termination_reason"]) for row in self.episode_rows
        ).items()))
        metric_summary = {
            key: _metric_summary(self.episode_rows, key)
            for key in (
                "episode_return",
                "running_reward_return",
                "progress_shaping_return",
                "terminal_reward_return",
                "rl_active_sim_time_s",
                "max_altitude_error",
                "max_abs_alpha",
                "max_abs_vertical_speed",
                "lambda_exec_mean",
                "lambda_exec_max",
                "lambda_smoothness",
                "lambda_saturation_fraction",
            )
        }
        success_metric_summary = {
            key: _metric_summary(success_rows, key)
            for key in (
                "episode_return",
                "running_reward_return",
                "progress_shaping_return",
                "terminal_reward_return",
                "rl_active_sim_time_s",
                "max_altitude_error",
                "max_abs_alpha",
                "max_abs_vertical_speed",
                "lambda_exec_mean",
                "lambda_exec_max",
                "lambda_smoothness",
                "lambda_saturation_fraction",
            )
        }
        summary = {
            "stage": "A0 deterministic nominal evaluation",
            "a0_deterministic_evaluation_protocol_pass": not pass_failures,
            "pass_failures": pass_failures,
            "episodes": episode_count,
            "requested_valid_episodes": self.requested_episodes,
            "attempts": len(self.attempt_rows),
            "invalid_attempts": sum(
                not bool(row["valid_episode"]) for row in self.attempt_rows
            ),
            "success_count": success_count,
            "success_rate": success_rate,
            "termination_counts": termination_counts,
            "decision_band": decision_band,
            "recommended_next_step": recommended_next_step,
            "deterministic_policy": True,
            "exploration_enabled": False,
            "learning_enabled": self.enable_learning,
            "normalizer_updates_enabled": False,
            "replay_recording_enabled": False,
            "sim_speed_factor": self.sim_speed_factor,
            "observation_dim": self.config.observation_dim,
            "action_dim": self.config.action_dim,
            "energy_reward_weight": self.config.energy_reward_weight,
            "reward_version": COMMON_REWARD_VERSION,
            "reward_weights": asdict(DebugRewardWeights()),
            "source_policy_checkpoint": str(self.policy_checkpoint),
            "source_normalizer_checkpoint": str(self.normalizer_checkpoint),
            "loaded_checkpoint_total_updates": self._loaded_checkpoint_updates,
            "policy_parameter_digest_before": self._evaluation_agent_digest_before,
            "policy_parameter_digest_after": agent_digest_after,
            "policy_frozen": policy_frozen,
            "normalizer_frozen": normalizer_frozen,
            "replay_size": self.replay.size,
            "replay_write_count": self.replay_write_count,
            "critic_update_count": self.critic_update_count,
            "actor_update_count": self.actor_update_count,
            "action_source_counts": dict(sorted(self.action_source_counts.items())),
            "episode_action_source_counts": {
                source: sum(
                    int(row[f"{source}_action_count"])
                    for row in self.episode_rows
                )
                for source in EPISODE_ACTION_SOURCES
            },
            "received_observation_count": self.received_observation_count,
            "action_publish_count": self.action_publish_count,
            "metric_summary": metric_summary,
            "success_metric_summary": success_metric_summary,
            "evaluation_metrics_csv": str(metrics_csv),
            "evaluation_attempt_metrics_csv": str(attempt_metrics_csv),
            "episode_rows": self.episode_rows,
            "attempt_rows": self.attempt_rows,
            "note": (
                "Frozen-policy nominal evaluation. Success rate is an outcome, "
                "not a protocol pass criterion."
            ),
        }
        summary_path = self.output_dir / "a0_deterministic_evaluation_summary.json"
        summary_path.write_text(
            json.dumps(_json_safe(summary), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(_json_safe(summary), indent=2, sort_keys=True))
        return summary

    def write_outputs(self) -> dict[str, Any]:
        if self.deterministic_evaluation:
            return self.write_evaluation_outputs()
        if not self.wait_for_learning_idle():
            self._learner_exception = "learner_queue_drain_timeout"
        metrics_csv = self.output_dir / "training_metrics.csv"
        attempt_metrics_csv = self.output_dir / "attempt_metrics.csv"
        metrics_csv.parent.mkdir(parents=True, exist_ok=True)
        if self.episode_rows:
            with metrics_csv.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(self.episode_rows[0].keys()))
                writer.writeheader()
                writer.writerows(self.episode_rows)
        if self.attempt_rows:
            with attempt_metrics_csv.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=list(self.attempt_rows[0].keys()),
                )
                writer.writeheader()
                writer.writerows(self.attempt_rows)
        checkpoint = self.output_dir / "checkpoints" / "a0_ros_td3_smoke.pt"
        self.agent.save(checkpoint)
        normalizer_path = self.output_dir / "checkpoints" / "a0_ros_normalizer.json"
        normalizer_state = self.normalizer.state_dict()
        normalizer_path.write_text(
            json.dumps(_json_safe(normalizer_state), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        reload_agent = TD3Agent(self.config, seed=1, device=self.agent.device)
        reload_payload = reload_agent.load(checkpoint)
        reload_normalizer = RunningObservationNormalizer(self.config.observation_dim)
        reload_normalizer.load_state_dict(normalizer_state)
        reload_action_max_abs_diff: float | None = None
        if self._last_rl_observation is not None:
            before_action = self.agent.select_action(
                self.normalizer.normalize(self._last_rl_observation),
                explore=False,
            )
            after_action = reload_agent.select_action(
                reload_normalizer.normalize(self._last_rl_observation),
                explore=False,
            )
            reload_action_max_abs_diff = float(
                np.max(np.abs(before_action - after_action))
            )
        pass_failures: list[str] = []
        if self.received_observation_count <= 0:
            pass_failures.append("no_online_observations")
        if self.action_publish_count <= 0:
            pass_failures.append("no_online_actions")
        if self.nonfinite_observation_count:
            pass_failures.append("nonfinite_or_malformed_observation")
        if self.nonfinite_loss_count:
            pass_failures.append("nonfinite_loss")
        if self._learner_exception is not None:
            pass_failures.append("learner_worker_failed")
        if self.infrastructure_failure_penalty_count:
            pass_failures.append("infrastructure_failure_penalty_observed")
        if len(self.episode_rows) != self.requested_episodes:
            pass_failures.append("requested_valid_episode_count_not_reached")
        if any(not row["valid_episode"] for row in self.attempt_rows):
            pass_failures.append("infrastructure_invalid_attempt_observed")
        if self.replay.size < self.config.batch_size:
            pass_failures.append("replay_not_populated")
        if self.enable_learning and self.critic_update_count <= 0:
            pass_failures.append("critic_not_updated")
        if self.enable_learning and self.actor_update_count <= 0:
            pass_failures.append("actor_not_updated")
        if int(reload_payload.get("total_updates", -1)) != self.agent.total_updates:
            pass_failures.append("checkpoint_reload_failed")
        if (
            reload_action_max_abs_diff is None
            or reload_action_max_abs_diff > 1.0e-6
        ):
            pass_failures.append("policy_normalizer_reload_equivalence_failed")
        if any(row["online_action_count"] <= 0 for row in self.episode_rows):
            pass_failures.append("episode_without_online_actions")
        if any(row["episode_length"] <= 0 for row in self.episode_rows):
            pass_failures.append("episode_without_transitions")
        if any(not row["terminal_seen"] for row in self.episode_rows):
            pass_failures.append("episode_terminal_not_observed")
        if any(row["termination_reason"] == "running" for row in self.episode_rows):
            pass_failures.append("episode_terminal_reason_still_running")
        if any(row["termination_reason"] == "a0_rl_action_timeout" for row in self.episode_rows):
            pass_failures.append("a0_rl_action_timeout_observed")
        if any(
            row["success"] and not row["terminal_actual_fw_confirmed"]
            for row in self.episode_rows
        ):
            pass_failures.append("success_without_actual_fw_confirmation")
        if any(row["rl_active_observation_count"] <= 0 for row in self.episode_rows):
            pass_failures.append("episode_without_rl_active_observations")
        if any(row["action_step_ratio"] < 0.9 for row in self.episode_rows):
            pass_failures.append("online_action_not_aligned_with_training_steps")
        if any(row["rl_active_stale_fraction"] > 0.01 for row in self.episode_rows):
            pass_failures.append("rl_active_action_stale_fraction_high")
        if any(
            row["observation_sim_dt_mean"] is None
            or not 0.045 <= float(row["observation_sim_dt_mean"]) <= 0.055
            for row in self.episode_rows
        ):
            pass_failures.append("observation_sim_dt_out_of_bounds")
        if any(
            row["rl_step_sim_dt_mean"] is None
            or not 0.09 <= float(row["rl_step_sim_dt_mean"]) <= 0.112
            for row in self.episode_rows
        ):
            pass_failures.append("rl_step_sim_dt_out_of_bounds")
        summary = {
            "stage": "A0 ROS/PX4 online training smoke",
            "a0_ros_training_smoke_pass": not pass_failures,
            "pass_failures": pass_failures,
            "episodes": len(self.episode_rows),
            "requested_valid_episodes": self.requested_episodes,
            "attempts": len(self.attempt_rows),
            "invalid_attempts": sum(
                not bool(row["valid_episode"]) for row in self.attempt_rows
            ),
            "observation_dim": self.config.observation_dim,
            "action_dim": self.config.action_dim,
            "energy_reward_weight": self.config.energy_reward_weight,
            "reward_version": COMMON_REWARD_VERSION,
            "reward_weights": asdict(DebugRewardWeights()),
            "learning_starts": self.config.learning_starts,
            "exploration_mode": (
                "piecewise_constant"
                if self.exploration_hold_s > 0.0 else "iid"
            ),
            "exploration_hold_s": self.exploration_hold_s,
            "learning_enabled": self.enable_learning,
            "fixed_action": self.fixed_action,
            "sim_speed_factor": self.sim_speed_factor,
            "received_observation_count": self.received_observation_count,
            "action_publish_count": self.action_publish_count,
            "task_terminal_count": self.task_terminal_count,
            "infrastructure_truncation_count": (
                self.infrastructure_truncation_count
            ),
            "infrastructure_failure_penalty_count": (
                self.infrastructure_failure_penalty_count
            ),
            "replay_size": self.replay.size,
            "critic_update_count": self.critic_update_count,
            "actor_update_count": self.actor_update_count,
            "learner_scheduled_count": self._learner_scheduled_count,
            "learner_completed_count": self._learner_completed_count,
            "learner_exception": self._learner_exception,
            "normalizer": self.normalizer.state_dict(),
            "checkpoint": str(checkpoint),
            "normalizer_checkpoint": str(normalizer_path),
            "policy_normalizer_reload_action_max_abs_diff": reload_action_max_abs_diff,
            "training_metrics_csv": str(metrics_csv),
            "attempt_metrics_csv": str(attempt_metrics_csv),
            "episode_rows": self.episode_rows,
            "attempt_rows": self.attempt_rows,
            "termination_counts": dict(sorted(Counter(
                str(row["termination_reason"]) for row in self.episode_rows
            ).items())),
            "reward_component_summary": {
                key: _metric_summary(self.episode_rows, key)
                for key in (
                    "episode_return",
                    "running_reward_return",
                    "progress_shaping_return",
                    "terminal_reward_return",
                )
            },
            "success_discovery_count": sum(
                bool(row["success"]) for row in self.episode_rows
            ),
            "post_warmup_success_count": sum(
                bool(row["success"])
                and int(row["warmup_action_count"]) == 0
                for row in self.episode_rows
            ),
            "lambda_exec_ge_0p9_episode_count": sum(
                float(row["lambda_exec_max"]) >= 0.9
                for row in self.episode_rows
            ),
            "lambda_exec_ge_0p9_dwell_1s_episode_count": sum(
                float(row["lambda_exec_ge_0p9_max_dwell_s"]) >= 1.0
                for row in self.episode_rows
            ),
            "a0_success_discovery_pilot_pass": any(
                bool(row["success"])
                and bool(row["terminal_actual_fw_confirmed"])
                for row in self.episode_rows
            ),
            "note": "Integration smoke only; not a formal A0 performance result.",
        }
        summary_path = self.output_dir / "a0_ros_training_smoke_summary.json"
        summary_path.write_text(
            json.dumps(_json_safe(summary), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(_json_safe(summary), indent=2, sort_keys=True))
        return summary

    def destroy(self) -> None:
        self.wait_for_learning_idle()
        if self._learner_thread is not None:
            self._learner_queue.put(None)
            self._learner_thread.join(timeout=5.0)
        self.node.destroy_node()


def _launch_command(
    project_root: Path,
    run_dir: Path,
    *,
    sim_speed_factor: float,
    condition_prefix: str = "a0_ros_training",
) -> list[str]:
    frozen = _parse_env_file(project_root / "config" / "nominal_low_level.env")
    return [
        "ros2", "launch", "ca_lsc_transition", "nominal_transition.launch.py",
        f"px4_dir:={project_root / 'PX4-Autopilot'}",
        f"px4_build_dir:={project_root / 'PX4-Autopilot' / 'build_ca_make'}",
        f"condition_id:={condition_prefix}_{run_dir.name}",
        f"sim_speed_factor:={sim_speed_factor:g}",
        "schedule_mode:=a0_rl_external",
        "a0_rl_command_timeout:=0.35",
        "require_a0_rl_action_ready:=false",
        "a0_rl_action_ready_timeout:=0.5",
        "a0_rl_handover_lambda:=0.95",
        "a0_rl_handover_dwell:=1.0",
        "va_target:=15.0",
        "va_target_tolerance:=0.5",
        "vt_unload_test_enable:=1",
        "hold_mc_s:=5.0",
        "hold_fw_s:=3.0",
        "forward_distance_m:=500.0",
        "max_horizontal_speed:=18.0",
        "max_vertical_speed:=1.5",
        "fw_min_forward_speed:=15.0",
        f"fw_airspeed_min:={frozen.get('NOMINAL_FW_AIRSPD_MIN', '12.0')}",
        f"fw_airspeed_trim:={frozen.get('NOMINAL_FW_AIRSPD_TRIM', '15.0')}",
        f"fw_airspeed_max:={frozen.get('NOMINAL_FW_AIRSPD_MAX', '20.0')}",
        f"vt_blend_airspeed:={frozen.get('NOMINAL_VT_ARSP_BLEND', '8.0')}",
        f"vt_transition_airspeed:={frozen.get('NOMINAL_VT_ARSP_TRANS', '13.0')}",
        "lambda_attitude_blend_start:=0.1",
        "lambda_attitude_blend_full:=0.9",
        "va_velocity_kp:=0.8",
        "va_velocity_min:=13.0",
        "va_velocity_max:=18.0",
        "va_altitude_kp:=0.35",
        "va_altitude_vertical_speed_kd:=0.5",
        "va_altitude_velocity_limit:=1.5",
        "pusher_airspeed_ff:=0.25",
        "pusher_airspeed_kp:=0.08",
        "pusher_airspeed_ki:=0.03",
        "pusher_throttle_min:=0.0",
        "pusher_throttle_max:=0.45",
        "pusher_handover_below_target:=1.0",
        "pusher_throttle_slew_up:=0.5",
        "pusher_throttle_slew_down:=1.5",
        "va_filter_alpha:=0.35",
        "va_runaway_margin:=3.0",
        "va_runaway_dwell:=0.5",
        "va_altitude_abort_error:=5.0",
        "va_vertical_speed_abort:=2.0",
        "va_vertical_abort_min_airspeed:=6.0",
        f"output_csv:={run_dir / 'telemetry.csv'}",
        f"px4_workdir:={run_dir / 'px4_workdir'}",
    ]


def _assert_fresh_output_dir(output_dir: Path) -> None:
    protected_outputs = (
        "a0_ros_training_smoke_summary.json",
        "a0_deterministic_evaluation_summary.json",
        "training_metrics.csv",
        "evaluation_metrics.csv",
    )
    existing_telemetry = sorted(output_dir.glob("*episode_*/telemetry.csv"))
    if (
        any((output_dir / name).exists() for name in protected_outputs)
        or existing_telemetry
    ):
        suggestion = output_dir.with_name(f"{output_dir.name}_v2")
        raise FileExistsError(
            f"{output_dir} already contains A0 ROS training outputs; "
            f"found {len(existing_telemetry)} episode telemetry file(s). "
            f"Use a fresh directory, for example: {suggestion}"
        )


def run_training_smoke(
    output_dir: Path,
    *,
    episodes: int,
    timeout_seconds: float,
    seed: int,
    device: str | None,
    sim_speed_factor: float,
    enable_learning: bool,
    fixed_action: float | None,
    max_attempts: int | None = None,
    deterministic_evaluation: bool = False,
    policy_checkpoint: Path | None = None,
    normalizer_checkpoint: Path | None = None,
    learning_starts: int | None = None,
    exploration_hold_s: float = 0.0,
) -> dict[str, Any]:
    if sim_speed_factor <= 0.0 or not math.isfinite(sim_speed_factor):
        raise ValueError(f"sim_speed_factor must be positive, got {sim_speed_factor}")
    project_root = Path("/home/weicheng/ca_lsc_td3")
    _assert_fresh_output_dir(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    import rclpy

    rclpy.init()
    trainer = A0OnlineTrainer(
        output_dir=output_dir,
        seed=seed,
        device=device,
        enable_learning=enable_learning,
        fixed_action=fixed_action,
        deterministic_evaluation=deterministic_evaluation,
        policy_checkpoint=policy_checkpoint,
        normalizer_checkpoint=normalizer_checkpoint,
        learning_starts=learning_starts,
        exploration_hold_s=exploration_hold_s,
    )
    trainer.sim_speed_factor = sim_speed_factor
    trainer.requested_episodes = episodes
    trainer.warm_up_control_inference()
    trainer.warm_up_learning_compute()
    attempt_limit = (
        int(max_attempts)
        if max_attempts is not None else max(episodes * 2, episodes + 10)
    )
    if attempt_limit < episodes:
        raise ValueError("max_attempts must be at least episodes")
    try:
        valid_episode_count = 0
        attempt = 0
        while valid_episode_count < episodes and attempt < attempt_limit:
            attempt += 1
            episode = valid_episode_count + 1
            trainer.begin_episode(episode, attempt=attempt)
            run_dir = output_dir / (
                f"attempt_{attempt:03d}_episode_{episode:03d}"
            )
            if (run_dir / "telemetry.csv").exists():
                raise FileExistsError(f"refusing to overwrite {run_dir / 'telemetry.csv'}")
            run_dir.mkdir(parents=True, exist_ok=True)
            (run_dir / "px4_workdir").mkdir(parents=True, exist_ok=True)
            with (run_dir / "console.log").open("w", encoding="utf-8") as console:
                proc = subprocess.Popen(
                    _launch_command(
                        project_root,
                        run_dir,
                        sim_speed_factor=sim_speed_factor,
                        condition_prefix=(
                            "a0_deterministic_eval"
                            if deterministic_evaluation
                            else "a0_ros_training"
                        ),
                    ),
                    cwd=str(project_root),
                    stdout=console,
                    stderr=subprocess.STDOUT,
                    text=True,
                    preexec_fn=os.setsid,
                )
                started = time.monotonic()
                timed_out = False
                while proc.poll() is None:
                    trainer.rclpy.spin_once(trainer.node, timeout_sec=0.05)
                    trainer.publish_fixed_action_keepalive()
                    if time.monotonic() - started > timeout_seconds:
                        timed_out = True
                        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                        break
                deadline = time.monotonic() + 15.0
                while proc.poll() is None and time.monotonic() < deadline:
                    trainer.rclpy.spin_once(trainer.node, timeout_sec=0.05)
                    trainer.publish_fixed_action_keepalive()
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    proc.wait()
                drain_deadline = time.monotonic() + 2.0
                while time.monotonic() < drain_deadline:
                    trainer.rclpy.spin_once(trainer.node, timeout_sec=0.02)
                    if trainer._episode_terminal_seen:
                        break
                launch_status = int(proc.returncode or 0)
                wall_time_s = time.monotonic() - started
            learner_idle = trainer.wait_for_learning_idle()
            if not learner_idle:
                trainer._learner_exception = "learner_queue_drain_timeout"
            row = trainer.finish_episode(
                launch_status=launch_status,
                timed_out=timed_out,
                wall_time_s=wall_time_s,
            )
            trainer.attempt_rows.append(row)
            if row["valid_episode"]:
                trainer.episode_rows.append(row)
                valid_episode_count += 1
                print(
                    _episode_report(
                        row,
                        episodes,
                        label=("EVAL" if deterministic_evaluation else "TRAIN"),
                    ),
                    flush=True,
                )
            else:
                print(
                    f"[RETRY attempt={attempt:03d} target_episode={episode:03d}] "
                    f"reason={row['termination_reason']} "
                    f"infrastructure={row['infrastructure_failure_reason']}",
                    flush=True,
                )
        return trainer.write_outputs()
    finally:
        trainer.destroy()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--timeout-seconds", type=float, default=180.0)
    parser.add_argument("--seed", type=int, default=20260907)
    parser.add_argument("--device", default=None)
    parser.add_argument("--sim-speed-factor", type=float, default=1.0)
    parser.add_argument(
        "--learning-starts",
        type=int,
        default=None,
        help=(
            "Replay transitions collected before TD3 updates begin. The "
            "legacy integration-smoke default is 256."
        ),
    )
    parser.add_argument(
        "--exploration-hold-s",
        type=float,
        default=0.0,
        help=(
            "Simulation-time duration for each warm-up action or policy-noise "
            "sample. Zero preserves legacy IID per-step exploration."
        ),
    )
    parser.add_argument(
        "--max-attempts",
        type=int,
        default=None,
        help=(
            "Maximum simulator launches used to obtain the requested number "
            "of valid RL_ACTIVE episodes."
        ),
    )
    parser.add_argument(
        "--disable-learning",
        action="store_true",
        help=(
            "Record replay and publish online actions without TD3 updates. "
            "Use this for platform/simulation-speed qualification only."
        ),
    )
    parser.add_argument(
        "--fixed-action",
        type=float,
        default=None,
        help=(
            "Use a fixed normalized TD3 action in [-1, 1]. Intended for "
            "platform qualification, not learning experiments."
        ),
    )
    args = parser.parse_args()
    if args.fixed_action is not None and not -1.0 <= args.fixed_action <= 1.0:
        print("ERROR: --fixed-action must be in [-1, 1]", file=sys.stderr)
        return 2
    try:
        summary = run_training_smoke(
            args.output_dir,
            episodes=args.episodes,
            timeout_seconds=args.timeout_seconds,
            seed=args.seed,
            device=args.device,
            sim_speed_factor=args.sim_speed_factor,
            enable_learning=not args.disable_learning,
            fixed_action=args.fixed_action,
            max_attempts=args.max_attempts,
            learning_starts=args.learning_starts,
            exploration_hold_s=args.exploration_hold_s,
        )
    except (FileExistsError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0 if summary["a0_ros_training_smoke_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

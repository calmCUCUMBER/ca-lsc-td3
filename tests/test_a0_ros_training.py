from __future__ import annotations

from collections import Counter
import json
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest

import numpy as np
import torch

from ca_lsc_td3.rl.a0_ros_training import (
    A0OnlineTrainer,
    _launch_command,
    _attempt_infrastructure_failure,
    _evaluation_decision,
    _latched_initial_readiness,
    _terminal_semantics,
)
from ca_lsc_td3.rl.config import A0TD3Config
from ca_lsc_td3.rl.exploration import PiecewiseConstantExplorer
from ca_lsc_td3.rl.normalization import RunningObservationNormalizer
from ca_lsc_td3.rl.replay_buffer import ReplayBuffer
from ca_lsc_td3.rl.td3 import TD3Agent


class A0RosTrainingSemanticsTests(unittest.TestCase):
    def test_initial_admission_uses_transition_entry_latch_not_live_gates(self) -> None:
        result = _latched_initial_readiness({
            "a0_rl_initial_readiness_available": 1,
            "a0_rl_initial_readiness_latched": 1,
            "a0_rl_initial_height_gate_ok": 1,
            "a0_rl_initial_vz_gate_ok": 1,
            "a0_rl_initial_groundspeed_gate_ok": 1,
            # These post-entry gates are expected to change during acceleration.
            "height_gate_ok": 1,
            "vz_gate_ok": 0,
            "groundspeed_gate_ok": 0,
        })
        self.assertEqual(result, (True, True, True, True, True, ""))

    def test_missing_transition_entry_latch_is_infrastructure_invalid(self) -> None:
        result = _latched_initial_readiness({
            "height_gate_ok": 1,
            "vz_gate_ok": 1,
            "groundspeed_gate_ok": 1,
        })
        self.assertFalse(result[0])
        self.assertFalse(result[4])
        self.assertEqual(
            result[5],
            "initial_rl_state_pre_rl_readiness_unavailable",
        )

    def test_launch_contract_uses_handover_not_success_threshold_names(self) -> None:
        command = _launch_command(
            Path('/tmp/project'),
            Path('/tmp/run'),
            sim_speed_factor=4.0,
        )
        joined = ' '.join(command)
        self.assertIn('a0_rl_handover_lambda:=0.95', joined)
        self.assertIn('a0_rl_handover_dwell:=1.0', joined)
        self.assertNotIn('a0_rl_success_lambda', joined)
        self.assertNotIn('a0_rl_success_dwell', joined)

    def test_success_is_bellman_terminal_without_failure_penalty(self) -> None:
        terminated, truncated, failure = _terminal_semantics(
            reason="success", terminated=True, truncated=False
        )
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertFalse(failure)

    def test_task_runaway_is_bellman_terminal_failure(self) -> None:
        terminated, truncated, failure = _terminal_semantics(
            reason="va_hold_vertical_speed_runaway",
            terminated=True,
            truncated=False,
        )
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertTrue(failure)

    def test_action_timeout_is_infrastructure_truncation(self) -> None:
        terminated, truncated, failure = _terminal_semantics(
            reason="a0_rl_action_timeout",
            terminated=True,
            truncated=False,
        )
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertFalse(failure)

    def test_pre_rl_abort_is_not_a_valid_training_episode(self) -> None:
        reason = _attempt_infrastructure_failure(
            terminal_seen=True,
            reason="va_hold_altitude_runaway",
            rl_active_observation_count=0,
            launch_status=0,
            timed_out=False,
        )
        self.assertEqual(reason, "pre_rl_abort_before_rl_active")

    def test_task_failure_after_rl_active_is_valid_training_data(self) -> None:
        reason = _attempt_infrastructure_failure(
            terminal_seen=True,
            reason="forward_transition_timeout",
            rl_active_observation_count=100,
            launch_status=0,
            timed_out=False,
        )
        self.assertEqual(reason, "")

    def test_failed_initial_readiness_is_initialization_invalid(self) -> None:
        reason = _attempt_infrastructure_failure(
            terminal_seen=True,
            reason="va_hold_altitude_runaway",
            rl_active_observation_count=1,
            launch_status=0,
            timed_out=False,
            initialization_invalid_reason=(
                "initial_rl_state_pre_rl_readiness_failed"
            ),
        )
        self.assertEqual(reason, "initial_rl_state_pre_rl_readiness_failed")

    def test_missing_terminal_is_infrastructure_invalid(self) -> None:
        reason = _attempt_infrastructure_failure(
            terminal_seen=False,
            reason="running",
            rl_active_observation_count=10,
            launch_status=0,
            timed_out=False,
        )
        self.assertEqual(reason, "terminal_not_observed")

    def test_repeated_terminal_cannot_overwrite_first_physical_reason(self) -> None:
        trainer = A0OnlineTrainer.__new__(A0OnlineTrainer)
        trainer.config = A0TD3Config()
        trainer.nonfinite_observation_count = 0
        trainer.received_observation_count = 0
        trainer._episode_observation_sim_dt_values = []
        trainer._episode_observation_wall_dt_values = []
        trainer._episode_terminal_seen = True
        trainer._episode_success = False
        trainer._episode_reason = "va_hold_vertical_speed_runaway"
        message = SimpleNamespace(data=json.dumps({
            "observation": [0.0] * trainer.config.observation_dim,
            "sim_time_s": 42.0,
            "command_state": "TRANSITION_FW",
            "terminated": True,
            "truncated": False,
            "termination_reason": "a0_rl_action_timeout",
            "a0_rl_action_stale": 1,
            "a0_rl_control_phase": "rl_active",
            "a0_observation_sim_dt_s": 0.05,
            "a0_observation_wall_dt_s": 0.0125,
        }))

        trainer._observation_callback(message)

        self.assertEqual(
            trainer._episode_reason,
            "va_hold_vertical_speed_runaway",
        )
        self.assertFalse(trainer._episode_success)

    def test_terminal_payload_metadata_is_latched_for_metrics(self) -> None:
        trainer = A0OnlineTrainer.__new__(A0OnlineTrainer)
        trainer.config = A0TD3Config()
        trainer.nonfinite_observation_count = 0
        trainer.received_observation_count = 0
        trainer._episode_observation_sim_dt_values = []
        trainer._episode_observation_wall_dt_values = []
        trainer._episode_terminal_seen = False
        trainer._episode_success = False
        trainer._episode_reason = "running"
        trainer._episode_terminal_command_state = ""
        trainer._episode_terminal_handover_ready = False
        trainer._episode_terminal_handover_ready_dwell_s = float("nan")
        trainer._episode_terminal_actual_fw_confirmed = False
        trainer._previous_observation = None
        trainer._previous_action = None
        message = SimpleNamespace(data=json.dumps({
            "observation": [0.0] * trainer.config.observation_dim,
            "sim_time_s": 42.0,
            "command_state": "HOLD_FW",
            "terminated": True,
            "truncated": False,
            "termination_reason": "success",
            "a0_rl_action_stale": 0,
            "a0_rl_control_phase": "rl_active",
            "a0_rl_handover_ready": 1,
            "a0_rl_handover_ready_dwell_s": 1.25,
            "a0_rl_actual_fw_confirmed": 1,
        }))

        trainer._observation_callback(message)

        self.assertTrue(trainer._episode_success)
        self.assertEqual(trainer._episode_reason, "success")
        self.assertEqual(trainer._episode_terminal_command_state, "HOLD_FW")
        self.assertTrue(trainer._episode_terminal_handover_ready)
        self.assertAlmostEqual(
            trainer._episode_terminal_handover_ready_dwell_s,
            1.25,
        )
        self.assertTrue(trainer._episode_terminal_actual_fw_confirmed)

    def test_compute_warmup_does_not_advance_real_agent(self) -> None:
        trainer = A0OnlineTrainer.__new__(A0OnlineTrainer)
        trainer.enable_learning = True
        trainer.config = A0TD3Config(batch_size=8)
        trainer.agent = TD3Agent(trainer.config, seed=11, device="cpu")
        parameters_before = [
            parameter.detach().clone()
            for parameter in trainer.agent.actor.parameters()
        ]
        self.assertEqual(trainer.agent.actor_optimizer.state, {})
        self.assertEqual(trainer.agent.critic_optimizer.state, {})

        trainer.warm_up_learning_compute()

        self.assertEqual(trainer.agent.total_updates, 0)
        self.assertEqual(trainer.agent.actor_optimizer.state, {})
        self.assertEqual(trainer.agent.critic_optimizer.state, {})
        for before, after in zip(
            parameters_before,
            trainer.agent.actor.parameters(),
        ):
            self.assertTrue(torch.equal(before, after))

    def test_deterministic_evaluation_action_has_no_exploration_noise(self) -> None:
        trainer = A0OnlineTrainer.__new__(A0OnlineTrainer)
        trainer.config = A0TD3Config()
        trainer.agent = TD3Agent(trainer.config, seed=17, device="cpu")
        trainer.control_actor = trainer.agent.actor
        trainer._control_actor_lock = threading.Lock()
        trainer.normalizer = RunningObservationNormalizer(
            trainer.config.observation_dim
        )
        trainer.deterministic_evaluation = True
        trainer.rng = np.random.default_rng(17)
        trainer.explorer = PiecewiseConstantExplorer(trainer.rng, hold_s=5.0)

        observation = np.zeros(trainer.config.observation_dim, dtype=np.float32)
        expected = trainer.agent.select_action(observation, explore=False)
        actual = trainer._select_control_action(observation, sim_time_s=1.0)

        np.testing.assert_array_equal(actual, expected)

    def test_deterministic_evaluation_bypasses_empty_replay_warmup(self) -> None:
        class Message:
            data = ""

        class Publisher:
            def __init__(self) -> None:
                self.messages = []

            def publish(self, message: Message) -> None:
                self.messages.append(message)

        trainer = A0OnlineTrainer.__new__(A0OnlineTrainer)
        trainer.config = A0TD3Config(learning_starts=2000)
        trainer.agent = TD3Agent(trainer.config, seed=23, device="cpu")
        trainer.control_actor = trainer.agent.actor.eval()
        trainer._control_actor_lock = threading.Lock()
        trainer.normalizer = RunningObservationNormalizer(
            trainer.config.observation_dim
        )
        trainer.deterministic_evaluation = True
        trainer.fixed_action = None
        trainer.replay = ReplayBuffer(
            capacity=8,
            observation_dim=trainer.config.observation_dim,
            action_dim=trainer.config.action_dim,
            seed=23,
        )
        trainer.rng = np.random.default_rng(23)
        trainer.explorer = PiecewiseConstantExplorer(trainer.rng, hold_s=0.0)
        trainer.String = Message
        trainer.publisher = Publisher()
        trainer.sequence_id = 0
        trainer.action_publish_count = 0
        trainer.action_source_counts = Counter()
        trainer._episode_actions = 0
        trainer._episode_warmup_action_count = 0
        trainer._episode_action_source_counts = Counter()

        observation = np.zeros(trainer.config.observation_dim, dtype=np.float32)
        expected = trainer.agent.select_action(observation, explore=False)
        actual = trainer._publish_action(observation, sim_time_s=1.0)
        payload = json.loads(trainer.publisher.messages[0].data)

        np.testing.assert_array_equal(actual, expected)
        self.assertEqual(payload["action_source"], "actor_deterministic")
        self.assertEqual(trainer._episode_warmup_action_count, 0)
        self.assertEqual(trainer.explorer.segment_count, 0)
        self.assertEqual(
            trainer._episode_action_source_counts["actor_deterministic"],
            1,
        )

    def test_deterministic_pretransition_keepalive_uses_actor(self) -> None:
        trainer = A0OnlineTrainer.__new__(A0OnlineTrainer)
        trainer.deterministic_evaluation = True
        trainer._next_keepalive_sim_time_s = None
        captured = []

        def publish(observation, sim_time_s, *, count_episode, forced_action):
            captured.append((observation, sim_time_s, count_episode, forced_action))

        trainer._publish_action = publish
        observation = np.zeros(10, dtype=np.float32)
        trainer._publish_safe_keepalive(observation, 2.0)

        self.assertEqual(len(captured), 1)
        self.assertFalse(captured[0][2])
        self.assertIsNone(captured[0][3])

    def test_preregistered_evaluation_decision_bands(self) -> None:
        self.assertEqual(
            _evaluation_decision(2, 10),
            ("A_0_to_20_percent", "continue_a0_training_to_100_episodes"),
        )
        self.assertEqual(
            _evaluation_decision(5, 10),
            ("B_30_to_50_percent", "continue_a0_training_to_100_episodes"),
        )
        self.assertEqual(
            _evaluation_decision(8, 10),
            (
                "C_60_to_80_percent",
                "candidate_a0_nominal_baseline_then_review_safety",
            ),
        )
        self.assertEqual(
            _evaluation_decision(10, 10),
            (
                "D_90_to_100_percent",
                "freeze_a0_nominal_baseline_and_start_a1",
            ),
        )

    def test_deterministic_evaluation_does_not_write_replay(self) -> None:
        trainer = A0OnlineTrainer.__new__(A0OnlineTrainer)
        trainer.deterministic_evaluation = True
        trainer.replay = ReplayBuffer(
            capacity=8,
            observation_dim=10,
            action_dim=1,
            seed=3,
        )
        trainer._previous_observation = np.zeros(10, dtype=np.float32)
        trainer._previous_action = np.zeros(1, dtype=np.float32)
        trainer._previous_sim_time_s = 1.0
        trainer._episode_rl_step_sim_dt_values = []
        trainer.infrastructure_failure_penalty_count = 0
        trainer._episode_return = 0.0
        trainer._episode_running_reward_return = 0.0
        trainer._episode_progress_shaping_return = 0.0
        trainer._episode_terminal_reward_return = 0.0
        trainer._episode_steps = 0
        trainer._max_altitude_error = 0.0
        trainer._max_abs_alpha = 0.0
        trainer._max_descent_rate = 0.0
        trainer._max_abs_vertical_speed = 0.0
        trainer._previous_lambda_exec = 0.0
        trainer._lambda_smoothness = 0.0
        trainer._previous_rl_step_wall_time_s = None
        trainer._episode_rl_step_wall_dt_values = []
        trainer._episode_first_rl_sim_time_s = None
        trainer._episode_last_rl_sim_time_s = None
        trainer._episode_first_rl_wall_time_s = None
        trainer._episode_last_rl_wall_time_s = None
        trainer._episode_lambda_sum = 0.0
        trainer._episode_lambda_count = 0
        trainer._episode_lambda_max = 0.0
        trainer._episode_lambda_saturation_count = 0
        trainer._episode_lambda_ge_0p9_started_at = None
        trainer._episode_lambda_ge_0p9_max_dwell_s = 0.0

        next_observation = np.zeros(10, dtype=np.float32)
        next_observation[3] = 0.25
        next_observation[4] = -0.5
        next_observation[7] = 0.2
        trainer._record_transition(
            observation=next_observation,
            sim_time_s=1.1,
            terminated=False,
            truncated=False,
            reason="running",
        )

        self.assertEqual(trainer.replay.size, 0)
        self.assertEqual(trainer._episode_steps, 1)
        self.assertAlmostEqual(trainer._max_abs_vertical_speed, 0.5)

    def test_initialization_invalid_attempt_does_not_write_replay(self) -> None:
        trainer = A0OnlineTrainer.__new__(A0OnlineTrainer)
        trainer.deterministic_evaluation = False
        trainer._episode_initialization_invalid_reason = (
            "initial_rl_state_pre_rl_readiness_failed"
        )
        trainer.replay = ReplayBuffer(
            capacity=8,
            observation_dim=10,
            action_dim=1,
            seed=3,
        )
        trainer.replay_write_count = 0
        trainer._previous_observation = np.zeros(10, dtype=np.float32)
        trainer._previous_action = np.zeros(1, dtype=np.float32)
        trainer._previous_sim_time_s = 1.0
        trainer._episode_rl_step_sim_dt_values = []
        trainer.infrastructure_failure_penalty_count = 0
        trainer._episode_return = 0.0
        trainer._episode_running_reward_return = 0.0
        trainer._episode_progress_shaping_return = 0.0
        trainer._episode_terminal_reward_return = 0.0
        trainer._episode_steps = 0
        trainer._max_altitude_error = 0.0
        trainer._max_abs_alpha = 0.0
        trainer._max_descent_rate = 0.0
        trainer._max_abs_vertical_speed = 0.0
        trainer._previous_lambda_exec = 0.0
        trainer._lambda_smoothness = 0.0
        trainer._previous_rl_step_wall_time_s = None
        trainer._episode_rl_step_wall_dt_values = []
        trainer._episode_first_rl_sim_time_s = None
        trainer._episode_last_rl_sim_time_s = None
        trainer._episode_first_rl_wall_time_s = None
        trainer._episode_last_rl_wall_time_s = None
        trainer._episode_lambda_sum = 0.0
        trainer._episode_lambda_count = 0
        trainer._episode_lambda_max = 0.0
        trainer._episode_lambda_saturation_count = 0
        trainer._episode_lambda_ge_0p9_started_at = None
        trainer._episode_lambda_ge_0p9_max_dwell_s = 0.0

        trainer._record_transition(
            observation=np.zeros(10, dtype=np.float32),
            sim_time_s=1.1,
            terminated=True,
            truncated=False,
            reason="va_hold_altitude_runaway",
        )

        self.assertEqual(trainer.replay.size, 0)
        self.assertEqual(trainer.replay_write_count, 0)

    def test_lambda_high_dwell_uses_simulation_time(self) -> None:
        trainer = A0OnlineTrainer.__new__(A0OnlineTrainer)
        trainer.deterministic_evaluation = True
        trainer.replay = ReplayBuffer(
            capacity=8,
            observation_dim=10,
            action_dim=1,
            seed=3,
        )
        trainer._previous_action = np.zeros(1, dtype=np.float32)
        trainer._episode_rl_step_sim_dt_values = []
        trainer.infrastructure_failure_penalty_count = 0
        trainer._episode_return = 0.0
        trainer._episode_running_reward_return = 0.0
        trainer._episode_progress_shaping_return = 0.0
        trainer._episode_terminal_reward_return = 0.0
        trainer._episode_steps = 0
        trainer._max_altitude_error = 0.0
        trainer._max_abs_alpha = 0.0
        trainer._max_descent_rate = 0.0
        trainer._max_abs_vertical_speed = 0.0
        trainer._previous_lambda_exec = 0.0
        trainer._lambda_smoothness = 0.0
        trainer._previous_rl_step_wall_time_s = None
        trainer._episode_rl_step_wall_dt_values = []
        trainer._episode_first_rl_sim_time_s = None
        trainer._episode_last_rl_sim_time_s = None
        trainer._episode_first_rl_wall_time_s = None
        trainer._episode_last_rl_wall_time_s = None
        trainer._episode_lambda_sum = 0.0
        trainer._episode_lambda_count = 0
        trainer._episode_lambda_max = 0.0
        trainer._episode_lambda_saturation_count = 0
        trainer._episode_lambda_ge_0p9_started_at = None
        trainer._episode_lambda_ge_0p9_max_dwell_s = 0.0

        for sim_time_s in (1.0, 1.5, 2.1):
            observation = np.zeros(10, dtype=np.float32)
            observation[7] = 0.95
            trainer._previous_observation = observation.copy()
            trainer._previous_sim_time_s = sim_time_s - 0.1
            trainer._record_transition(
                observation=observation,
                sim_time_s=sim_time_s,
                terminated=False,
                truncated=False,
                reason="running",
            )

        self.assertAlmostEqual(
            trainer._episode_lambda_ge_0p9_max_dwell_s,
            1.1,
        )


if __name__ == "__main__":
    unittest.main()

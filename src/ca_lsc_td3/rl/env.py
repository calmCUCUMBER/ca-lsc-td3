"""A0 Vanilla TD3 smoke environment.

This is not the final ROS/PX4 training environment.  It is a deterministic,
cheap contract test that exercises reset, finite observations, action mapping,
replay, TD3 updates, termination, checkpointing, and telemetry before the
expensive SITL environment is connected.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from .config import action_to_lambda
from .observations import pack_a0_observation
from .reward import debug_reward


@dataclass
class ToyTransitionState:
    airspeed_mps: float = 0.0
    alpha_rad: float = 0.0
    gamma_rad: float = 0.0
    altitude_error_m: float = 0.0
    vertical_speed_up_mps: float = 0.0
    pitch_rad: float = 0.0
    pitch_rate_rps: float = 0.0
    lambda_exec: float = 0.0
    lift_power_w: float = 1800.0
    pusher_power_w: float = 0.0


class ToyA0TransitionEnv(gym.Env):
    """Small deterministic environment for A0 training-code smoke."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        *,
        seed: int = 0,
        max_steps: int = 300,
        dt_s: float = 0.1,
    ) -> None:
        super().__init__()
        self.max_steps = int(max_steps)
        self.dt_s = float(dt_s)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)
        high = np.full((10,), np.inf, dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        self._rng = np.random.default_rng(seed)
        self._state = ToyTransitionState()
        self._step_count = 0
        self._previous_lambda = 0.0
        self._success_dwell = 0

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self._step_count = 0
        self._previous_lambda = 0.0
        self._success_dwell = 0
        self._state = ToyTransitionState(
            airspeed_mps=float(self._rng.normal(0.0, 0.05)),
            altitude_error_m=float(self._rng.normal(0.0, 0.05)),
            lift_power_w=1800.0,
        )
        return self._observation(), {"termination_reason": "reset"}

    def step(
        self,
        action: np.ndarray,
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (1,) or not np.all(np.isfinite(action)):
            raise ValueError("A0 action must be finite with shape (1,)")
        lambda_d = action_to_lambda(float(action[0]))
        previous_airspeed_mps = self._state.airspeed_mps
        previous_lambda_exec = self._state.lambda_exec
        # Match the real allocator direction: unloading rises slowly and
        # recovers faster.
        rate = 0.25 if lambda_d >= self._state.lambda_exec else 1.0
        max_delta = rate * self.dt_s
        delta = np.clip(lambda_d - self._state.lambda_exec, -max_delta, max_delta)
        lambda_exec = float(np.clip(self._state.lambda_exec + delta, 0.0, 1.0))

        airspeed_error = 15.0 - self._state.airspeed_mps
        # Keep the toy plant consistent with the frozen schema-13/SITL
        # contract: pusher control is independent from lift unloading and must
        # still hold Va≈15 m/s when lambda≈1.  The old trim 0.25 made the
        # success condition mathematically unreachable at high lambda.
        pusher = float(np.clip(0.37 + 0.04 * airspeed_error, 0.0, 0.65))
        wing_support = np.clip((self._state.airspeed_mps / 15.0) ** 2, 0.0, 1.25)
        rotor_support = max(0.0, 1.0 - lambda_exec)
        support_error = rotor_support + wing_support - 1.0

        airspeed = self._state.airspeed_mps + self.dt_s * (
            9.0 * pusher - 0.18 * self._state.airspeed_mps - 0.6 * lambda_exec
        )
        vertical_speed = 0.92 * self._state.vertical_speed_up_mps + 0.35 * support_error
        altitude_error = self._state.altitude_error_m + self.dt_s * vertical_speed
        pitch = 0.08 + 0.10 * lambda_exec - 0.02 * airspeed
        pitch_rate = (pitch - self._state.pitch_rad) / self.dt_s
        gamma = math.atan2(vertical_speed, max(airspeed, 0.1))
        alpha = pitch - gamma
        lift_power = 1800.0 * max(0.0, 1.0 - lambda_exec) ** 1.5
        pusher_power = 220.0 * pusher**3

        self._state = ToyTransitionState(
            airspeed_mps=float(np.clip(airspeed, 0.0, 25.0)),
            alpha_rad=float(np.clip(alpha, -0.8, 0.8)),
            gamma_rad=float(np.clip(gamma, -0.5, 0.5)),
            altitude_error_m=float(np.clip(altitude_error, -20.0, 20.0)),
            vertical_speed_up_mps=float(np.clip(vertical_speed, -5.0, 5.0)),
            pitch_rad=float(np.clip(pitch, -0.8, 0.8)),
            pitch_rate_rps=float(np.clip(pitch_rate, -5.0, 5.0)),
            lambda_exec=lambda_exec,
            lift_power_w=float(lift_power),
            pusher_power_w=float(pusher_power),
        )
        self._step_count += 1

        success_condition = (
            self._state.airspeed_mps >= 14.0
            and self._state.lambda_exec >= 0.90
            and abs(self._state.altitude_error_m) <= 3.0
            and abs(self._state.alpha_rad) <= 0.35
        )
        self._success_dwell = self._success_dwell + 1 if success_condition else 0
        success = self._success_dwell >= 10
        hard_failure = (
            abs(self._state.altitude_error_m) > 5.0
            or self._state.vertical_speed_up_mps < -2.0
            or abs(self._state.alpha_rad) > 0.60
        )
        truncated = self._step_count >= self.max_steps
        terminated = success or hard_failure
        lambda_delta = self._state.lambda_exec - self._previous_lambda
        self._previous_lambda = self._state.lambda_exec
        reward = debug_reward(
            dt_s=self.dt_s,
            previous_airspeed_mps=previous_airspeed_mps,
            airspeed_mps=self._state.airspeed_mps,
            previous_lambda_executed=previous_lambda_exec,
            lambda_executed=self._state.lambda_exec,
            altitude_error_m=self._state.altitude_error_m,
            pitch_rate_rps=self._state.pitch_rate_rps,
            alpha_rad=self._state.alpha_rad,
            lambda_delta=lambda_delta,
            success=success,
            failure=hard_failure,
            unsafe_failure=hard_failure,
        )
        reason = (
            "success" if success else
            "hard_failure" if hard_failure else
            "time_limit" if truncated else
            "running"
        )
        return self._observation(), reward, terminated, truncated, {
            "success": success,
            "termination_reason": reason,
            "lambda_d": lambda_d,
            "lambda_exec": self._state.lambda_exec,
            "max_altitude_error": abs(self._state.altitude_error_m),
            "max_abs_alpha": abs(self._state.alpha_rad),
            "descent_rate": max(0.0, -self._state.vertical_speed_up_mps),
        }

    def _observation(self) -> np.ndarray:
        return pack_a0_observation({
            "airspeed_mps": self._state.airspeed_mps,
            "angle_of_attack_rad": self._state.alpha_rad,
            "flight_path_angle_rad": self._state.gamma_rad,
            "altitude_error_m": self._state.altitude_error_m,
            "vertical_speed_up_mps": self._state.vertical_speed_up_mps,
            "pitch_angle_rad": self._state.pitch_rad,
            "pitch_rate_rps": self._state.pitch_rate_rps,
            "lambda_executed": self._state.lambda_exec,
            "lift_rotor_power_w": self._state.lift_power_w,
            "pusher_power_w": self._state.pusher_power_w,
        })

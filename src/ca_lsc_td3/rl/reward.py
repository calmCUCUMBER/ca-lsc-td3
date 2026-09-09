"""Capability-free common reward used before power qualification.

The progress term deliberately uses only variables available to A0.  In
particular, eta_L and eta_C are excluded so the same reward can be shared by
the later capability-feature ablations without leaking their information into
the vanilla baseline.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


COMMON_REWARD_VERSION = "capability_free_transition_potential_safety_v2"


@dataclass(frozen=True)
class DebugRewardWeights:
    time: float = 0.02
    altitude: float = 1.0
    pitch_rate: float = 0.10
    alpha: float = 1.0
    lambda_smoothness: float = 0.05
    success_bonus: float = 20.0
    # Generic/timeout failures retain the original terminal cost.  Physical
    # safety violations terminate earlier and therefore need a larger cost so
    # a short runaway cannot be preferable to a long, stable timeout.
    failure_penalty: float = 20.0
    unsafe_failure_penalty: float = 35.0
    altitude_reference_m: float = 5.0
    pitch_rate_reference_rps: float = 1.0
    alpha_soft_rad: float = 0.25
    progress_discount: float = 0.99
    airspeed_progress_weight: float = 1.0
    lambda_progress_weight: float = 4.0
    airspeed_progress_start_mps: float = 8.0
    airspeed_handover_mps: float = 13.0
    lambda_progress_start: float = 0.0
    lambda_handover: float = 0.95


@dataclass(frozen=True)
class CommonRewardTerms:
    """Auditable decomposition of one common-reward transition."""

    running: float
    progress_shaping: float
    terminal: float

    @property
    def total(self) -> float:
        return float(self.running + self.progress_shaping + self.terminal)


def alpha_penalty(alpha_rad: float, soft_limit_rad: float) -> float:
    if not math.isfinite(alpha_rad):
        return 1.0
    return max(0.0, abs(alpha_rad) - soft_limit_rad) ** 2


def _saturated_linear(value: float, lower: float, upper: float) -> float:
    if not all(math.isfinite(item) for item in (value, lower, upper)):
        raise ValueError("progress-potential inputs must be finite")
    if upper <= lower:
        raise ValueError("progress-potential upper bound must exceed lower bound")
    return min(1.0, max(0.0, (value - lower) / (upper - lower)))


def transition_progress_potential(
    *,
    airspeed_mps: float,
    lambda_executed: float,
    weights: DebugRewardWeights = DebugRewardWeights(),
) -> float:
    """Return capability-free progress toward the frozen handover region.

    This is not an absolute high-lambda reward.  It is consumed only through
    ``gamma * Phi(next) - Phi(current)`` below, so increasing lambda and then
    abandoning the handover region does not provide a permanent per-step
    reward.  Existing altitude, pitch-rate, AoA, and terminal penalties remain
    independent of the potential.
    """
    g_v = _saturated_linear(
        airspeed_mps,
        weights.airspeed_progress_start_mps,
        weights.airspeed_handover_mps,
    )
    g_lambda = _saturated_linear(
        lambda_executed,
        weights.lambda_progress_start,
        weights.lambda_handover,
    )
    return float(
        weights.airspeed_progress_weight * g_v
        + weights.lambda_progress_weight * g_lambda
    )


def common_reward_terms(
    *,
    dt_s: float,
    previous_airspeed_mps: float,
    airspeed_mps: float,
    previous_lambda_executed: float,
    lambda_executed: float,
    altitude_error_m: float,
    pitch_rate_rps: float,
    alpha_rad: float,
    lambda_delta: float,
    success: bool,
    failure: bool = False,
    unsafe_failure: bool = False,
    weights: DebugRewardWeights = DebugRewardWeights(),
) -> CommonRewardTerms:
    """Return the common reward split into running, shaping, and terminal terms."""
    values = (
        dt_s,
        previous_airspeed_mps,
        airspeed_mps,
        previous_lambda_executed,
        lambda_executed,
        altitude_error_m,
        pitch_rate_rps,
        alpha_rad,
        lambda_delta,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("reward inputs must be finite")
    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive")
    if not 0.0 <= weights.progress_discount <= 1.0:
        raise ValueError("progress_discount must lie in [0, 1]")
    if unsafe_failure and not failure:
        raise ValueError("unsafe_failure requires failure=True")

    running = -weights.time * dt_s
    running -= weights.altitude * (
        altitude_error_m / weights.altitude_reference_m
    ) ** 2
    running -= weights.pitch_rate * (
        pitch_rate_rps / weights.pitch_rate_reference_rps
    ) ** 2
    running -= weights.alpha * alpha_penalty(alpha_rad, weights.alpha_soft_rad)
    running -= weights.lambda_smoothness * lambda_delta**2

    previous_potential = transition_progress_potential(
        airspeed_mps=previous_airspeed_mps,
        lambda_executed=previous_lambda_executed,
        weights=weights,
    )
    # Task terminals transition to a shared absorbing state with Phi=0.  This
    # is required for policy-invariant potential shaping in an episodic MDP.
    # Infrastructure truncations call this function with both flags false and
    # therefore retain the physical next-state potential for bootstrapping.
    next_potential = (
        0.0
        if success or failure
        else transition_progress_potential(
            airspeed_mps=airspeed_mps,
            lambda_executed=lambda_executed,
            weights=weights,
        )
    )
    progress_shaping = (
        weights.progress_discount * next_potential - previous_potential
    )

    terminal = 0.0
    if success:
        terminal += weights.success_bonus
    if failure:
        terminal -= (
            weights.unsafe_failure_penalty
            if unsafe_failure else weights.failure_penalty
        )
    return CommonRewardTerms(
        running=float(running),
        progress_shaping=float(progress_shaping),
        terminal=float(terminal),
    )


def debug_reward(
    *,
    dt_s: float,
    previous_airspeed_mps: float,
    airspeed_mps: float,
    previous_lambda_executed: float,
    lambda_executed: float,
    altitude_error_m: float,
    pitch_rate_rps: float,
    alpha_rad: float,
    lambda_delta: float,
    success: bool,
    failure: bool = False,
    unsafe_failure: bool = False,
    weights: DebugRewardWeights = DebugRewardWeights(),
) -> float:
    """Return the capability-free common reward for A0 and later ablations.

    The current propulsion power proxy is intentionally not used.  It remains
    telemetry only until Power Qualification validates a defensible power
    channel.
    """
    return common_reward_terms(
        dt_s=dt_s,
        previous_airspeed_mps=previous_airspeed_mps,
        airspeed_mps=airspeed_mps,
        previous_lambda_executed=previous_lambda_executed,
        lambda_executed=lambda_executed,
        altitude_error_m=altitude_error_m,
        pitch_rate_rps=pitch_rate_rps,
        alpha_rad=alpha_rad,
        lambda_delta=lambda_delta,
        success=success,
        failure=failure,
        unsafe_failure=unsafe_failure,
        weights=weights,
    ).total

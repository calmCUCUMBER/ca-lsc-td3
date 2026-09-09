"""Hard command-envelope projection and asymmetric lambda rate limiting."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


def _unit_interval(value: float) -> float:
    if not math.isfinite(value):
        raise ValueError("shield values must be finite")
    return min(1.0, max(0.0, float(value)))


def hard_unloading_limit(gates: Iterable[float]) -> float:
    """Return lambda_max_hard as the minimum normalized envelope gate."""
    values = tuple(_unit_interval(value) for value in gates)
    if not values:
        raise ValueError("at least one hard-envelope gate is required")
    return min(values)


@dataclass(frozen=True)
class AsymmetricRateLimiter:
    """Limit unloading slowly while allowing faster lift recovery."""

    unloading_rate_per_s: float
    recovery_rate_per_s: float
    emergency_recovery_rate_per_s: float | None = None

    def __post_init__(self) -> None:
        rates = (self.unloading_rate_per_s, self.recovery_rate_per_s)
        if not all(math.isfinite(value) and value > 0.0 for value in rates):
            raise ValueError("rate limits must be finite and positive")
        if self.unloading_rate_per_s >= self.recovery_rate_per_s:
            raise ValueError(
                "recovery rate must exceed unloading rate"
            )
        emergency = self.emergency_recovery_rate_per_s
        if emergency is not None:
            if not math.isfinite(emergency) or emergency <= 0.0:
                raise ValueError("emergency recovery rate must be positive")
            if emergency < self.recovery_rate_per_s:
                raise ValueError(
                    "emergency recovery cannot be slower than normal recovery"
                )

    def step(
        self,
        previous: float,
        target: float,
        dt_s: float,
        *,
        emergency_recovery: bool = False,
    ) -> float:
        """Advance an executed unloading command by one control interval."""
        previous = _unit_interval(previous)
        target = _unit_interval(target)
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("dt_s must be finite and positive")

        delta = target - previous
        if delta >= 0.0:
            limited_delta = min(delta, self.unloading_rate_per_s * dt_s)
        else:
            recovery_rate = self.recovery_rate_per_s
            if emergency_recovery and self.emergency_recovery_rate_per_s:
                recovery_rate = self.emergency_recovery_rate_per_s
            limited_delta = max(delta, -recovery_rate * dt_s)
        return _unit_interval(previous + limited_delta)


@dataclass(frozen=True)
class ShieldResult:
    """Traceable candidate, projected, and executed unloading commands."""

    candidate: float
    hard_limit: float
    projected: float
    executed: float

    @property
    def projection_intervention(self) -> float:
        return abs(self.candidate - self.projected)

    @property
    def total_intervention(self) -> float:
        return abs(self.candidate - self.executed)


def apply_unloading_shield(
    *,
    candidate: float,
    gates: Iterable[float],
    previous_executed: float,
    dt_s: float,
    rate_limiter: AsymmetricRateLimiter,
    emergency_recovery: bool = False,
) -> ShieldResult:
    """Project a candidate lambda and apply the asymmetric rate limiter."""
    candidate = _unit_interval(candidate)
    limit = hard_unloading_limit(gates)
    projected = min(candidate, limit)
    rate_limited = rate_limiter.step(
        previous_executed,
        projected,
        dt_s,
        emergency_recovery=emergency_recovery,
    )
    # A contracting hard envelope has priority over the nominal recovery
    # rate. The limiter shapes ordinary commands, but may never leave the
    # command above the currently valid hard limit.
    executed = min(rate_limited, limit)
    return ShieldResult(candidate, limit, projected, executed)

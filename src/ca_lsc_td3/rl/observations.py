"""Canonical observation ordering used by every policy baseline."""

from __future__ import annotations

import math
from collections.abc import Mapping

import numpy as np


OBSERVATION_FIELDS = (
    "airspeed_mps",
    "angle_of_attack_rad",
    "flight_path_angle_rad",
    "eta_l",
    "eta_c",
    "altitude_error_m",
    "vertical_speed_up_mps",
    "pitch_angle_rad",
    "pitch_rate_rps",
    "lambda_executed",
    "lift_rotor_power_w",
    "pusher_power_w",
)


A0_OBSERVATION_FIELDS = (
    "airspeed_mps",
    "angle_of_attack_rad",
    "flight_path_angle_rad",
    "altitude_error_m",
    "vertical_speed_up_mps",
    "pitch_angle_rad",
    "pitch_rate_rps",
    "lambda_executed",
    "lift_rotor_power_w",
    "pusher_power_w",
)


def pack_observation(values: Mapping[str, float]) -> np.ndarray:
    """Pack raw up-positive SI observations in one stable 12-D order.

    This function deliberately does not normalize. Training code must fit
    preprocessing on training data, freeze it for evaluation, and share it
    across every MLP/LSTM/CfC baseline.
    """
    missing = [field for field in OBSERVATION_FIELDS if field not in values]
    if missing:
        raise KeyError(f"missing observation fields: {', '.join(missing)}")
    packed = np.asarray(
        [float(values[field]) for field in OBSERVATION_FIELDS],
        dtype=np.float32,
    )
    if not all(math.isfinite(float(value)) for value in packed):
        raise ValueError("observation values must be finite")
    return packed


def pack_a0_observation(values: Mapping[str, float]) -> np.ndarray:
    """Pack the frozen A0 Vanilla TD3 10-D observation.

    A0 deliberately excludes eta_L and eta_C.  It is the baseline that tests
    whether capability features add value in A1, so sharing the 12-D contract
    here would quietly contaminate the ablation.
    """
    missing = [field for field in A0_OBSERVATION_FIELDS if field not in values]
    if missing:
        raise KeyError(f"missing A0 observation fields: {', '.join(missing)}")
    packed = np.asarray(
        [float(values[field]) for field in A0_OBSERVATION_FIELDS],
        dtype=np.float32,
    )
    if not all(math.isfinite(float(value)) for value in packed):
        raise ValueError("A0 observation values must be finite")
    return packed

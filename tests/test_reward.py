from __future__ import annotations

import unittest

from ca_lsc_td3.rl.config import A0TD3Config
from ca_lsc_td3.rl.reward import (
    DebugRewardWeights,
    common_reward_terms,
    transition_progress_potential,
)


class CommonRewardTests(unittest.TestCase):
    def test_progress_discount_matches_td3_discount(self) -> None:
        self.assertEqual(
            DebugRewardWeights().progress_discount,
            A0TD3Config().gamma,
        )

    def test_potential_is_bounded_by_preregistered_weights(self) -> None:
        weights = DebugRewardWeights()
        lower = transition_progress_potential(
            airspeed_mps=0.0,
            lambda_executed=0.0,
            weights=weights,
        )
        upper = transition_progress_potential(
            airspeed_mps=30.0,
            lambda_executed=1.0,
            weights=weights,
        )
        self.assertEqual(lower, 0.0)
        self.assertEqual(
            upper,
            weights.airspeed_progress_weight
            + weights.lambda_progress_weight,
        )

    def test_joint_transition_progress_is_positive(self) -> None:
        terms = common_reward_terms(
            dt_s=0.1,
            previous_airspeed_mps=10.0,
            airspeed_mps=11.0,
            previous_lambda_executed=0.2,
            lambda_executed=0.3,
            altitude_error_m=0.0,
            pitch_rate_rps=0.0,
            alpha_rad=0.0,
            lambda_delta=0.1,
            success=False,
            failure=False,
        )
        self.assertGreater(terms.progress_shaping, 0.0)

    def test_retreat_from_handover_region_is_negative(self) -> None:
        terms = common_reward_terms(
            dt_s=0.1,
            previous_airspeed_mps=15.0,
            airspeed_mps=12.0,
            previous_lambda_executed=0.95,
            lambda_executed=0.7,
            altitude_error_m=0.0,
            pitch_rate_rps=0.0,
            alpha_rad=0.0,
            lambda_delta=-0.25,
            success=False,
            failure=False,
        )
        self.assertLess(terms.progress_shaping, 0.0)

    def test_terminal_ordering_keeps_runaway_below_timeout_below_success(self) -> None:
        common = dict(
            dt_s=0.1,
            previous_airspeed_mps=14.8,
            airspeed_mps=15.0,
            previous_lambda_executed=0.90,
            lambda_executed=0.95,
            pitch_rate_rps=0.0,
            alpha_rad=0.0,
            lambda_delta=0.05,
        )
        success = common_reward_terms(
            **common,
            altitude_error_m=0.0,
            success=True,
            failure=False,
        ).total
        timeout = common_reward_terms(
            **common,
            altitude_error_m=0.0,
            success=False,
            failure=True,
        ).total
        runaway = common_reward_terms(
            **common,
            altitude_error_m=5.0,
            success=False,
            failure=True,
            unsafe_failure=True,
        ).total
        self.assertLess(runaway, timeout)
        self.assertLess(timeout, success)

    def test_unsafe_terminal_penalty_exceeds_timeout_penalty(self) -> None:
        weights = DebugRewardWeights()
        common = dict(
            dt_s=0.1,
            previous_airspeed_mps=10.0,
            airspeed_mps=10.0,
            previous_lambda_executed=0.5,
            lambda_executed=0.5,
            altitude_error_m=0.0,
            pitch_rate_rps=0.0,
            alpha_rad=0.0,
            lambda_delta=0.0,
            success=False,
            failure=True,
            weights=weights,
        )
        timeout = common_reward_terms(**common).terminal
        unsafe = common_reward_terms(**common, unsafe_failure=True).terminal
        self.assertEqual(timeout, -weights.failure_penalty)
        self.assertEqual(unsafe, -weights.unsafe_failure_penalty)
        self.assertLess(unsafe, timeout)

    def test_task_terminal_uses_zero_absorbing_potential(self) -> None:
        weights = DebugRewardWeights()
        terms = common_reward_terms(
            dt_s=0.1,
            previous_airspeed_mps=15.0,
            airspeed_mps=15.0,
            previous_lambda_executed=0.95,
            lambda_executed=0.95,
            altitude_error_m=0.0,
            pitch_rate_rps=0.0,
            alpha_rad=0.0,
            lambda_delta=0.0,
            success=True,
            failure=False,
            weights=weights,
        )
        self.assertEqual(
            terms.progress_shaping,
            -transition_progress_potential(
                airspeed_mps=15.0,
                lambda_executed=0.95,
                weights=weights,
            ),
        )


if __name__ == "__main__":
    unittest.main()

import unittest

from ca_lsc_td3.safety.shield import (
    AsymmetricRateLimiter,
    apply_unloading_shield,
    hard_unloading_limit,
)


class ShieldTests(unittest.TestCase):
    def test_hard_limit_uses_most_restrictive_gate(self):
        self.assertAlmostEqual(hard_unloading_limit([0.9, 0.7, 0.8]), 0.7)

    def test_rate_limiter_unloads_slowly_and_recovers_faster(self):
        limiter = AsymmetricRateLimiter(
            unloading_rate_per_s=0.2,
            recovery_rate_per_s=1.0,
        )
        self.assertAlmostEqual(limiter.step(0.5, 1.0, 0.1), 0.52)
        self.assertAlmostEqual(limiter.step(0.5, 0.0, 0.1), 0.4)

    def test_shield_records_projection_and_rate_intervention(self):
        limiter = AsymmetricRateLimiter(
            unloading_rate_per_s=0.2,
            recovery_rate_per_s=1.0,
        )
        result = apply_unloading_shield(
            candidate=0.9,
            gates=[0.8, 0.6, 0.7],
            previous_executed=0.5,
            dt_s=0.1,
            rate_limiter=limiter,
        )
        self.assertAlmostEqual(result.hard_limit, 0.6)
        self.assertAlmostEqual(result.projected, 0.6)
        self.assertAlmostEqual(result.executed, 0.52)
        self.assertAlmostEqual(result.projection_intervention, 0.3)
        self.assertAlmostEqual(result.total_intervention, 0.38)

    def test_contracting_hard_envelope_overrides_recovery_rate(self):
        limiter = AsymmetricRateLimiter(
            unloading_rate_per_s=0.2,
            recovery_rate_per_s=1.0,
        )
        result = apply_unloading_shield(
            candidate=0.9,
            gates=[0.2, 0.8],
            previous_executed=0.8,
            dt_s=0.1,
            rate_limiter=limiter,
        )
        self.assertAlmostEqual(result.hard_limit, 0.2)
        self.assertAlmostEqual(result.executed, 0.2)
        self.assertLessEqual(result.executed, result.hard_limit)


if __name__ == "__main__":
    unittest.main()

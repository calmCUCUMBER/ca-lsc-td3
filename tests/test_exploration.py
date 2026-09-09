from __future__ import annotations

import unittest

import numpy as np

from ca_lsc_td3.rl.exploration import PiecewiseConstantExplorer


class PiecewiseConstantExplorerTests(unittest.TestCase):
    def test_uniform_action_is_held_until_simulation_deadline(self) -> None:
        explorer = PiecewiseConstantExplorer(
            np.random.default_rng(11),
            hold_s=5.0,
        )
        first = explorer.uniform_action(sim_time_s=10.0, shape=(1,))
        held = explorer.uniform_action(sim_time_s=14.9, shape=(1,))
        next_segment = explorer.uniform_action(sim_time_s=15.0, shape=(1,))

        np.testing.assert_array_equal(first, held)
        self.assertFalse(np.array_equal(first, next_segment))
        self.assertEqual(explorer.segment_count, 2)

    def test_source_change_starts_fresh_segment(self) -> None:
        explorer = PiecewiseConstantExplorer(
            np.random.default_rng(12),
            hold_s=5.0,
        )
        explorer.uniform_action(sim_time_s=1.0, shape=(1,))
        explorer.gaussian_noise(
            sim_time_s=1.1,
            shape=(1,),
            standard_deviation=0.1,
        )

        self.assertEqual(explorer.segment_count, 2)

    def test_clock_rollback_forces_resample(self) -> None:
        explorer = PiecewiseConstantExplorer(
            np.random.default_rng(13),
            hold_s=5.0,
        )
        before = explorer.uniform_action(sim_time_s=10.0, shape=(1,))
        after = explorer.uniform_action(sim_time_s=1.0, shape=(1,))

        self.assertFalse(np.array_equal(before, after))
        self.assertEqual(explorer.segment_count, 2)

    def test_zero_hold_preserves_iid_sampling(self) -> None:
        explorer = PiecewiseConstantExplorer(
            np.random.default_rng(14),
            hold_s=0.0,
        )
        first = explorer.uniform_action(sim_time_s=1.0, shape=(1,))
        second = explorer.uniform_action(sim_time_s=1.1, shape=(1,))

        self.assertFalse(np.array_equal(first, second))
        self.assertEqual(explorer.segment_count, 2)


if __name__ == "__main__":
    unittest.main()

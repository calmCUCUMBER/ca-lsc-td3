import unittest

import numpy as np

from ca_lsc_td3.rl.observations import (
    A0_OBSERVATION_FIELDS,
    OBSERVATION_FIELDS,
    pack_a0_observation,
    pack_observation,
)


class ObservationTests(unittest.TestCase):
    def test_observation_contract_is_exactly_twelve_dimensional(self):
        values = {
            field: float(index)
            for index, field in enumerate(OBSERVATION_FIELDS)
        }
        observation = pack_observation(values)
        self.assertEqual(len(OBSERVATION_FIELDS), 12)
        self.assertEqual(observation.shape, (12,))
        self.assertEqual(observation.dtype, np.float32)

    def test_missing_observation_field_is_rejected(self):
        values = {field: 0.0 for field in OBSERVATION_FIELDS[:-1]}
        with self.assertRaises(KeyError):
            pack_observation(values)

    def test_a0_observation_contract_is_exactly_ten_dimensional(self):
        self.assertEqual(len(A0_OBSERVATION_FIELDS), 10)
        self.assertNotIn("eta_l", A0_OBSERVATION_FIELDS)
        self.assertNotIn("eta_c", A0_OBSERVATION_FIELDS)
        values = {
            field: float(index)
            for index, field in enumerate(A0_OBSERVATION_FIELDS)
        }
        observation = pack_a0_observation(values)
        self.assertEqual(observation.shape, (10,))
        self.assertEqual(observation.dtype, np.float32)

    def test_a0_missing_observation_field_is_rejected(self):
        values = {field: 0.0 for field in A0_OBSERVATION_FIELDS[:-1]}
        with self.assertRaises(KeyError):
            pack_a0_observation(values)


if __name__ == "__main__":
    unittest.main()

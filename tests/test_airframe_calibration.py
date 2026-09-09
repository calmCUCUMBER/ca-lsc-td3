import json
import math
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

from ca_lsc_td3.physics.airframe_calibration import (
    parse_standard_vtol_sdf,
    write_calibration,
)


PROJECT = Path(__file__).resolve().parents[1]
MODEL = PROJECT / 'PX4-Autopilot/Tools/simulation/gz/models/standard_vtol/model.sdf'


class AirframeCalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.calibration = parse_standard_vtol_sdf(MODEL)

    def test_stock_airframe_constants_are_extracted(self):
        self.assertAlmostEqual(self.calibration.mass_kg, 5.025, places=6)
        self.assertAlmostEqual(self.calibration.wing_area_m2, 1.0)
        self.assertEqual(len(self.calibration.lift_rotors), 4)
        self.assertAlmostEqual(self.calibration.pusher.max_omega_rad_s, 3500.0)
        self.assertAlmostEqual(
            self.calibration.elevator_moment_derivative_per_dynamic_pressure,
            -0.06,
        )

    def test_sdf_a0_is_added_to_geometric_angle(self):
        wing = self.calibration.main_wings[0]
        self.assertGreater(wing.alpha_offset_rad, 0.0)
        self.assertAlmostEqual(
            wing.lift_coefficient(0.0),
            wing.lift_slope_per_rad * wing.alpha_offset_rad,
        )
        self.assertAlmostEqual(
            wing.lift_coefficient(-wing.alpha_offset_rad),
            0.0,
            places=12,
        )

    def test_hover_solution_and_power_are_finite(self):
        hover_omega = math.sqrt(
            self.calibration.mass_kg * 9.80665
            / sum(item.thrust_coefficient for item in self.calibration.lift_rotors)
        )
        self.assertLess(hover_omega, 1500.0)
        power = sum(
            item.ideal_mechanical_power_w(hover_omega)
            for item in self.calibration.lift_rotors
        )
        self.assertGreater(power, 0.0)

    def test_artifacts_are_written(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            summary = write_calibration(self.calibration, output)
            self.assertTrue(summary['checks']['test_5_model_calibration_pass'])
            parsed = json.loads(
                (output / 'airframe_calibration.json').read_text(encoding='utf-8')
            )
            self.assertEqual(parsed['esc_speed_unit'],
                             'rad_s_despite_esc_rpm_field_name_in_gz_sitl')
            self.assertEqual(
                parsed['lift_drag_a0_semantics'],
                'alpha_plugin_rad=alpha_geometric_rad+a0_rad',
            )
            self.assertIn('alpha_offset_rad', parsed['main_wings'][0])
            self.assertTrue((output / 'cl_alpha.csv').exists())
            self.assertTrue((output / 'elevator_authority.csv').exists())
            self.assertTrue((output / 'rotor_curves.csv').exists())

    def test_insufficient_rotor_authority_fails_model_check(self):
        weak_rotors = tuple(
            replace(rotor, max_omega_rad_s=100.0)
            for rotor in self.calibration.rotors
        )
        weak_calibration = replace(self.calibration, rotors=weak_rotors)
        with tempfile.TemporaryDirectory() as directory:
            summary = write_calibration(weak_calibration, Path(directory))
        self.assertFalse(summary['checks']['hover_omega_below_lift_limit'])
        self.assertLess(summary['checks']['lift_thrust_margin_over_weight'], 1.0)
        self.assertFalse(summary['checks']['test_5_model_calibration_pass'])


if __name__ == '__main__':
    unittest.main()

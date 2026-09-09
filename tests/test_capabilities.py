import math
import unittest

from ca_lsc_td3.physics.capabilities import (
    aerodynamic_control_authority,
    aerodynamic_control_authority_from_dimensional_derivative,
    estimate_wing_vertical_support,
    piecewise_lift_coefficient,
    physics_prior,
    shadow_pitch_moment_increment,
    smoothstep,
    wing_vertical_support_capability,
)


class CapabilityTests(unittest.TestCase):
    def test_piecewise_lift_coefficient_matches_offset_and_post_stall_law(self):
        self.assertAlmostEqual(
            piecewise_lift_coefficient(
                geometric_alpha_rad=0.1,
                alpha_offset_rad=0.05,
                lift_slope_per_rad=4.0,
                stall_angle_rad=0.3,
                post_stall_slope_per_rad=-2.0,
            ),
            0.6,
        )
        self.assertAlmostEqual(
            piecewise_lift_coefficient(
                geometric_alpha_rad=0.35,
                alpha_offset_rad=0.05,
                lift_slope_per_rad=4.0,
                stall_angle_rad=0.3,
                post_stall_slope_per_rad=-2.0,
            ),
            1.0,
        )

    def test_smoothstep_has_expected_limits_and_midpoint(self):
        self.assertEqual(smoothstep(-1.0, 0.0, 1.0), 0.0)
        self.assertEqual(smoothstep(2.0, 0.0, 1.0), 1.0)
        self.assertAlmostEqual(smoothstep(0.5, 0.0, 1.0), 0.5)

    def test_eta_l_is_zero_without_airspeed_and_saturates_at_one(self):
        common = dict(
            air_density_kg_m3=1.225,
            wing_area_m2=1.0,
            lift_coefficient=1.0,
            flight_path_angle_rad=0.0,
            roll_angle_rad=0.0,
            estimated_mass_kg=5.0,
        )
        self.assertEqual(
            wing_vertical_support_capability(airspeed_mps=0.0, **common),
            0.0,
        )
        self.assertEqual(
            wing_vertical_support_capability(airspeed_mps=100.0, **common),
            1.0,
        )

    def test_eta_l_uses_estimated_mass_and_attitude_projection(self):
        level = wing_vertical_support_capability(
            air_density_kg_m3=1.225,
            airspeed_mps=12.0,
            wing_area_m2=1.0,
            lift_coefficient=0.6,
            flight_path_angle_rad=0.0,
            roll_angle_rad=0.0,
            estimated_mass_kg=10.0,
        )
        banked = wing_vertical_support_capability(
            air_density_kg_m3=1.225,
            airspeed_mps=12.0,
            wing_area_m2=1.0,
            lift_coefficient=0.6,
            flight_path_angle_rad=0.0,
            roll_angle_rad=math.radians(45.0),
            estimated_mass_kg=10.0,
        )
        self.assertLess(banked, level)

    def test_eta_l_detailed_result_exposes_dimensional_terms(self):
        estimate = estimate_wing_vertical_support(
            air_density_kg_m3=1.2,
            airspeed_mps=10.0,
            wing_area_m2=1.0,
            lift_coefficient=0.5,
            flight_path_angle_rad=0.0,
            roll_angle_rad=0.0,
            estimated_mass_kg=5.0,
            gravity_mps2=10.0,
        )
        self.assertAlmostEqual(estimate.dynamic_pressure_pa, 60.0)
        self.assertAlmostEqual(estimate.wing_lift_n, 30.0)
        self.assertAlmostEqual(estimate.vertical_support_n, 30.0)
        self.assertAlmostEqual(estimate.required_support_n, 50.0)
        self.assertAlmostEqual(estimate.eta_l, 0.6)

    def test_eta_l_is_conservatively_gated_below_valid_aoa_speed(self):
        eta_l = wing_vertical_support_capability(
            air_density_kg_m3=1.225,
            airspeed_mps=4.99,
            wing_area_m2=1.0,
            lift_coefficient=10.0,
            flight_path_angle_rad=0.0,
            roll_angle_rad=0.0,
            estimated_mass_kg=5.0,
            minimum_valid_airspeed_mps=5.0,
        )
        self.assertEqual(eta_l, 0.0)

    def test_eta_c_is_gated_to_zero_at_zero_dynamic_pressure(self):
        eta_c = aerodynamic_control_authority(
            dynamic_pressure_pa=0.0,
            wing_area_m2=1.0,
            mean_aerodynamic_chord_m=0.5,
            pitch_moment_derivative_per_rad=-1.0,
            elevator_angle_rad=0.0,
            elevator_min_rad=-0.5,
            elevator_max_rad=0.5,
            shadow_pitch_moment_increment_nm=0.0,
            pressure_gate_lower_pa=10.0,
            pressure_gate_upper_pa=100.0,
        )
        self.assertEqual(eta_c, 0.0)

    def test_eta_c_uses_directional_remaining_elevator_margin(self):
        common = dict(
            dynamic_pressure_pa=200.0,
            wing_area_m2=1.0,
            mean_aerodynamic_chord_m=0.5,
            pitch_moment_derivative_per_rad=-1.0,
            elevator_angle_rad=0.4,
            elevator_min_rad=-0.5,
            elevator_max_rad=0.5,
            pressure_gate_lower_pa=10.0,
            pressure_gate_upper_pa=100.0,
        )
        positive_demand = aerodynamic_control_authority(
            shadow_pitch_moment_increment_nm=20.0,
            **common,
        )
        negative_demand = aerodynamic_control_authority(
            shadow_pitch_moment_increment_nm=-20.0,
            **common,
        )
        # Cm_delta_e < 0: positive moment requires motion toward delta_min,
        # where this elevator has substantially more remaining travel.
        self.assertGreater(positive_demand, negative_demand)

        positive_derivative_common = {
            **common,
            "pitch_moment_derivative_per_rad": 1.0,
        }
        positive_demand = aerodynamic_control_authority(
            shadow_pitch_moment_increment_nm=20.0,
            **positive_derivative_common,
        )
        negative_demand = aerodynamic_control_authority(
            shadow_pitch_moment_increment_nm=-20.0,
            **positive_derivative_common,
        )
        self.assertLess(positive_demand, negative_demand)

    def test_eta_c_is_zero_when_the_elevator_has_no_effectiveness(self):
        eta_c = aerodynamic_control_authority(
            dynamic_pressure_pa=200.0,
            wing_area_m2=1.0,
            mean_aerodynamic_chord_m=0.5,
            pitch_moment_derivative_per_rad=0.0,
            elevator_angle_rad=0.0,
            elevator_min_rad=-0.5,
            elevator_max_rad=0.5,
            shadow_pitch_moment_increment_nm=0.0,
            pressure_gate_lower_pa=10.0,
            pressure_gate_upper_pa=100.0,
        )
        self.assertEqual(eta_c, 0.0)

    def test_shadow_demand_uses_increment_from_actual_elevator_position(self):
        shadow = shadow_pitch_moment_increment(
            dynamic_pressure_pa=100.0,
            dimensional_moment_derivative_per_q_m3=-0.06,
            normalized_shadow_pitch_demand=-0.4,
            current_elevator_angle_rad=-0.1,
            elevator_min_rad=-0.53,
            elevator_max_rad=0.53,
        )
        self.assertAlmostEqual(shadow.target_elevator_angle_rad, -0.4)
        self.assertAlmostEqual(shadow.target_pitch_moment_nm, 2.4)
        self.assertAlmostEqual(shadow.current_pitch_moment_nm, 0.6)
        self.assertAlmostEqual(shadow.increment_pitch_moment_nm, 1.8)

    def test_dimensional_eta_c_respects_negative_derivative_direction(self):
        common = dict(
            dynamic_pressure_pa=100.0,
            dimensional_moment_derivative_per_q_m3=-0.06,
            elevator_min_rad=-0.53,
            elevator_max_rad=0.53,
            shadow_pitch_moment_increment_nm=0.5,
            pressure_gate_lower_pa=10.0,
            pressure_gate_upper_pa=50.0,
        )
        near_helpful_limit = aerodynamic_control_authority_from_dimensional_derivative(
            elevator_angle_rad=-0.5, **common
        )
        far_from_helpful_limit = aerodynamic_control_authority_from_dimensional_derivative(
            elevator_angle_rad=0.5, **common
        )
        self.assertLess(near_helpful_limit.eta_c, far_from_helpful_limit.eta_c)

    def test_invalid_capability_gates_are_rejected(self):
        with self.assertRaises(ValueError):
            physics_prior(
                0.5,
                0.5,
                eta_l_lower=-0.1,
                eta_l_upper=0.8,
                eta_c_lower=0.2,
                eta_c_upper=0.8,
            )
        with self.assertRaises(ValueError):
            aerodynamic_control_authority(
                dynamic_pressure_pa=200.0,
                wing_area_m2=1.0,
                mean_aerodynamic_chord_m=0.5,
                pitch_moment_derivative_per_rad=-1.0,
                elevator_angle_rad=0.0,
                elevator_min_rad=-0.5,
                elevator_max_rad=0.5,
                shadow_pitch_moment_increment_nm=0.0,
                pressure_gate_lower_pa=-1.0,
                pressure_gate_upper_pa=100.0,
            )

    def test_physics_prior_requires_both_capabilities(self):
        thresholds = dict(
            eta_l_lower=0.2,
            eta_l_upper=0.8,
            eta_c_lower=0.2,
            eta_c_upper=0.8,
        )
        self.assertEqual(physics_prior(1.0, 0.0, **thresholds), 0.0)
        self.assertEqual(physics_prior(0.0, 1.0, **thresholds), 0.0)
        self.assertEqual(physics_prior(1.0, 1.0, **thresholds), 1.0)


if __name__ == "__main__":
    unittest.main()

import json
import math
import tempfile
import unittest
from pathlib import Path

from ca_lsc_td3.evaluation.f1_grid import (
    _eta_l_proxy_values,
    _main_wing_cl,
    _percentile,
    _power_sanity,
    aggregate,
    aggregate_roots,
)
from ca_lsc_td3.physics.capabilities import wing_vertical_support_capability


def _write_point(
    root: Path,
    name: str,
    runs: list[dict[str, object]],
) -> dict[str, object]:
    point = root / name
    point.mkdir(parents=True)
    for index, payload in enumerate(runs, start=1):
        run_dir = point / f'run_{index}'
        run_dir.mkdir()
        summary = {
            'source_csv': str(run_dir / 'telemetry.csv'),
            'va_target_config': 12.0,
            'lambda_target_config': 0.3,
            'phase_0_75_va_hold_pass': False,
            'va_hold_measurement_complete': False,
            'primary_failure_cause': 'none',
            'va_hold_abort_reason': 'none',
            'va_hold_measurement_duration_s': None,
        }
        summary.update(payload)
        (run_dir / 'summary.json').write_text(
            json.dumps(summary, allow_nan=False) + '\n',
            encoding='utf-8',
        )
    repeat = {
        'passes': sum(
            bool(run.get('phase_0_75_va_hold_pass')) for run in runs
        ),
        'total': len(runs),
    }
    (point / 'repeat_summary.json').write_text(
        json.dumps(repeat, allow_nan=False) + '\n',
        encoding='utf-8',
    )
    result = aggregate(root, calibration_path=None)
    return next(item for item in result['points'] if item['point_dir'] == name)


class F1GridAggregationTests(unittest.TestCase):
    def test_eta_l_uses_selected_airspeed_and_actual_run_mass(self):
        calibration = {
            'air_density_kg_m3': 1.2,
            'mass_kg': 5.0,
            'wing_area_m2': 1.0,
            'main_wings': [{
                'area_m2': 1.0,
                'alpha_offset_rad': 0.0,
                'lift_slope_per_rad': 4.0,
                'stall_angle_rad': 0.3,
                'post_stall_slope_per_rad': -2.0,
            }],
        }
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / 'telemetry.csv'
            csv_path.write_text(
                'va_hold_measurement_active,va_hold_measurement_complete,'
                'selected_airspeed_mps,airspeed_mps,alpha_valid,'
                'alpha_est_rad,gamma_est_rad,roll_rad,'
                'actual_model_mass_kg_config\n'
                '1,0,6,60,1,0.1,0,0,12\n',
                encoding='utf-8',
            )
            summary = {
                'source_csv': str(csv_path),
                'actual_model_mass_kg_config': 12.0,
            }
            values = _eta_l_proxy_values([summary], calibration)

        cl = _main_wing_cl(calibration, 0.1)
        expected = wing_vertical_support_capability(
            air_density_kg_m3=1.2,
            airspeed_mps=6.0,
            wing_area_m2=1.0,
            lift_coefficient=cl,
            flight_path_angle_rad=0.0,
            roll_angle_rad=0.0,
            estimated_mass_kg=12.0,
        )
        self.assertEqual(len(values), 1)
        self.assertAlmostEqual(values[0], expected)

    def test_eta_l_falls_back_for_frozen_pre_schema_14_data(self):
        calibration = {
            'air_density_kg_m3': 1.2,
            'mass_kg': 5.0,
            'wing_area_m2': 1.0,
            'main_wings': [{
                'area_m2': 1.0,
                'alpha_offset_rad': 0.0,
                'lift_slope_per_rad': 4.0,
                'stall_angle_rad': 0.3,
                'post_stall_slope_per_rad': -2.0,
            }],
        }
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / 'telemetry.csv'
            csv_path.write_text(
                'va_hold_measurement_active,va_hold_measurement_complete,'
                'airspeed_mps,alpha_valid,alpha_est_rad,gamma_est_rad,'
                'roll_rad\n1,0,6,1,0.1,0,0\n',
                encoding='utf-8',
            )
            values = _eta_l_proxy_values(
                [{'source_csv': str(csv_path)}], calibration
            )
        self.assertEqual(len(values), 1)
        self.assertTrue(math.isfinite(values[0]))

    def test_extracts_feasible_boundary_and_energy_optimum(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for lam, power, classification_pass in (
                (0.0, 500.0, True),
                (0.1, 420.0, True),
                (0.2, 450.0, True),
                (0.3, 300.0, False),
            ):
                run = {
                    'telemetry_schema_version': 7,
                    'f1_measurement_pass': classification_pass,
                    'va_hold_measurement_complete': classification_pass,
                    'va_hold_measurement_duration_s': 3.0 if classification_pass else None,
                    'va_hold_mean_total_power_w': power,
                    'va_hold_altitude_rmse_m': 0.2,
                    'va_hold_max_abs_altitude_error_m': 0.3,
                    'va_hold_max_abs_vertical_speed_mps': 0.1,
                    'lambda_target_config': lam,
                    **{
                        'vt_unload_altitude_pitch_kp_config': 2.0,
                        'vt_unload_vertical_speed_pitch_kd_config': 2.0,
                        'vt_unload_pitch_min_deg_config': -12.0,
                        'vt_unload_pitch_max_deg_config': 10.0,
                    },
                }
                _write_point(root, f'va_12_lambda_{lam}', [run] * 5)
            result = aggregate(root, calibration_path=None)
            row = result['derived_by_airspeed'][0]
            self.assertAlmostEqual(row['lambda_max_feasible'], 0.2)
            self.assertTrue(math.isnan(row['lambda_energy_optimal']))
            self.assertFalse(row['energy_optimum_reportable'])
            self.assertAlmostEqual(row['lambda_energy_optimal_proxy'], 0.1)
            self.assertAlmostEqual(
                row['minimum_feasible_total_power_proxy_w'], 420.0
            )
            self.assertFalse(row['lambda_grid_has_0p1_resolution'])
            self.assertTrue(result['protocol_consistent'])

    def test_f1_specific_pass_overrides_phase_0_75_diagnostic_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = {
                'f1_measurement_pass': True,
                'phase_0_75_va_hold_pass': False,
                'va_hold_measurement_complete': True,
                'va_hold_measurement_duration_s': 3.0,
                'va_hold_altitude_rmse_m': 0.2,
                'va_hold_max_abs_altitude_error_m': 0.3,
                'va_hold_max_abs_vertical_speed_mps': 0.1,
            }
            point = _write_point(
                root,
                'va_6_lambda_0p3',
                [run, run, run, run, run],
            )

            self.assertEqual(point['passes'], 5)
            self.assertEqual(point['valid_f1_passes'], 5)
            self.assertEqual(point['classification'], 'nominal_feasible')

    def test_majority_pre_measurement_transient_is_not_unsafe_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pass_run = {
                'phase_0_75_va_hold_pass': True,
                'va_hold_measurement_complete': True,
                'va_hold_measurement_duration_s': 3.0,
                'va_hold_altitude_rmse_m': 0.2,
                'va_hold_max_abs_altitude_error_m': 0.3,
                'va_hold_max_abs_vertical_speed_mps': 0.1,
            }
            transient_run = {
                'primary_failure_cause': 'pre_measurement_altitude_transient',
                'va_hold_abort_reason': 'va_hold_altitude_runaway',
                'va_hold_pre_measurement_altitude_transient': True,
            }
            point = _write_point(
                root,
                'va_12_lambda_0p3',
                [pass_run, pass_run, transient_run, transient_run, transient_run],
            )

            self.assertEqual(point['classification'], 'protocol_invalid')
            self.assertEqual(point['outcome_class'], 'protocol_invalid')
            self.assertEqual(point['pre_measurement_transient_count'], 3)
            self.assertEqual(point['valid_f1_total'], 2)
            self.assertEqual(point['valid_f1_passes'], 2)

    def test_single_pre_measurement_transient_marks_point_protocol_invalid(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pass_run = {
                'phase_0_75_va_hold_pass': True,
                'va_hold_measurement_complete': True,
                'va_hold_measurement_duration_s': 3.0,
                'va_hold_altitude_rmse_m': 0.4,
                'va_hold_max_abs_altitude_error_m': 0.6,
                'va_hold_max_abs_vertical_speed_mps': 0.2,
                'va_hold_max_abs_alpha_rad': 0.02,
                'va_hold_pitch_rate_p95_rad_s': 0.05,
                'va_hold_servo_saturation_fraction': 0.0,
                'lift_rotor_spike_samples': 0,
            }
            transient_run = {
                'primary_failure_cause': 'pre_measurement_altitude_transient',
                'va_hold_abort_reason': 'va_hold_altitude_runaway',
                'va_hold_pre_measurement_altitude_transient': True,
            }
            point = _write_point(
                root,
                'va_14_lambda_0p7',
                [pass_run, pass_run, pass_run, pass_run, transient_run],
            )

            self.assertEqual(point['classification'], 'protocol_invalid')
            self.assertEqual(point['physical_class'], 'unknown')
            self.assertEqual(point['data_quality'], 'protocol_unstable')
            self.assertIn('pre_measurement_altitude_transient',
                          point['data_quality_reasons'])
            self.assertEqual(point['pre_measurement_transient_count'], 1)
            self.assertEqual(point['valid_f1_total'], 4)
            self.assertEqual(point['valid_f1_pass_rate'], 1.0)

    def test_missing_repeat_set_is_unknown_not_nominal(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = {
                'f1_measurement_pass': True,
                'va_hold_measurement_complete': True,
                'va_hold_altitude_rmse_m': 0.2,
            }
            point = _write_point(root, 'partial', [run, run, run])
            self.assertEqual(point['physical_class'], 'unknown')
            self.assertEqual(point['data_quality'], 'missing')

    def test_single_soft_outlier_is_recorded_but_not_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runs = []
            for value in (0.6, 0.64, 0.68, 0.70, 2.1):
                runs.append({
                    'f1_measurement_pass': True,
                    'va_hold_measurement_complete': True,
                    'va_hold_max_abs_altitude_error_m': value,
                    'va_hold_altitude_rmse_m': 0.2,
                })
            point = _write_point(root, 'single_outlier', runs)
            self.assertEqual(point['physical_class'], 'nominal_feasible')
            self.assertIn(
                'max_altitude_error', point['single_run_outlier_reasons']
            )
            self.assertAlmostEqual(
                point['va_hold_max_abs_altitude_error_m_median'], 0.68
            )
            self.assertAlmostEqual(
                point['va_hold_max_abs_altitude_error_m_p90'], 1.54
            )
            self.assertAlmostEqual(
                point['va_hold_max_abs_altitude_error_m_p95'], 1.82
            )

    def test_repeated_soft_exceedance_is_boundary_with_reason(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runs = [{
                'f1_measurement_pass': True,
                'va_hold_measurement_complete': True,
                'va_hold_altitude_rmse_m': value,
            } for value in (0.2, 0.3, 0.4, 1.2, 1.4)]
            point = _write_point(root, 'repeated_boundary', runs)
            self.assertEqual(point['physical_class'], 'boundary_feasible')
            self.assertEqual(point['boundary_reasons'], 'altitude_rmse')

    def test_physical_abort_and_protocol_failure_are_separated(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pass_run = {
                'f1_measurement_pass': True,
                'va_hold_measurement_complete': True,
            }
            physical = {
                'primary_failure_cause': 'va_hold_altitude_runaway',
                'va_hold_abort_reason': 'va_hold_altitude_runaway',
                'lambda_target_reached': True,
                'maximum_transition_airspeed_mps': 12.0,
                'va_target_tolerance': 0.5,
            }
            point = _write_point(
                root, 'physical_abort',
                [pass_run, pass_run, pass_run, pass_run, physical],
            )
            self.assertEqual(point['physical_class'], 'boundary_feasible')
            self.assertEqual(point['outcome_class'], 'boundary_feasible')
            self.assertEqual(point['unsafe_reasons'], 'altitude_runaway')
            self.assertEqual(point['unsafe_repeatability'], 'single_run')

            protocol = {
                'primary_failure_cause': 'sequence_or_telemetry_incomplete',
            }
            point = _write_point(
                root, 'protocol_failure',
                [pass_run, pass_run, protocol, protocol, protocol],
            )
            self.assertEqual(point['physical_class'], 'unknown')
            self.assertEqual(point['data_quality'], 'protocol_unstable')

    def test_repeated_target_directed_ramp_abort_is_physical_unsafe(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            physical = {
                'f1_measurement_pass': False,
                'telemetry_schema_version': 13,
                'primary_failure_cause': 'va_hold_altitude_runaway',
                'va_hold_abort_reason': 'va_hold_altitude_runaway',
                'lambda_target_reached': False,
                'lambda_target_config': 0.9,
                'lambda_target_tolerance': 0.03,
                'max_lambda_exec': 0.62,
                'lambda_external_active_fraction': 0.8,
                'maximum_transition_airspeed_mps': 10.1,
                'va_target_config': 10.0,
                'va_target_tolerance': 0.3,
                'transition_lambda_attitude_sync_samples': 25,
                'transition_lambda_attitude_sync_pass': True,
                'target_lambda_attitude_sync_pass': False,
            }
            point = _write_point(root, 'target_directed', [physical] * 5)
            self.assertEqual(point['physical_class'], 'physical_unsafe')
            self.assertEqual(point['outcome_class'], 'physical_unsafe')
            self.assertEqual(point['unsafe_run_count'], 5)
            self.assertEqual(point['data_quality'], 'valid')

    def test_target_condition_invalid_does_not_mask_directed_physical_abort(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            physical = {
                'f1_measurement_pass': False,
                'f1_attempt_quality': 'protocol_invalid',
                'f1_attempt_quality_reason': 'target_condition_not_established',
                'telemetry_schema_version': 13,
                'primary_failure_cause': 'va_hold_altitude_runaway',
                'va_hold_abort_reason': 'va_hold_altitude_runaway',
                'lambda_target_reached': False,
                'lambda_target_config': 1.0,
                'lambda_target_tolerance': 0.03,
                'max_lambda_exec': 0.95,
                'lambda_external_active_fraction': 0.9,
                'maximum_transition_airspeed_mps': 6.1,
                'va_target_config': 6.0,
                'va_target_tolerance': 0.3,
                'transition_lambda_attitude_sync_samples': 25,
                'transition_lambda_attitude_sync_pass': True,
            }
            point = _write_point(root, 'legacy_directed_abort', [physical] * 5)
            self.assertEqual(point['physical_class'], 'physical_unsafe')
            self.assertEqual(point['data_quality'], 'valid')

    def test_short_target_sync_sample_does_not_mask_ramp_abort(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            physical = {
                'f1_measurement_pass': False,
                'f1_attempt_quality': 'valid',
                'telemetry_schema_version': 13,
                'primary_failure_cause': 'va_hold_vertical_speed_runaway',
                'va_hold_abort_reason': 'va_hold_vertical_speed_runaway',
                'f1_target_condition_established': True,
                'f1_target_directed_ramp_established': True,
                'lambda_target_reached': True,
                'lambda_target_config': 1.0,
                'lambda_target_tolerance': 0.03,
                'max_lambda_exec': 1.0,
                'lambda_external_active_fraction': 0.9,
                'maximum_transition_airspeed_mps': 8.3,
                'va_target_config': 8.0,
                'va_target_tolerance': 0.3,
                'transition_lambda_attitude_sync_samples': 200,
                'transition_lambda_attitude_sync_pass': True,
                'target_lambda_attitude_sync_samples': 5,
                'target_lambda_attitude_sync_pass': False,
            }
            point = _write_point(root, 'short_target_sync', [physical] * 5)
            self.assertEqual(point['physical_class'], 'physical_unsafe')
            self.assertEqual(point['data_quality'], 'valid')
            self.assertEqual(point['unsafe_reasons'], 'vertical_speed_runaway')

    def test_protocol_float_noise_does_not_break_consistency(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            values = (0.09999999999999999, 0.1, 0.10000000000000002)
            for index, value in enumerate(values):
                run = {
                    'telemetry_schema_version': 13,
                    'f1_measurement_pass': True,
                    'va_hold_measurement_complete': True,
                    'lambda_target_config': 0.3,
                    'lambda_attitude_blend_start_config': value,
                    'lambda_attitude_blend_full_config': 0.9,
                    'va_target_dwell_s_config': 2.0,
                    'lambda_target_dwell_s_config': 2.0,
                    'va_measurement_duration_s_config': 3.0,
                    'va_hold_measurement_min_band_fraction_config': 0.9,
                    'va_hold_measurement_max_abs_airspeed_error_mps_config': 0.6,
                    'vt_unload_altitude_pitch_kp_config': 2.0,
                    'vt_unload_vertical_speed_pitch_kd_config': 2.0,
                    'vt_unload_pitch_min_deg_config': -12.0,
                    'vt_unload_pitch_max_deg_config': 10.0,
                }
                _write_point(root, f'point_{index}', [run] * 5)
            result = aggregate(root, calibration_path=None)
            self.assertTrue(result['protocol_consistent'])
            self.assertEqual(len(result['protocol_configurations']), 1)

    def test_five_valid_runs_with_one_invalid_attempt_remain_classifiable(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            good = {
                'f1_measurement_pass': True,
                'va_hold_measurement_complete': True,
                'va_hold_altitude_rmse_m': 0.2,
            }
            invalid = {
                'f1_attempt_quality': 'protocol_invalid',
                'f1_attempt_quality_reason': (
                    'pre_measurement_altitude_transient'
                ),
                'primary_failure_cause': (
                    'pre_measurement_altitude_transient'
                ),
                'va_hold_pre_measurement_altitude_transient': True,
            }
            point = _write_point(
                root, 'audited_retry',
                [good, good, invalid, good, good, good],
            )

            self.assertEqual(point['physical_class'], 'nominal_feasible')
            self.assertEqual(point['data_quality'], 'valid_with_retries')
            self.assertEqual(point['valid_f1_total'], 5)
            self.assertEqual(point['attempt_count'], 6)
            self.assertEqual(point['protocol_invalid_run_count'], 1)
            self.assertAlmostEqual(
                point['protocol_invalid_attempt_rate'], 1 / 6
            )

    def test_protocol_invalid_attempt_cannot_mask_repeated_physical_abort(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            physical = {
                'f1_measurement_pass': False,
                'primary_failure_cause': 'va_hold_altitude_runaway',
                'va_hold_abort_reason': 'va_hold_altitude_runaway',
                'lambda_target_reached': True,
                'maximum_transition_airspeed_mps': 12.0,
                'va_target_tolerance': 0.5,
            }
            invalid = {
                'f1_attempt_quality': 'protocol_invalid',
                'f1_attempt_quality_reason': (
                    'pre_measurement_altitude_transient'
                ),
                'primary_failure_cause': (
                    'pre_measurement_altitude_transient'
                ),
                'va_hold_pre_measurement_altitude_transient': True,
            }
            point = _write_point(
                root, 'unsafe_with_retry',
                [physical, physical, invalid, physical, physical, physical],
            )

            self.assertEqual(point['physical_class'], 'physical_unsafe')
            self.assertEqual(point['data_quality'], 'valid_with_retries')
            self.assertEqual(point['unsafe_run_count'], 5)
            self.assertEqual(point['protocol_invalid_run_count'], 1)

    def test_abort_before_lambda_execution_is_protocol_invalid_not_unsafe(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            good = {
                'f1_measurement_pass': True,
                'va_hold_measurement_complete': True,
                'va_hold_altitude_rmse_m': 2.0,
            }
            precondition_abort = {
                'primary_failure_cause': 'va_hold_altitude_runaway',
                'va_hold_abort_reason': 'va_hold_altitude_runaway',
                'lambda_target_reached': False,
                'max_lambda_exec': 0.0,
                'maximum_transition_airspeed_mps': 5.1,
                'va_target_tolerance': 0.5,
            }
            point = _write_point(
                root, 'precondition_abort',
                [good, good, good, good, precondition_abort],
            )
            self.assertEqual(point['physical_class'], 'unknown')
            self.assertEqual(point['outcome_class'], 'protocol_invalid')
            self.assertEqual(point['unsafe_run_count'], 0)
            self.assertEqual(point['protocol_invalid_run_count'], 1)
            self.assertIn('target_condition_not_established',
                          point['data_quality_reasons'])

    def test_repeated_settle_timeout_is_nonconvergent_not_unsafe(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            failed = {
                'f1_measurement_pass': False,
                'primary_failure_cause': 'va_hold_settle_timeout',
                'lambda_target_reached': True,
                'maximum_transition_airspeed_mps': 12.0,
                'va_target_tolerance': 0.5,
            }
            point = _write_point(root, 'nonconvergent', [failed] * 5)
            self.assertEqual(point['physical_class'], 'nonconvergent')
            self.assertEqual(point['outcome_class'], 'nonconvergent')
            self.assertEqual(point['unsafe_run_count'], 0)
            self.assertEqual(point['nonconvergent_run_count'], 5)

    def test_fragmented_measurement_is_not_eligible_physical_statistics(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fragmented = {
                'f1_measurement_pass': True,
                'va_hold_measurement_complete': True,
                'va_hold_measurement_segment_count': 3,
                'va_hold_altitude_rmse_m': 999.0,
            }
            point = _write_point(root, 'fragmented', [fragmented] * 5)
            self.assertEqual(point['physical_class'], 'nonconvergent')
            self.assertEqual(point['measurement_eligible_run_count'], 0)
            self.assertTrue(math.isnan(point['va_hold_altitude_rmse_m_mean']))

    def test_transient_metric_is_excluded_from_physical_statistics(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            good = {
                'f1_measurement_pass': True,
                'va_hold_measurement_complete': True,
                'va_hold_altitude_rmse_m': 1.0,
            }
            transient = {
                'primary_failure_cause': 'pre_measurement_altitude_transient',
                'va_hold_pre_measurement_altitude_transient': True,
                'va_hold_altitude_rmse_m': 999.0,
            }
            point = _write_point(
                root, 'transient_excluded',
                [good, good, good, good, transient],
            )
            self.assertAlmostEqual(point['va_hold_altitude_rmse_m_mean'], 1.0)
            self.assertAlmostEqual(point['va_hold_altitude_rmse_m_p95'], 1.0)

    def test_percentile_definition_is_linear_n_minus_one(self):
        values = [1.0, 2.0, 3.0, 4.0, 5.0]
        self.assertAlmostEqual(_percentile(values, 0.5), 3.0)
        self.assertAlmostEqual(_percentile(values, 0.9), 4.6)
        self.assertAlmostEqual(_percentile(values, 0.95), 4.8)

    def test_right_censored_boundary_is_displayed_as_lower_bound(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for lam in (0.0, 0.5, 0.9):
                run = {
                    'f1_measurement_pass': True,
                    'va_hold_measurement_complete': True,
                    'lambda_target_config': lam,
                }
                _write_point(root, f'lambda_{lam}', [run] * 5)
            row = aggregate(root, None)['derived_by_airspeed'][0]
            self.assertAlmostEqual(row['lambda_max_feasible'], 0.9)
            self.assertTrue(row['lambda_max_feasible_is_lower_bound'])
            self.assertEqual(row['lambda_max_feasible_display'], '>=0.9')

    def test_power_sanity_rejects_apparent_efficiency_above_one(self):
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory)
            csv_path = run_dir / 'telemetry.csv'
            rows = [
                'time_s,va_hold_measurement_active,va_hold_measurement_complete,'
                'airspeed_mps,alpha_valid,alpha_est_rad,pusher_thrust_n,'
                'pusher_power_w,lift_power_w',
            ]
            for index in range(40):
                rows.append(f'{index/20},1,0,20,1,0,8,80,1')
            csv_path.write_text('\n'.join(rows) + '\n', encoding='utf-8')
            result = _power_sanity([{'source_csv': str(csv_path)}])
            self.assertFalse(result['pusher_power_sanity_pass'])
            self.assertEqual(
                result['pusher_power_sanity_status'],
                'model_inconsistent_at_forward_speed',
            )
            self.assertAlmostEqual(
                result['pusher_propulsive_efficiency_proxy_mean'], 2.0
            )

    def test_explicit_last_duplicate_policy_replaces_protocol_recheck(self):
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            old = parent / 'old'
            new = parent / 'new'
            transient = {
                'primary_failure_cause': 'pre_measurement_altitude_transient',
                'va_hold_pre_measurement_altitude_transient': True,
            }
            good = {
                'f1_measurement_pass': True,
                'va_hold_measurement_complete': True,
            }
            _write_point(old, 'point', [good, good, transient, transient,
                                         transient])
            _write_point(new, 'point', [good] * 5)
            with self.assertRaises(ValueError):
                aggregate_roots([old, new], None)
            result = aggregate_roots(
                [old, new], None, duplicate_policy='last'
            )
            self.assertEqual(result['point_count'], 1)
            self.assertEqual(result['points'][0]['physical_class'],
                             'nominal_feasible')
            self.assertIn('/new/', result['points'][0]['source_point_dir'])


if __name__ == '__main__':
    unittest.main()

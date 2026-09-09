import csv
import json
import tempfile
import unittest
from pathlib import Path

from ca_lsc_td3.evaluation.f5_dataset import (
    F5LabelConfig,
    build_f5_dataset,
)


FIELDS = [
    'schema_version', 'time_s', 'schedule_mode', 'va_hold_phase',
    'va_hold_measurement_active', 'va_hold_measurement_complete',
    'selected_airspeed_mps', 'lambda_exec', 'eta_l', 'eta_l_valid',
    'eta_c', 'eta_c_valid', 'alpha_est_rad', 'alpha_valid',
    'altitude_error_m', 'vz_up_mps', 'failsafe', 'va_hold_abort_reason',
    'condition_id', 'va_target_config', 'lambda_target_config',
    'mass_scale_config', 'actual_model_mass_kg_config',
    'gust_amplitude_mps_config', 'wind_model_config',
]


def _write_run(
    root: Path,
    name: str,
    *,
    unsafe_at: float | None = None,
    eta_c_valid: bool = True,
    duration: float = 4.0,
) -> None:
    run = root / name
    run.mkdir(parents=True)
    (run / 'summary.json').write_text(json.dumps({
        'telemetry_schema_version': 23,
        'va_target_config': 10.0,
        'lambda_target_config': 0.5,
    }), encoding='utf-8')
    with (run / 'telemetry.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        steps = int(duration / 0.1) + 1
        for index in range(steps):
            time_s = index * 0.1
            unsafe = unsafe_at is not None and time_s >= unsafe_at
            writer.writerow({
                'schema_version': 23,
                'time_s': time_s,
                'schedule_mode': 'va_hold_target',
                'va_hold_phase': 'measurement',
                'va_hold_measurement_active': 1,
                'va_hold_measurement_complete': 0,
                'selected_airspeed_mps': 10.0,
                'lambda_exec': 0.5,
                'eta_l': 0.55,
                'eta_l_valid': 1,
                'eta_c': 0.65,
                'eta_c_valid': int(eta_c_valid),
                'alpha_est_rad': 0.1,
                'alpha_valid': 1,
                'altitude_error_m': 6.0 if unsafe else 0.2,
                'vz_up_mps': 0.0,
                'failsafe': 0,
                'va_hold_abort_reason': (
                    'va_hold_altitude_runaway' if unsafe else 'none'
                ),
                'condition_id': 'condition_a',
                'va_target_config': 10.0,
                'lambda_target_config': 0.5,
                'mass_scale_config': 1.0,
                'actual_model_mass_kg_config': 5.025,
                'gust_amplitude_mps_config': 0.0,
                'wind_model_config': 'none',
            })


class F5DatasetTests(unittest.TestCase):
    def test_safe_and_future_unsafe_labels_use_two_second_horizon(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'experiment'
            _write_run(root, 'run_1', unsafe_at=2.5)
            dataset, audit = build_f5_dataset([root])
            labels = {int(row['label_safe_2s']) for row in dataset}
            self.assertEqual(labels, {0, 1})
            early = min(dataset, key=lambda row: float(row['time_s']))
            self.assertEqual(early['label_safe_2s'], 1)
            leading = [
                row for row in dataset
                if 0.5 <= float(row['time_s']) <= 2.5
            ]
            self.assertTrue(all(row['label_safe_2s'] == 0 for row in leading))
            self.assertEqual(audit['class_balance']['unsafe'], len(leading) + 15)

    def test_incomplete_horizon_is_not_labeled_safe(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'experiment'
            _write_run(root, 'run_1', duration=1.0)
            dataset, audit = build_f5_dataset([root])
            self.assertEqual(dataset, [])
            self.assertEqual(audit['samples_with_2s_horizon'], 0)

    def test_missing_eta_c_is_reported_and_excluded(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'experiment'
            _write_run(root, 'run_1', eta_c_valid=False)
            dataset, audit = build_f5_dataset([root])
            self.assertEqual(dataset, [])
            self.assertEqual(audit['runs'][0]['eta_c_coverage'], 0.0)

    def test_label_does_not_depend_on_capability_values(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'experiment'
            _write_run(root, 'run_1')
            dataset_a, _ = build_f5_dataset([root])
            telemetry = root / 'run_1' / 'telemetry.csv'
            with telemetry.open(newline='', encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
            for row in rows:
                row['eta_l'] = 0.01
                row['eta_c'] = 0.99
            with telemetry.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=FIELDS)
                writer.writeheader()
                writer.writerows(rows)
            dataset_b, _ = build_f5_dataset([root])
            self.assertEqual(
                [row['label_safe_2s'] for row in dataset_a],
                [row['label_safe_2s'] for row in dataset_b],
            )

    def test_invalid_config_is_rejected(self):
        with self.assertRaises(ValueError):
            F5LabelConfig(horizon_s=0.0).validate()


if __name__ == '__main__':
    unittest.main()

import json
import tempfile
import unittest
from pathlib import Path

from ca_lsc_td3.evaluation.f2_payload import aggregate, write_outputs


class F2PayloadTests(unittest.TestCase):
    def _condition(self, root: Path, scale: float, outcomes: list[str]) -> None:
        slug = str(scale).replace('.', 'p')
        directory = root / f'f2a_payload_m{slug}'
        directory.mkdir()
        (directory / 'condition.json').write_text(json.dumps({
            'mass_scale': scale,
            'nominal_model_mass_kg': 5.0,
            'actual_model_mass_kg': 5.0 * scale,
            'payload_mass_kg': 5.0 * (scale - 1.0),
        }))
        points = []
        for index, outcome in enumerate(outcomes):
            passes = 5 if outcome == 'nominal_feasible' else 3
            points.append({
                'va_target_mps': 6.0,
                'lambda_target': 0.3 + 0.1 * index,
                'physical_class': outcome,
                'outcome_class': outcome,
                'data_quality': 'valid',
                'valid_f1_passes': passes,
                'valid_f1_total': 5,
                'valid_f1_pass_rate': passes / 5,
                'eta_l_proxy_mean': 0.5 / scale,
            })
        (directory / 'f2a_payload_summary.json').write_text(json.dumps({
            'protocol_consistent': True,
            'points': points,
            'derived_by_airspeed': [{
                'va_target_mps': 6.0,
                'tested_lambda_count': len(points),
                'feasible_lambda_count': 1,
                'lambda_max_feasible': 0.3,
                'lambda_max_feasible_display': '0.3',
                'lambda_max_feasible_is_lower_bound': False,
                'feasibility_nonmonotonic': False,
            }],
        }))

    def test_aggregate_preserves_mass_outcomes_and_transitions(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self._condition(root, 1.0, [
                'nominal_feasible', 'boundary_feasible'
            ])
            self._condition(root, 1.2, [
                'boundary_feasible', 'physical_unsafe'
            ])
            payload = aggregate(root)
            self.assertEqual(payload['condition_count'], 2)
            self.assertEqual(payload['point_count'], 4)
            self.assertEqual(payload['primary_point_count'], 4)
            heavy = [
                point for point in payload['points']
                if point['mass_scale'] == 1.2
            ]
            self.assertEqual(heavy[0]['actual_model_mass_kg'], 6.0)
            self.assertEqual(
                heavy[0]['classification_transition_from_nominal'],
                'nominal_feasible->boundary_feasible',
            )
            self.assertEqual(heavy[1]['valid_f1_pass_rate'], 0.6)

    def test_write_outputs_creates_tables_and_three_figures(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'input'
            root.mkdir()
            for scale in (1.0, 1.1, 1.2):
                self._condition(root, scale, [
                    'nominal_feasible', 'physical_unsafe'
                ])
            payload = aggregate(root)
            output = Path(directory) / 'output'
            write_outputs(payload, output)
            for name in (
                'f2a_payload_summary.json',
                'f2a_payload_points.csv',
                'f2a_payload_points_all.csv',
                'f2a_payload_boundaries.csv',
                'f2a_payload_boundaries_all.csv',
                'f2a_payload_outcome_maps.png',
                'f2a_payload_pass_rate_maps.png',
                'f2a_payload_confirmed_feasible.png',
            ):
                self.assertGreater((output / name).stat().st_size, 0)

    def test_va10_lambda1_is_retained_but_excluded_from_primary_grid(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            condition = root / 'f2a_payload_m1p0'
            condition.mkdir()
            (condition / 'condition.json').write_text(json.dumps({
                'mass_scale': 1.0,
                'nominal_model_mass_kg': 5.0,
                'actual_model_mass_kg': 5.0,
                'payload_mass_kg': 0.0,
            }))
            points = [
                {
                    'va_target_mps': 10.0,
                    'lambda_target': lam,
                    'physical_class': 'nominal_feasible',
                    'data_quality': 'valid',
                    'valid_f1_passes': 5,
                    'valid_f1_total': 5,
                }
                for lam in (0.7, 0.8, 0.9, 1.0)
            ]
            (condition / 'f2a_payload_summary.json').write_text(json.dumps({
                'protocol_consistent': True,
                'points': points,
                'derived_by_airspeed': [],
            }))
            payload = aggregate(root)
            self.assertEqual(payload['point_count'], 4)
            self.assertEqual(payload['primary_point_count'], 3)
            self.assertEqual(
                [row['lambda_target'] for row in payload['primary_points']],
                [0.7, 0.8, 0.9],
            )
            self.assertEqual(
                payload['primary_boundaries'][0][
                    'lambda_max_feasible_display'
                ],
                '>=0.9',
            )


if __name__ == '__main__':
    unittest.main()

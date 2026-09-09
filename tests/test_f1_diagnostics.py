import csv
import json
import tempfile
import unittest
from pathlib import Path

from ca_lsc_td3.evaluation.f1_diagnostics import (
    select_representative_run,
)


def _write_run(point: Path, index: int, summary: dict[str, object],
               abort_at: float | None = None) -> None:
    run = point / f'run_{index}'
    run.mkdir(parents=True)
    payload = {
        'source_csv': str(run / 'telemetry.csv'),
        'primary_failure_cause': 'none',
        'va_hold_abort_reason': 'none',
        'va_hold_measurement_complete': True,
        'f1_measurement_pass': True,
    }
    payload.update(summary)
    (run / 'summary.json').write_text(json.dumps(payload) + '\n')
    with (run / 'telemetry.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=(
            'time_s', 'va_hold_abort_reason',
        ))
        writer.writeheader()
        for time in (1.0, 2.0, 3.0):
            writer.writerow({
                'time_s': time,
                'va_hold_abort_reason': (
                    'va_hold_altitude_runaway'
                    if abort_at is not None and time >= abort_at else 'none'
                ),
            })


class F1DiagnosticsTests(unittest.TestCase):
    def test_failure_representative_uses_modal_reason_and_median_timing(self):
        with tempfile.TemporaryDirectory() as directory:
            point = Path(directory)
            good = {}
            failure = {
                'primary_failure_cause': 'va_hold_altitude_runaway',
                'va_hold_abort_reason': 'va_hold_altitude_runaway',
                'va_hold_measurement_complete': False,
                'f1_measurement_pass': False,
                'lambda_target_reached': True,
                'va_target_config': 12.0,
                'va_target_tolerance': 0.5,
                'maximum_transition_airspeed_mps': 12.0,
            }
            _write_run(point, 1, good)
            _write_run(point, 2, failure, 1.0)
            _write_run(point, 3, failure, 2.0)
            _write_run(point, 4, failure, 3.0)
            _write_run(point, 5, good)
            run, _, reason, scores = select_representative_run(point)
            self.assertEqual(run.name, 'run_3')
            self.assertIn('modal physical abort', reason)
            self.assertEqual(scores['run_3'], 2.0)


if __name__ == '__main__':
    unittest.main()

"""Aggregate and plot the formal F2-A payload partial-grid experiment.

Each mass condition is first evaluated with :mod:`f1_grid`, so this module
does not redefine the physical classification.  It joins those condition
summaries, attaches the actual Gazebo mass, and renders comparisons without
assuming that feasibility is monotonic in lambda.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import BoundaryNorm, ListedColormap

from ca_lsc_td3.evaluation.f1_grid import _json_safe
from ca_lsc_td3.evaluation.f1_heatmaps import _outcome_class


OUTCOME_STYLE = {
    'nominal_feasible': (0.0, 'N', '#4daf4a'),
    'boundary_feasible': (1.0, 'B', '#ffe066'),
    'nonconvergent': (2.0, 'I', '#ff9f43'),
    'physical_unsafe': (3.0, 'X', '#d73027'),
    'missing': (4.0, '', '#bdbdbd'),
}

# The formal paper grid was frozen at 18 cells after F2-A finished.  The
# (10 m/s, 1.0) cell was collected and remains useful as a diagnostic, but it
# was not one of the predeclared paper cells and must not silently enter the
# primary maps or boundary comparison.
PRIMARY_DIAGNOSTIC_EXCLUSIONS = {(10.0, 1.0)}

POINT_CSV_FIELDS = (
    'condition_id',
    'mass_scale',
    'nominal_model_mass_kg',
    'actual_model_mass_kg',
    'payload_mass_kg',
    'va_target_mps',
    'lambda_target',
    'outcome_class',
    'physical_class',
    'data_quality',
    'physical_class_reasons',
    'data_quality_reasons',
    'valid_f1_passes',
    'valid_f1_total',
    'valid_f1_pass_rate',
    'measurement_eligible_run_count',
    'eta_l_proxy_mean',
    'eta_l_proxy_std',
    'eta_l_proxy_samples',
    'va_hold_mean_airspeed_mps_mean',
    'va_hold_altitude_rmse_m_mean',
    'va_hold_max_abs_altitude_error_m_p95',
    'source_point_dir',
    'classification_transition_from_nominal',
)

BOUNDARY_CSV_FIELDS = (
    'condition_id',
    'mass_scale',
    'actual_model_mass_kg',
    'va_target_mps',
    'tested_lambda_count',
    'feasible_lambda_count',
    'lambda_max_feasible',
    'lambda_max_feasible_display',
    'lambda_max_feasible_is_lower_bound',
    'lambda_first_infeasible_above',
    'lambda_feasible_interval_display',
    'feasibility_nonmonotonic',
)


def _finite(value: object) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(payload, dict):
        raise ValueError(f'expected JSON object: {path}')
    return payload


def _condition_summaries(root: Path) -> list[tuple[Path, Path]]:
    result: list[tuple[Path, Path]] = []
    for summary_path in sorted(root.glob('f2a_payload_m*/f2a_payload_summary.json')):
        condition_path = summary_path.parent / 'condition.json'
        if condition_path.is_file():
            result.append((condition_path, summary_path))
    if not result:
        raise ValueError(
            f'no f2a_payload_m*/f2a_payload_summary.json conditions under {root}'
        )
    return result


def aggregate(root: Path) -> dict[str, object]:
    """Join all mass-condition F1 summaries beneath *root*."""
    root = root.expanduser().resolve()
    conditions: list[dict[str, object]] = []
    points: list[dict[str, object]] = []
    boundaries: list[dict[str, object]] = []

    for condition_path, summary_path in _condition_summaries(root):
        condition = _load_json(condition_path)
        summary = _load_json(summary_path)
        condition_id = summary_path.parent.name
        metadata = {
            'condition_id': condition_id,
            'mass_scale': _finite(condition.get('mass_scale')),
            'nominal_model_mass_kg': _finite(
                condition.get('nominal_model_mass_kg')
            ),
            'actual_model_mass_kg': _finite(
                condition.get('actual_model_mass_kg')
            ),
            'payload_mass_kg': _finite(condition.get('payload_mass_kg')),
        }
        condition_points = summary.get('points', [])
        condition_boundaries = summary.get('derived_by_airspeed', [])
        if not isinstance(condition_points, list) or not isinstance(
            condition_boundaries, list
        ):
            raise ValueError(f'invalid F1-compatible summary: {summary_path}')
        for original in condition_points:
            if not isinstance(original, dict):
                continue
            point = {**original, **metadata}
            point['outcome_class'] = _outcome_class(point)
            points.append(point)
        for original in condition_boundaries:
            if isinstance(original, dict):
                boundaries.append({**original, **metadata})
        conditions.append({
            **metadata,
            'summary_json': str(summary_path.resolve()),
            'point_count': len(condition_points),
            'protocol_consistent': bool(summary.get('protocol_consistent')),
        })

    conditions.sort(key=lambda item: _finite(item.get('mass_scale')))
    points.sort(key=lambda item: (
        _finite(item.get('mass_scale')),
        _finite(item.get('va_target_mps')),
        _finite(item.get('lambda_target')),
    ))
    boundaries.sort(key=lambda item: (
        _finite(item.get('mass_scale')),
        _finite(item.get('va_target_mps')),
    ))

    nominal_candidates = [
        item for item in conditions
        if abs(_finite(item.get('mass_scale')) - 1.0) <= 1e-9
    ]
    nominal_id = str(nominal_candidates[0]['condition_id']) if (
        nominal_candidates
    ) else str(conditions[0]['condition_id'])
    nominal_outcomes = {
        (_finite(item.get('va_target_mps')), _finite(item.get('lambda_target'))):
        str(item.get('outcome_class'))
        for item in points if item.get('condition_id') == nominal_id
    }
    for point in points:
        key = (
            _finite(point.get('va_target_mps')),
            _finite(point.get('lambda_target')),
        )
        baseline = nominal_outcomes.get(key, 'missing')
        current = str(point.get('outcome_class', 'missing'))
        point['classification_transition_from_nominal'] = (
            current if point.get('condition_id') == nominal_id
            else f'{baseline}->{current}'
        )

    primary_points = [
        item for item in points
        if (
            _finite(item.get('va_target_mps')),
            _finite(item.get('lambda_target')),
        ) not in PRIMARY_DIAGNOSTIC_EXCLUSIONS
    ]
    primary_boundaries: list[dict[str, object]] = []
    for condition in conditions:
        condition_id = str(condition['condition_id'])
        grouped: dict[float, list[dict[str, object]]] = {}
        for point in primary_points:
            if str(point.get('condition_id')) != condition_id:
                continue
            grouped.setdefault(
                _finite(point.get('va_target_mps')), []
            ).append(point)
        for va, rows in sorted(grouped.items()):
            rows.sort(key=lambda row: _finite(row.get('lambda_target')))
            feasible = [
                row for row in rows
                if row.get('data_quality') in {'valid', 'valid_with_retries'}
                and row.get('outcome_class') in {
                    'nominal_feasible', 'boundary_feasible'
                }
            ]
            maximum = max(
                (_finite(row.get('lambda_target')) for row in feasible),
                default=math.nan,
            )
            higher_infeasible = [
                _finite(row.get('lambda_target')) for row in rows
                if math.isfinite(maximum)
                and _finite(row.get('lambda_target')) > maximum
                and row.get('outcome_class') in {
                    'nonconvergent', 'physical_unsafe'
                }
            ]
            highest_tested = max(
                (_finite(row.get('lambda_target')) for row in rows),
                default=math.nan,
            )
            lower_bound = (
                math.isfinite(maximum)
                and maximum == highest_tested
                and not higher_infeasible
            )
            nonmonotonic = any(
                row.get('outcome_class') in {
                    'nonconvergent', 'physical_unsafe'
                }
                and _finite(row.get('lambda_target')) < maximum
                for row in rows
            ) if math.isfinite(maximum) else False
            primary_boundaries.append({
                **{
                    key: condition.get(key) for key in (
                        'condition_id', 'mass_scale',
                        'actual_model_mass_kg',
                    )
                },
                'va_target_mps': va,
                'tested_lambda_count': len(rows),
                'feasible_lambda_count': len(feasible),
                'lambda_max_feasible': maximum,
                'lambda_max_feasible_display': (
                    f'>={maximum:g}' if lower_bound
                    else f'{maximum:g}' if math.isfinite(maximum) else ''
                ),
                'lambda_max_feasible_is_lower_bound': lower_bound,
                'lambda_first_infeasible_above': (
                    min(higher_infeasible) if higher_infeasible else math.nan
                ),
                'lambda_feasible_interval_display': (
                    f'[{maximum:g}, {min(higher_infeasible):g})'
                    if higher_infeasible else
                    f'>={maximum:g}' if lower_bound else
                    f'{maximum:g}' if math.isfinite(maximum) else ''
                ),
                'feasibility_nonmonotonic': nonmonotonic,
            })

    return {
        'experiment': 'F2A_payload_Va_lambda_partial_scan',
        'root': str(root),
        'condition_count': len(conditions),
        'point_count': len(points),
        'primary_point_count': len(primary_points),
        'nominal_condition_id': nominal_id,
        'conditions': conditions,
        'points': points,
        'primary_points': primary_points,
        'boundaries': boundaries,
        'primary_boundaries': primary_boundaries,
        'primary_diagnostic_exclusions': [
            {'va_target_mps': va, 'lambda_target': lam}
            for va, lam in sorted(PRIMARY_DIAGNOSTIC_EXCLUSIONS)
        ],
        'eta_l_proxy_note': (
            'Inherited from the corrected F1 aggregator: selected air-relative '
            'speed and each run actual_model_mass_kg_config are used. This is '
            'still an offline model-based diagnostic; formal eta_L validation '
            'belongs to F3.'
        ),
        'boundary_note': (
            'Maximum confirmed feasible lambda within the tested partial grid. '
            'It is not forced to be monotonic in airspeed or mass; starred '
            'rows contain a nonconvergent/unsafe pocket below a higher feasible '
            'sample.'
        ),
        'power_model_reportable': False,
    }


def _axes(points: list[dict[str, object]]) -> tuple[list[float], list[float]]:
    vas = sorted({
        _finite(item.get('va_target_mps')) for item in points
        if math.isfinite(_finite(item.get('va_target_mps')))
    })
    lambdas = sorted({
        _finite(item.get('lambda_target')) for item in points
        if math.isfinite(_finite(item.get('lambda_target')))
    })
    return vas, lambdas


def _condition_matrix(
    points: list[dict[str, object]], condition_id: str, key: str,
) -> tuple[list[float], list[float], np.ndarray, dict[tuple[int, int], str]]:
    vas, lambdas = _axes(points)
    values = np.full((len(vas), len(lambdas)), np.nan)
    annotations: dict[tuple[int, int], str] = {}
    vi = {value: index for index, value in enumerate(vas)}
    li = {value: index for index, value in enumerate(lambdas)}
    for item in points:
        if str(item.get('condition_id')) != condition_id:
            continue
        va = _finite(item.get('va_target_mps'))
        lam = _finite(item.get('lambda_target'))
        if va not in vi or lam not in li:
            continue
        row, column = vi[va], li[lam]
        if key == 'outcome_class':
            outcome = str(item.get('outcome_class', 'missing'))
            value, label, _ = OUTCOME_STYLE.get(
                outcome, OUTCOME_STYLE['missing']
            )
            values[row, column] = value
            annotations[(row, column)] = label
        else:
            value = _finite(item.get(key))
            values[row, column] = value
            passes = int(_finite(item.get('valid_f1_passes'))) if math.isfinite(
                _finite(item.get('valid_f1_passes'))
            ) else 0
            total = int(_finite(item.get('valid_f1_total'))) if math.isfinite(
                _finite(item.get('valid_f1_total'))
            ) else 0
            annotations[(row, column)] = f'{passes}/{total}'
    return vas, lambdas, values, annotations


def _mass_label(condition: dict[str, object]) -> str:
    return (
        f"{_finite(condition.get('mass_scale')):g}$m_0$ "
        f"({_finite(condition.get('actual_model_mass_kg')):.3f} kg)"
    )


def _render_maps(
    payload: dict[str, object], output: Path, *, pass_rate: bool,
) -> None:
    points = payload.get('primary_points', payload['points'])
    conditions = payload['conditions']
    assert isinstance(points, list) and isinstance(conditions, list)
    figure, axes = plt.subplots(
        1, len(conditions), figsize=(5.1 * len(conditions), 5.0),
        constrained_layout=True, squeeze=False,
    )
    if pass_rate:
        cmap = plt.get_cmap('viridis').copy()
        cmap.set_bad('#d9d9d9')
        norm = None
    else:
        cmap = ListedColormap([
            OUTCOME_STYLE[name][2] for name in (
                'nominal_feasible', 'boundary_feasible', 'nonconvergent',
                'physical_unsafe', 'missing',
            )
        ])
        cmap.set_bad('#d9d9d9')
        norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5, 4.5], cmap.N)
    image = None
    for axis, condition in zip(axes[0], conditions):
        condition_id = str(condition['condition_id'])
        key = 'valid_f1_pass_rate' if pass_rate else 'outcome_class'
        vas, lambdas, values, annotations = _condition_matrix(
            points, condition_id, key
        )
        image = axis.imshow(
            np.ma.masked_invalid(values), origin='lower', aspect='auto',
            cmap=cmap, norm=norm, vmin=0.0 if pass_rate else None,
            vmax=1.0 if pass_rate else None,
        )
        axis.set_xticks(range(len(lambdas)), [f'{value:g}' for value in lambdas])
        axis.set_yticks(range(len(vas)), [f'{value:g}' for value in vas])
        axis.set_xlabel(r'Transition allocation factor $\lambda$')
        axis.set_ylabel(r'Target airspeed $V_a$ (m/s)')
        axis.set_title(_mass_label(condition))
        for (row, column), label in annotations.items():
            if label:
                axis.text(
                    column, row, label, ha='center', va='center', fontsize=8,
                    fontweight='bold', color='white' if (
                        not pass_rate and label in {'I', 'X'}
                    ) else 'black',
                )
    if image is not None:
        ticks = [0, 1, 2, 3, 4] if not pass_rate else None
        colorbar = figure.colorbar(image, ax=axes.ravel().tolist(), ticks=ticks)
        if pass_rate:
            colorbar.set_label('Valid-repeat F1 pass rate')
        else:
            colorbar.ax.set_yticklabels(['N', 'B', 'I', 'X', 'missing'])
            colorbar.set_label('Physical outcome class')
    figure.suptitle(
        'F2-A payload partial scan: valid-repeat pass rate' if pass_rate else
        'F2-A payload partial scan: N/B/I/X outcome',
        fontsize=13,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=220)
    plt.close(figure)


def _render_boundaries(payload: dict[str, object], output: Path) -> None:
    conditions = payload['conditions']
    boundaries = payload.get('primary_boundaries', payload['boundaries'])
    assert isinstance(conditions, list) and isinstance(boundaries, list)
    figure, axis = plt.subplots(figsize=(7.6, 4.9), constrained_layout=True)
    for condition in conditions:
        condition_id = str(condition['condition_id'])
        rows = [
            row for row in boundaries
            if row.get('condition_id') == condition_id
            and math.isfinite(_finite(row.get('lambda_max_feasible')))
        ]
        rows.sort(key=lambda row: _finite(row.get('va_target_mps')))
        if not rows:
            continue
        x = [_finite(row.get('va_target_mps')) for row in rows]
        y = [_finite(row.get('lambda_max_feasible')) for row in rows]
        line, = axis.plot(x, y, '--o', label=_mass_label(condition))
        for row, va, lam in zip(rows, x, y):
            if bool(row.get('lambda_max_feasible_is_lower_bound')):
                axis.annotate(
                    '', xy=(va, min(1.03, lam + 0.07)), xytext=(va, lam),
                    arrowprops={'arrowstyle': '-|>', 'color': line.get_color()},
                )
            if bool(row.get('feasibility_nonmonotonic')):
                axis.text(va, min(1.05, lam + 0.04), '*', ha='center',
                          color=line.get_color(), fontweight='bold')
    axis.set_xlabel(r'Target airspeed $V_a$ (m/s)')
    axis.set_ylabel(r'Maximum confirmed feasible $\lambda$')
    axis.set_ylim(-0.02, 1.08)
    axis.set_title('F2-A confirmed feasibility within the partial scan')
    axis.grid(True, alpha=0.25)
    axis.legend()
    axis.text(
        0.0, -0.20,
        'Arrows: right-censored by the highest tested lambda.  *: '
        'non-monotonic row.  Lines guide comparison only.',
        transform=axis.transAxes, fontsize=8,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=220)
    plt.close(figure)


def _write_csv(rows: list[dict[str, object]], fields: tuple[str, ...], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _json_safe(row.get(key)) for key in fields})


def write_outputs(payload: dict[str, object], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'f2a_payload_summary.json').write_text(
        json.dumps(_json_safe(payload), indent=2, allow_nan=False) + '\n',
        encoding='utf-8',
    )
    _write_csv(payload.get('primary_points', payload['points']), POINT_CSV_FIELDS,
               output_dir / 'f2a_payload_points.csv')
    _write_csv(payload['points'], POINT_CSV_FIELDS,
               output_dir / 'f2a_payload_points_all.csv')
    _write_csv(payload.get('primary_boundaries', payload['boundaries']),
               BOUNDARY_CSV_FIELDS,
               output_dir / 'f2a_payload_boundaries.csv')
    _write_csv(payload['boundaries'], BOUNDARY_CSV_FIELDS,
               output_dir / 'f2a_payload_boundaries_all.csv')
    _render_maps(payload, output_dir / 'f2a_payload_outcome_maps.png',
                 pass_rate=False)
    _render_maps(payload, output_dir / 'f2a_payload_pass_rate_maps.png',
                 pass_rate=True)
    _render_boundaries(payload, output_dir / 'f2a_payload_confirmed_feasible.png')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    payload = aggregate(args.root)
    output_dir = args.output_dir or args.root / 'combined'
    write_outputs(payload, output_dir)
    print(json.dumps(_json_safe(payload), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()

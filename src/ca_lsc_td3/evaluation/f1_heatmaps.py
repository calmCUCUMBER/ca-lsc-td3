"""Render physical, data-quality, boundary, and diagnostic F1 figures."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def _finite(value: object) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _point_classes(item: dict[str, object]) -> tuple[str, str]:
    """Read the split schema with a legacy fallback for archived summaries."""
    physical = str(item.get('physical_class', ''))
    quality = str(item.get('data_quality', ''))
    if physical and quality:
        return physical, quality
    legacy = str(item.get('classification', ''))
    if legacy in {'protocol_transient', 'protocol_invalid'}:
        return 'unknown', 'protocol_invalid'
    if legacy == 'unsafe_or_abort':
        return 'physical_unsafe', 'valid'
    return legacy or 'unknown', 'valid'


def _quality_usable(value: object) -> bool:
    return str(value) in {'valid', 'valid_with_retries'}


def _outcome_class(item: dict[str, object]) -> str:
    physical, quality = _point_classes(item)
    if not _quality_usable(quality):
        return 'missing'
    outcome = str(item.get('outcome_class', ''))
    if outcome in {
        'nominal_feasible', 'boundary_feasible', 'nonconvergent',
        'physical_unsafe',
    }:
        return outcome
    return physical


def _grid_axes(
    points: list[dict[str, object]],
) -> tuple[list[float], list[float]]:
    vas = sorted({_finite(item.get('va_target_mps')) for item in points})
    lambdas = sorted({_finite(item.get('lambda_target')) for item in points})
    vas = [value for value in vas if math.isfinite(value)]
    lambdas = [value for value in lambdas if math.isfinite(value)]
    # F1's declared domain is lambda=0:0.1:1.  Preserve every declared column
    # even in exploratory partial batches so an entirely untested value (for
    # example lambda=0.4 in schema v7) appears gray instead of disappearing.
    if lambdas and min(lambdas) >= -1e-9 and max(lambdas) <= 1.0 + 1e-9:
        lambdas = sorted(set(lambdas) | {index / 10 for index in range(11)})
    return vas, lambdas


def _matrix(
    points: list[dict[str, object]], key: str, *, valid_only: bool = True
) -> tuple[list[float], list[float], np.ndarray, dict[tuple[int, int], str]]:
    vas, lambdas = _grid_axes(points)
    values = np.full((len(vas), len(lambdas)), np.nan, dtype=float)
    labels: dict[tuple[int, int], str] = {}
    va_index = {value: index for index, value in enumerate(vas)}
    lambda_index = {value: index for index, value in enumerate(lambdas)}
    for item in points:
        va = _finite(item.get('va_target_mps'))
        lam = _finite(item.get('lambda_target'))
        value = _finite(item.get(key))
        if va not in va_index or lam not in lambda_index:
            continue
        row, column = va_index[va], lambda_index[lam]
        physical, quality = _point_classes(item)
        if not valid_only or _quality_usable(quality):
            values[row, column] = value
        if _quality_usable(quality):
            labels[(row, column)] = {
                'physical_unsafe': 'X',
                'nonconvergent': 'I',
                'boundary_feasible': 'B',
            }.get(physical, '')
    return vas, lambdas, values, labels


def _render_heatmap(
    points: list[dict[str, object]],
    *,
    key: str,
    title: str,
    colorbar_label: str,
    output: Path,
    footer: str,
    valid_only: bool = True,
) -> None:
    vas, lambdas, values, labels = _matrix(
        points, key, valid_only=valid_only
    )
    if not vas or not lambdas:
        raise ValueError('no finite F1 grid points available for plotting')
    masked = np.ma.masked_invalid(values)
    cmap = plt.get_cmap('viridis').copy()
    cmap.set_bad('#d9d9d9')
    figure, axis = plt.subplots(figsize=(8.4, 5.6), constrained_layout=True)
    image = axis.imshow(masked, origin='lower', aspect='auto', cmap=cmap)
    axis.set_xticks(range(len(lambdas)), [f'{value:g}' for value in lambdas])
    axis.set_yticks(range(len(vas)), [f'{value:g}' for value in vas])
    axis.set_xlabel(r'Transition allocation factor $\lambda$')
    axis.set_ylabel(r'Target airspeed $V_a$ (m/s)')
    axis.set_title(title)
    colorbar = figure.colorbar(image, ax=axis)
    colorbar.set_label(colorbar_label)
    for (row, column), label in labels.items():
        if label:
            axis.text(
                column, row, label, ha='center', va='center',
                color='white' if label in {'I', 'X'} else 'black',
                fontsize=9, fontweight='bold',
            )
    axis.text(0.0, -0.16, footer, transform=axis.transAxes, fontsize=8)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=220)
    plt.close(figure)


def _render_data_quality(
    points: list[dict[str, object]], output: Path
) -> None:
    vas, lambdas = _grid_axes(points)
    values = np.full((len(vas), len(lambdas)), np.nan)
    va_index = {value: index for index, value in enumerate(vas)}
    lambda_index = {value: index for index, value in enumerate(lambdas)}
    labels = {
        'valid': 0.0,
        'valid_with_retries': 1.0,
        'protocol_unstable': 2.0,
        'protocol_invalid': 2.0,
        'missing': 3.0,
    }
    for item in points:
        va = _finite(item.get('va_target_mps'))
        lam = _finite(item.get('lambda_target'))
        if va in va_index and lam in lambda_index:
            _, quality = _point_classes(item)
            values[va_index[va], lambda_index[lam]] = labels.get(
                quality, 3.0
            )
    from matplotlib.colors import BoundaryNorm, ListedColormap
    cmap = ListedColormap(['#4daf4a', '#80b1d3', '#ffbf00', '#bdbdbd'])
    cmap.set_bad('#bdbdbd')
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5], cmap.N)
    figure, axis = plt.subplots(figsize=(8.4, 5.2), constrained_layout=True)
    image = axis.imshow(np.ma.masked_invalid(values), origin='lower',
                        aspect='auto', cmap=cmap, norm=norm)
    axis.set_xticks(range(len(lambdas)), [f'{value:g}' for value in lambdas])
    axis.set_yticks(range(len(vas)), [f'{value:g}' for value in vas])
    axis.set_xlabel(r'Transition allocation factor $\lambda$')
    axis.set_ylabel(r'Target airspeed $V_a$ (m/s)')
    axis.set_title('F1 experiment data quality (separate from physical class)')
    colorbar = figure.colorbar(image, ax=axis, ticks=[0, 1, 2, 3])
    colorbar.ax.set_yticklabels([
        '5 valid / no retry', '>=5 valid / with audited retries',
        'protocol unstable', 'not sampled / missing',
    ])
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=220)
    plt.close(figure)


def _render_boundary(rows: list[dict[str, object]], output: Path) -> None:
    valid = [
        row for row in rows
        if math.isfinite(_finite(row.get('lambda_max_feasible')))
    ]
    if not valid:
        raise ValueError('no finite feasible boundary values')
    x = [_finite(row.get('va_target_mps')) for row in valid]
    y = [_finite(row.get('lambda_max_feasible')) for row in valid]
    figure, axis = plt.subplots(figsize=(7.2, 4.6), constrained_layout=True)
    axis.plot(
        x, y, '-o', color='#1f77b4',
        label='maximum confirmed feasible',
    )
    for row, va, lam in zip(valid, x, y):
        if bool(row.get('feasibility_nonmonotonic')):
            axis.text(
                va, min(1.04, lam + 0.045), '*', ha='center',
                va='bottom', color='#1f77b4', fontsize=13,
                fontweight='bold',
            )
        if bool(row.get('lambda_max_feasible_is_lower_bound')):
            axis.annotate(
                '', xy=(va, min(1.02, lam + 0.09)), xytext=(va, lam),
                arrowprops={'arrowstyle': '-|>', 'color': '#1f77b4'},
            )
            axis.text(va, min(1.04, lam + 0.11), f'$\geq${lam:g}',
                      ha='center', va='bottom', fontsize=8)
        else:
            upper = _finite(row.get('lambda_first_infeasible_above'))
            if not math.isfinite(upper):
                upper = _finite(row.get('lambda_first_unsafe_above'))
            if math.isfinite(upper):
                axis.vlines(va, lam, upper, color='#d62728', linewidth=2)
                axis.plot([va], [upper], marker='x', color='#d62728')
    axis.set_xlim(min(x) - 0.5, max(x) + 0.5)
    axis.set_ylim(-0.02, 1.08)
    axis.set_xlabel(r'Target airspeed $V_a$ (m/s)')
    axis.set_ylabel(r'Maximum confirmed feasible $\lambda$')
    axis.set_title(r'Maximum confirmed feasible $\lambda$ versus airspeed')
    axis.grid(True, alpha=0.25)
    axis.legend(loc='lower right')
    axis.text(
        0.01, -0.19,
        '*: non-monotonic feasibility observed within the tested lambda row; '
        'red brackets are grid-resolution bounds, not confidence intervals.',
        transform=axis.transAxes, fontsize=8,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=220)
    plt.close(figure)


def _render_outcome_classes(
    points: list[dict[str, object]], output: Path
) -> None:
    vas, lambdas = _grid_axes(points)
    values = np.full((len(vas), len(lambdas)), np.nan)
    va_index = {value: index for index, value in enumerate(vas)}
    lambda_index = {value: index for index, value in enumerate(lambdas)}
    labels = {
        'nominal_feasible': (0.0, 'N'),
        'boundary_feasible': (1.0, 'B'),
        'nonconvergent': (2.0, 'I'),
        'physical_unsafe': (3.0, 'X'),
        'missing': (4.0, ''),
    }
    annotations: dict[tuple[int, int], str] = {}
    for item in points:
        va = _finite(item.get('va_target_mps'))
        lam = _finite(item.get('lambda_target'))
        if va not in va_index or lam not in lambda_index:
            continue
        outcome = _outcome_class(item)
        value, label = labels.get(outcome, labels['missing'])
        row, column = va_index[va], lambda_index[lam]
        values[row, column] = value
        annotations[(row, column)] = label
    from matplotlib.colors import BoundaryNorm, ListedColormap
    cmap = ListedColormap([
        '#4daf4a', '#ffe066', '#ff9f43', '#d73027', '#bdbdbd'
    ])
    cmap.set_bad('#bdbdbd')
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5, 4.5], cmap.N)
    figure, axis = plt.subplots(figsize=(8.8, 5.4), constrained_layout=True)
    image = axis.imshow(np.ma.masked_invalid(values), origin='lower',
                        aspect='auto', cmap=cmap, norm=norm)
    axis.set_xticks(range(len(lambdas)), [f'{value:g}' for value in lambdas])
    axis.set_yticks(range(len(vas)), [f'{value:g}' for value in vas])
    axis.set_xlabel(r'Transition allocation factor $\lambda$')
    axis.set_ylabel(r'Target airspeed $V_a$ (m/s)')
    axis.set_title('F1 fixed-condition outcome map')
    for (row, column), label in annotations.items():
        if label:
            axis.text(column, row, label, ha='center', va='center',
                      color='white' if label == 'X' else 'black',
                      fontweight='bold')
    colorbar = figure.colorbar(image, ax=axis, ticks=range(5))
    colorbar.ax.set_yticklabels([
        'N nominal', 'B marginal feasible', 'I non-convergent',
        'X physical unsafe', 'not resolved (see data quality)',
    ])
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=220)
    plt.close(figure)


def _render_paper_feasibility(
    points: list[dict[str, object]], output: Path
) -> None:
    """Render the deliberately simple paper-facing feasible/infeasible map."""
    vas, lambdas = _grid_axes(points)
    values = np.full((len(vas), len(lambdas)), np.nan)
    va_index = {value: index for index, value in enumerate(vas)}
    lambda_index = {value: index for index, value in enumerate(lambdas)}
    for item in points:
        va = _finite(item.get('va_target_mps'))
        lam = _finite(item.get('lambda_target'))
        if va not in va_index or lam not in lambda_index:
            continue
        _, quality = _point_classes(item)
        if not _quality_usable(quality):
            continue
        outcome = _outcome_class(item)
        values[va_index[va], lambda_index[lam]] = (
            1.0 if outcome in {'nominal_feasible', 'boundary_feasible'}
            else 0.0
        )
    from matplotlib.colors import BoundaryNorm, ListedColormap
    cmap = ListedColormap(['#8c8c8c', '#2ca25f'])
    cmap.set_bad('#e0e0e0')
    norm = BoundaryNorm([-0.5, 0.5, 1.5], cmap.N)
    figure, axis = plt.subplots(figsize=(8.4, 5.2), constrained_layout=True)
    image = axis.imshow(
        np.ma.masked_invalid(values), origin='lower', aspect='auto',
        cmap=cmap, norm=norm,
    )
    axis.set_xticks(range(len(lambdas)), [f'{value:g}' for value in lambdas])
    axis.set_yticks(range(len(vas)), [f'{value:g}' for value in vas])
    axis.set_xlabel(r'Transition allocation factor $\lambda$')
    axis.set_ylabel(r'Target airspeed $V_a$ (m/s)')
    axis.set_title('F1 feasibility under frozen low-level control')
    colorbar = figure.colorbar(image, ax=axis, ticks=[0, 1])
    colorbar.ax.set_yticklabels(['unsafe / infeasible', 'feasible'])
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=220)
    plt.close(figure)


def render_all(payload: dict[str, object], output_dir: Path) -> None:
    points = payload.get('points')
    if not isinstance(points, list):
        raise ValueError('summary JSON does not contain a points list')
    _render_outcome_classes(points, output_dir / 'f1_outcome_classes.png')
    _render_paper_feasibility(points, output_dir / 'f1_feasible_map.png')
    _render_heatmap(
        points,
        key='va_hold_mean_total_power_w_mean',
        title='F1 diagnostic: static motor-model effort proxy',
        colorbar_label='Static $Q\\omega$ model proxy (W; not reportable power)',
        output=output_dir / 'f1_va_lambda_total_power.png',
        footer='B: marginal feasible   I: non-convergent   X: physical unsafe   gray: excluded/not sampled; diagnostic only',
    )
    _render_heatmap(
        points,
        key='va_hold_max_abs_altitude_error_m_p95',
        title=r'F1: $V_a$-$\lambda$ P95 maximum altitude error',
        colorbar_label=r'Across-run P95 of max $|\Delta h|$ (m)',
        output=output_dir / 'f1_va_lambda_max_altitude_error.png',
        footer='B: marginal feasible   I: non-convergent   X: physical unsafe   gray: protocol invalid/not sampled',
    )
    _render_heatmap(
        points,
        key='va_hold_max_abs_altitude_error_m_max',
        title=r'F1 coarse diagnostic: worst-run maximum altitude error',
        colorbar_label=r'Worst run: max $|\Delta h|$ (m)',
        output=output_dir / 'f1_va_lambda_max_altitude_error_diagnostic.png',
        footer='Worst-of-five diagnostic; do not use alone for repeatable boundary classification',
        valid_only=False,
    )
    _render_heatmap(
        points,
        key='pusher_propulsive_efficiency_proxy_median',
        title=r'F1 power sanity: median $T_PV_{axial}/P_P$',
        colorbar_label='Apparent efficiency (values > 1 violate energy bound)',
        output=output_dir / 'f1_pusher_power_sanity.png',
        footer='Values > 1 indicate the static motor power proxy is not physically reportable',
    )
    _render_data_quality(points, output_dir / 'f1_data_quality.png')
    rows = payload.get('derived_by_airspeed')
    if isinstance(rows, list):
        _render_boundary(rows, output_dir / 'f1_feasible_boundary.png')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('summary_json', type=Path)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    payload = json.loads(args.summary_json.read_text(encoding='utf-8'))
    render_all(payload, args.output_dir or args.summary_json.parent)


if __name__ == '__main__':
    main()

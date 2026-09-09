"""Analyze Physics Prior Only nominal-transition smoke runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import fmean

import matplotlib

matplotlib.use("Agg")


FRESHNESS_LIMIT_S = 0.5


def _number(row: dict[str, str], key: str) -> float:
    try:
        value = float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def _values(rows: list[dict[str, str]], key: str) -> list[float]:
    return [
        value
        for row in rows
        for value in [_number(row, key)]
        if math.isfinite(value)
    ]


def _mean(values: list[float]) -> float:
    return fmean(values) if values else math.nan


def _rmse(values: list[float]) -> float:
    return math.sqrt(fmean(value * value for value in values)) if values else math.nan


def _fraction(values: list[bool]) -> float:
    return sum(values) / len(values) if values else math.nan


def _contains_in_order(sequence: list[object], required: tuple[object, ...]) -> bool:
    required_index = 0
    for value in sequence:
        if value == required[required_index]:
            required_index += 1
            if required_index == len(required):
                return True
    return False


def _json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def _transition_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    return [
        row for row in rows
        if row.get("command_state", "").split("|", 1)[0] == "TRANSITION_FW"
    ]


def _fresh_fraction(rows: list[dict[str, str]], key: str, age_key: str) -> float:
    return _fraction([
        math.isfinite(_number(row, key))
        and math.isfinite(_number(row, age_key))
        and 0.0 <= _number(row, age_key) <= FRESHNESS_LIMIT_S
        for row in rows
    ])


def _range_fraction(values: list[float], lower: float, upper: float) -> float:
    return _fraction([lower <= value <= upper for value in values])


def _early_late_means(
    rows: list[dict[str, str]],
    key: str,
    *,
    fraction: float = 0.2,
) -> tuple[float, float]:
    if len(rows) < 5:
        return math.nan, math.nan
    count = max(1, int(round(len(rows) * fraction)))
    return _mean(_values(rows[:count], key)), _mean(_values(rows[-count:], key))


def _max_continuous_dwell_s(
    rows: list[dict[str, str]],
    predicate,
) -> float:
    started_at = math.nan
    previous_time = math.nan
    best = 0.0
    for row in rows:
        time_s = _number(row, "time_s")
        if not math.isfinite(time_s) or not predicate(row):
            started_at = math.nan
            previous_time = math.nan
            continue
        if not math.isfinite(started_at):
            started_at = time_s
        previous_time = time_s
        best = max(best, previous_time - started_at)
    return best


def _first_config_value(
    rows: list[dict[str, str]],
    key: str,
    default: float,
) -> float:
    for row in rows:
        value = _number(row, key)
        if math.isfinite(value):
            return value
    return default


def _plot_timeseries(
    rows: list[dict[str, str]],
    output_png: Path,
) -> bool:
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        return False

    transition = _transition_rows(rows)
    if not transition:
        return False
    t0 = _number(transition[0], "time_s")

    def series(key: str) -> tuple[list[float], list[float]]:
        xs: list[float] = []
        ys: list[float] = []
        for row in transition:
            t = _number(row, "time_s")
            value = _number(row, key)
            if math.isfinite(t) and math.isfinite(value):
                xs.append(t - t0 if math.isfinite(t0) else t)
                ys.append(value)
        return xs, ys

    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(4, 1, figsize=(9.0, 7.0), sharex=True)
    for key, label in (
        ("selected_airspeed_mps", "selected Va [m/s]"),
        ("eta_l", "eta_L"),
        ("eta_c", "eta_C"),
    ):
        x, y = series(key)
        axes[0 if key == "selected_airspeed_mps" else 1].plot(x, y, label=label)
    axes[1].legend(loc="best")
    for key, label in (
        ("lambda_phy", "lambda_phy"),
        ("lambda_max_hard", "lambda_max_hard"),
        ("lambda_command", "lambda_command"),
        ("lambda_exec", "lambda_exec"),
    ):
        x, y = series(key)
        axes[2].plot(x, y, label=label)
    axes[2].legend(loc="best")
    for key, label in (
        ("altitude_error_m", "e_h [m]"),
        ("vz_up_mps", "vz_up [m/s]"),
        ("alpha_est_rad", "alpha [rad]"),
    ):
        x, y = series(key)
        axes[3].plot(x, y, label=label)
    axes[3].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    for axis in axes[1:]:
        axis.grid(True, alpha=0.3)
    axes[-1].set_xlabel("transition time [s]")
    fig.tight_layout()
    fig.savefig(output_png, dpi=160)
    plt.close(fig)
    return True


def analyze_csv(path: Path, *, plot: bool = True) -> dict[str, object]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"no telemetry rows in {path}")

    states = [row.get("command_state", "").split("|", 1)[0] for row in rows]
    vtol_states = [
        int(value)
        for row in rows
        for value in [_number(row, "vtol_state")]
        if math.isfinite(value)
    ]
    transition = _transition_rows(rows)
    prior_rows = [
        row for row in transition
        if row.get("schedule_mode") == "physics_prior_only"
        and _number(row, "physics_prior_active") > 0.5
        and math.isfinite(_number(row, "lambda_command"))
    ]

    lambda_phy = _values(prior_rows, "lambda_phy")
    lambda_command = _values(prior_rows, "lambda_command")
    lambda_exec = _values(prior_rows, "lambda_exec")
    lambda_hard = _values(prior_rows, "lambda_max_hard")
    selected_airspeed = _values(prior_rows, "selected_airspeed_mps")
    eta_l = _values(prior_rows, "eta_l")
    eta_c = _values(prior_rows, "eta_c")
    alpha = [
        _number(row, "alpha_est_rad")
        for row in prior_rows
        if _number(row, "alpha_valid") > 0.5
        and math.isfinite(_number(row, "alpha_est_rad"))
    ]
    altitude_errors = _values(transition, "altitude_error_m")
    vz = _values(transition, "vz_up_mps")
    tracking_errors = [
        _number(row, "lambda_exec") - _number(row, "lambda_command")
        for row in prior_rows
        if math.isfinite(_number(row, "lambda_exec"))
        and math.isfinite(_number(row, "lambda_command"))
    ]
    hard_violations = [
        _number(row, "lambda_command") - _number(row, "lambda_max_hard")
        for row in prior_rows
        if math.isfinite(_number(row, "lambda_command"))
        and math.isfinite(_number(row, "lambda_max_hard"))
    ]
    early_lambda, late_lambda = _early_late_means(prior_rows, "lambda_phy")
    early_eta_l, late_eta_l = _early_late_means(prior_rows, "eta_l")
    early_eta_c, late_eta_c = _early_late_means(prior_rows, "eta_c")
    release_lambda = _first_config_value(
        rows, "physics_prior_release_lambda_config", 0.95
    )
    release_dwell_s = _first_config_value(
        rows, "physics_prior_release_dwell_s_config", 2.0
    )
    vt_transition_airspeed = _first_config_value(
        rows, "vt_transition_airspeed_mps_config", 13.0
    )
    va_target = _first_config_value(rows, "va_target_config", 15.0)
    full_allocation_dwell = _max_continuous_dwell_s(
        prior_rows,
        lambda row: (
            _number(row, "lambda_phy") >= release_lambda
            and _number(row, "lambda_exec") >= release_lambda
            and _number(row, "selected_airspeed_mps") >= vt_transition_airspeed
        ),
    )
    lambda_phy_peak_reached = max(lambda_phy, default=math.nan) >= release_lambda
    release_dwell_pass = full_allocation_dwell >= release_dwell_s
    lambda_phy_post_peak_drop = math.nan
    if lambda_phy:
        lambda_phy_post_peak_drop = max(lambda_phy) - lambda_phy[-1]
    eta_l_gt_errors = [
        _number(row, "eta_l") - _number(row, "eta_l_gt")
        for row in prior_rows
        if math.isfinite(_number(row, "eta_l"))
        and math.isfinite(_number(row, "eta_l_gt"))
    ]
    hard_envelope_active_flags = [
        _number(row, "lambda_max_hard") < _number(row, "lambda_phy") - 1.0e-6
        for row in prior_rows
        if math.isfinite(_number(row, "lambda_max_hard"))
        and math.isfinite(_number(row, "lambda_phy"))
    ]
    pusher_active_flags = [
        _number(row, "pusher_throttle_external_active") > 0.5
        for row in prior_rows
        if math.isfinite(_number(row, "pusher_throttle_external_active"))
    ]
    pusher_handover_rows = [
        row for row in prior_rows
        if math.isfinite(_number(row, "selected_airspeed_mps"))
        and _number(row, "selected_airspeed_mps") >= va_target - 1.0
    ]
    pusher_handover_active_flags = [
        _number(row, "pusher_throttle_external_active") > 0.5
        for row in pusher_handover_rows
        if math.isfinite(_number(row, "pusher_throttle_external_active"))
    ]

    schema_version = max(_values(rows, "schema_version"), default=0.0)
    schedule_modes = sorted({
        row.get("schedule_mode", "") for row in rows
        if row.get("schedule_mode", "")
    })
    any_failsafe = any(row.get("failsafe") == "1" for row in rows)
    state_sequence_ok = _contains_in_order(
        states, ("HOLD_MC", "TRANSITION_FW", "HOLD_FW")
    )
    vtol_state_sequence_ok = _contains_in_order(vtol_states, (3, 1, 4))

    transition_time = math.nan
    if transition:
        t0 = _number(transition[0], "time_s")
        t1 = _number(transition[-1], "time_s")
        if math.isfinite(t0) and math.isfinite(t1):
            transition_time = t1 - t0

    max_altitude_error = max((abs(value) for value in altitude_errors), default=math.nan)
    altitude_rmse = _rmse(altitude_errors)
    max_descent_rate = max((-value for value in vz), default=math.nan)
    max_abs_alpha = max((abs(value) for value in alpha), default=math.nan)

    pass_failures: list[str] = []
    if schema_version < 24.0:
        pass_failures.append("schema_version_below_24")
    if "physics_prior_only" not in schedule_modes:
        pass_failures.append("schedule_mode_not_physics_prior_only")
    if not state_sequence_ok or not vtol_state_sequence_ok:
        pass_failures.append("transition_sequence_incomplete")
    if any_failsafe:
        pass_failures.append("px4_failsafe")
    if len(prior_rows) < 30:
        pass_failures.append("insufficient_prior_rows")
    if _fresh_fraction(prior_rows, "lambda_exec", "lambda_status_age_s") < 0.9:
        pass_failures.append("lambda_status_stale_or_missing")
    if _fresh_fraction(prior_rows, "eta_l", "airspeed_age_s") < 0.8:
        pass_failures.append("eta_l_missing_or_stale")
    if _fraction([_number(row, "eta_c_valid") > 0.5 for row in prior_rows]) < 0.5:
        pass_failures.append("eta_c_insufficient_coverage")
    if _range_fraction(lambda_phy, 0.0, 1.0) < 1.0:
        pass_failures.append("lambda_phy_out_of_range")
    if _range_fraction(lambda_command, 0.0, 1.0) < 1.0:
        pass_failures.append("lambda_command_out_of_range")
    if _range_fraction(lambda_hard, 0.0, 1.0) < 1.0:
        pass_failures.append("lambda_hard_out_of_range")
    if hard_violations and max(hard_violations) > 1.0e-6:
        pass_failures.append("command_exceeds_hard_limit")
    if not math.isfinite(altitude_rmse) or altitude_rmse > 3.0:
        pass_failures.append("altitude_rmse_excessive")
    if not math.isfinite(max_altitude_error) or max_altitude_error > 5.0:
        pass_failures.append("altitude_error_hard_violation")
    if math.isfinite(max_descent_rate) and max_descent_rate > 2.0:
        pass_failures.append("descent_rate_hard_violation")
    if math.isfinite(max_abs_alpha) and max_abs_alpha > 0.60:
        pass_failures.append("aoa_hard_violation")
    if not lambda_phy_peak_reached:
        pass_failures.append("lambda_phy_peak_not_reached")
    if not release_dwell_pass:
        pass_failures.append("full_allocation_release_dwell_not_met")
    if max(lambda_phy, default=0.0) < 0.30:
        pass_failures.append("lambda_phy_peak_too_low")
    if pusher_handover_rows and _fraction(pusher_handover_active_flags) < 0.8:
        pass_failures.append("pusher_pi_not_active_after_handover")
    if (
        math.isfinite(max(selected_airspeed, default=math.nan))
        and math.isfinite(va_target)
        and max(selected_airspeed) > va_target + 5.0
    ):
        pass_failures.append("selected_airspeed_runaway")

    plot_created = False
    plot_path = path.with_name("physics_prior_timeseries.png")
    if plot:
        plot_created = _plot_timeseries(rows, plot_path)

    return _json_safe({
        "source_csv": str(path.resolve()),
        "physics_prior_smoke_pass": not pass_failures,
        "pass_failures": pass_failures,
        "telemetry_schema_version": schema_version,
        "schedule_modes_seen": schedule_modes,
        "row_count": len(rows),
        "transition_row_count": len(transition),
        "prior_row_count": len(prior_rows),
        "state_sequence_ok": state_sequence_ok,
        "vtol_state_sequence_ok": vtol_state_sequence_ok,
        "any_failsafe": any_failsafe,
        "transition_time_s": transition_time,
        "altitude_rmse_m": altitude_rmse,
        "max_abs_altitude_error_m": max_altitude_error,
        "max_descent_rate_mps": max_descent_rate,
        "max_abs_alpha_rad": max_abs_alpha,
        "eta_l_valid_fraction": _fraction([
            _number(row, "eta_l_valid") > 0.5 for row in prior_rows
        ]),
        "eta_c_valid_fraction": _fraction([
            _number(row, "eta_c_valid") > 0.5 for row in prior_rows
        ]),
        "eta_l_early_mean": early_eta_l,
        "eta_l_late_mean": late_eta_l,
        "eta_c_early_mean": early_eta_c,
        "eta_c_late_mean": late_eta_c,
        "eta_l_gt_rmse_during_prior": _rmse(eta_l_gt_errors),
        "eta_l_gt_max_abs_error_during_prior": max(
            (abs(value) for value in eta_l_gt_errors), default=math.nan
        ),
        "selected_airspeed_mean_mps": _mean(selected_airspeed),
        "selected_airspeed_max_mps": max(selected_airspeed, default=math.nan),
        "pusher_external_active_fraction": _fraction(pusher_active_flags),
        "pusher_external_active_after_handover_fraction": _fraction(
            pusher_handover_active_flags
        ),
        "lambda_phy_early_mean": early_lambda,
        "lambda_phy_late_mean": late_lambda,
        "lambda_phy_peak_reached": lambda_phy_peak_reached,
        "full_allocation_dwell_max_s": full_allocation_dwell,
        "release_dwell_s_config": release_dwell_s,
        "release_dwell_pass": release_dwell_pass,
        "lambda_phy_post_peak_drop": lambda_phy_post_peak_drop,
        "lambda_phy_min": min(lambda_phy, default=math.nan),
        "lambda_phy_mean": _mean(lambda_phy),
        "lambda_phy_max": max(lambda_phy, default=math.nan),
        "lambda_max_hard_min": min(lambda_hard, default=math.nan),
        "lambda_max_hard_mean": _mean(lambda_hard),
        "lambda_command_mean": _mean(lambda_command),
        "lambda_command_max": max(lambda_command, default=math.nan),
        "lambda_exec_mean": _mean(lambda_exec),
        "lambda_exec_max": max(lambda_exec, default=math.nan),
        "lambda_tracking_rmse": _rmse(tracking_errors),
        "lambda_tracking_max_abs": max(
            (abs(value) for value in tracking_errors), default=math.nan
        ),
        "shield_projection_intervention_max": max(
            _values(prior_rows, "lambda_shield_projection_intervention"),
            default=math.nan,
        ),
        "hard_envelope_active_fraction": _fraction(hard_envelope_active_flags),
        "shield_total_intervention_mean": _mean(
            _values(prior_rows, "lambda_shield_total_intervention")
        ),
        "shield_emergency_recovery_fraction": _fraction([
            _number(row, "lambda_shield_emergency_recovery") > 0.5
            for row in prior_rows
        ]),
        "timeseries_plot": str(plot_path.resolve()) if plot_created else "",
    })


def analyze_directory(root: Path, *, plot: bool = True) -> dict[str, object]:
    run_summaries = []
    for telemetry in sorted(root.glob("run_*/telemetry.csv")):
        summary = analyze_csv(telemetry, plot=plot)
        summary_path = telemetry.with_name("physics_prior_summary.json")
        summary_path.write_text(
            json.dumps(summary, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        run_summaries.append(summary)
    if not run_summaries and (root / "telemetry.csv").exists():
        summary = analyze_csv(root / "telemetry.csv", plot=plot)
        (root / "physics_prior_summary.json").write_text(
            json.dumps(summary, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        run_summaries.append(summary)
    if not run_summaries:
        raise ValueError(f"no telemetry.csv files found under {root}")
    passes = sum(bool(item["physics_prior_smoke_pass"]) for item in run_summaries)
    aggregate = _json_safe({
        "source_root": str(root.resolve()),
        "physics_prior_repeat_pass": passes == len(run_summaries),
        "passes": passes,
        "total": len(run_summaries),
        "failed_runs": [
            item["source_csv"] for item in run_summaries
            if not item["physics_prior_smoke_pass"]
        ],
        "mean_lambda_phy_max": _mean([
            float(item["lambda_phy_max"]) for item in run_summaries
            if item["lambda_phy_max"] is not None
        ]),
        "mean_altitude_rmse_m": _mean([
            float(item["altitude_rmse_m"]) for item in run_summaries
            if item["altitude_rmse_m"] is not None
        ]),
        "runs": run_summaries,
    })
    (root / "physics_prior_repeat_summary.json").write_text(
        json.dumps(aggregate, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return aggregate


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    path = args.path
    result = (
        analyze_csv(path, plot=not args.no_plot)
        if path.is_file()
        else analyze_directory(path, plot=not args.no_plot)
    )
    text = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()

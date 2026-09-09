#!/usr/bin/env python3
"""Freeze the 5/5 Physics Prior Only smoke qualification record.

This script deliberately does not introduce new pass criteria.  It only
combines already-produced ``physics_prior_repeat_summary.json`` files and
records whether the frozen smoke analyzer reports five passing runs.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def _load_summary(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"missing physics prior summary: {path}")
    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if "runs" not in payload or not isinstance(payload["runs"], list):
        raise ValueError(f"summary does not contain a runs list: {path}")
    return payload


def build_record(paths: list[Path]) -> dict[str, Any]:
    runs: list[dict[str, Any]] = []
    source_summaries: list[str] = []
    source_repeat_passes: list[bool | None] = []
    for path in paths:
        summary = _load_summary(path)
        source_summaries.append(str(path))
        source_repeat_passes.append(summary.get("physics_prior_repeat_pass"))
        for run in summary["runs"]:
            copied = dict(run)
            copied.setdefault("source_summary", str(path))
            runs.append(copied)

    pass_flags = [bool(run.get("physics_prior_smoke_pass")) for run in runs]
    failures = [
        {
            "index": index,
            "source_summary": run.get("source_summary"),
            "pass_failures": run.get("pass_failures", []),
        }
        for index, (run, passed) in enumerate(zip(runs, pass_flags), start=1)
        if not passed
    ]
    status = "DONE_FROZEN" if len(runs) == 5 and all(pass_flags) else "NOT_FROZEN"
    return {
        "stage": "Physics Prior Only",
        "status": status,
        "valid_runs": sum(pass_flags),
        "total_runs": len(runs),
        "freeze_rule": "status is DONE_FROZEN iff exactly five runs exist and every run has physics_prior_smoke_pass=true",
        "source_summaries": source_summaries,
        "source_repeat_passes": source_repeat_passes,
        "failures": failures,
        "runs": runs,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summaries", nargs="+", type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("data/evaluation/physics_prior_only_qualification_v1.json"),
    )
    args = parser.parse_args()

    record = build_record(args.summaries)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(record, indent=2, sort_keys=True))
    return 0 if record["status"] == "DONE_FROZEN" else 1


if __name__ == "__main__":
    raise SystemExit(main())

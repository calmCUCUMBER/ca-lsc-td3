#!/usr/bin/env python3
"""Fail closed if the frozen schema-13 F1 architecture has drifted."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def check(manifest_path: Path) -> dict[str, Any]:
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    errors: list[str] = []

    if manifest.get('status') != 'frozen':
        errors.append('manifest status is not frozen')
    if manifest.get('telemetry_schema_version') != 13:
        errors.append('frozen telemetry schema must be 13')
    mapping = manifest.get('transition_allocation', {})
    if mapping.get('lambda_attitude_blend_start') != 0.1:
        errors.append('frozen lambda attitude blend start must be 0.1')
    if mapping.get('lambda_attitude_blend_full') != 0.9:
        errors.append('frozen lambda attitude blend full must be 0.9')

    verification_path = PROJECT_ROOT / manifest['verification_summary']
    if not verification_path.is_file():
        errors.append(f'missing verification summary: {verification_path}')
    else:
        verification = json.loads(
            verification_path.read_text(encoding='utf-8')
        )
        points = verification.get('points', [])
        if not verification.get('synchronization_pass'):
            errors.append('nine-point synchronization_pass is not true')
        if verification.get('completed_point_count') != 9:
            errors.append('architecture verification is not 9/9 complete')
        if len(points) != 9:
            errors.append(f'expected 9 verification points, got {len(points)}')
        if any(not point.get('architecture_verified') for point in points):
            errors.append('at least one selected verification point is invalid')

    checked_files = []
    for relative, expected_hash in manifest.get('critical_file_sha256', {}).items():
        path = PROJECT_ROOT / relative
        if not path.is_file():
            errors.append(f'missing frozen file: {relative}')
            continue
        actual_hash = _sha256(path)
        checked_files.append({
            'path': relative,
            'expected_sha256': expected_hash,
            'actual_sha256': actual_hash,
            'match': actual_hash == expected_hash,
        })
        if actual_hash != expected_hash:
            errors.append(f'frozen file changed: {relative}')

    return {
        'freeze_check_pass': not errors,
        'manifest': str(manifest_path.resolve()),
        'verification_summary': str(verification_path.resolve()),
        'checked_file_count': len(checked_files),
        'checked_files': checked_files,
        'errors': errors,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--manifest',
        type=Path,
        default=PROJECT_ROOT / 'config' / 'transition_allocation_freeze.json',
    )
    arguments = parser.parse_args()
    result = check(arguments.manifest.resolve())
    print(json.dumps(result, indent=2, allow_nan=False))
    if not result['freeze_check_pass']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()

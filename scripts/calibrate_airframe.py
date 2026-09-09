#!/usr/bin/env python3
"""Generate model-based calibration artifacts for the stock PX4 quadplane."""

import argparse
from pathlib import Path

from ca_lsc_td3.physics.airframe_calibration import (
    parse_standard_vtol_sdf,
    write_calibration,
)


def main() -> None:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--sdf',
        type=Path,
        default=(
            project_root / 'PX4-Autopilot' / 'Tools' / 'simulation'
            / 'gz' / 'models' / 'standard_vtol' / 'model.sdf'
        ),
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=project_root / 'data' / 'calibration' / 'model_based',
    )
    arguments = parser.parse_args()
    calibration = parse_standard_vtol_sdf(arguments.sdf)
    summary = write_calibration(calibration, arguments.output_dir)
    print(f"mass_kg={summary['mass_kg']:.6f}")
    print(f"wing_area_m2={summary['wing_area_m2']:.6f}")
    print(f"hover_omega_rad_s={summary['hover_omega_rad_s']:.3f}")
    print(f"output_dir={arguments.output_dir.resolve()}")


if __name__ == '__main__':
    main()

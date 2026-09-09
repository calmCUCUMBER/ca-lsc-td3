#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
conda_script="${VTOL_CONDA_SH:-/home/weicheng/anaconda3/etc/profile.d/conda.sh}"

if [[ "${CONDA_DEFAULT_ENV:-}" != "vtol_nav" ]]; then
    if [[ ! -f "${conda_script}" ]]; then
        echo "Cannot find Conda initialization script: ${conda_script}" >&2
        exit 1
    fi
    # shellcheck disable=SC1090
    source "${conda_script}"
    conda activate vtol_nav
fi

unset GZ_SIM_SYSTEM_PLUGIN_PATH
unset GZ_SIM_RESOURCE_PATH

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${project_root}/ros2_ws/install/setup.bash"
set -u

export VTOL_SIM_ROOT="${project_root}"
export PYTHONNOUSERSITE=1

exec python -m vtol_rl.preflight_calibration "$@"

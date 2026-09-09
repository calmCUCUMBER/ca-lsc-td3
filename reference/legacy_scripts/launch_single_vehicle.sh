#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

unset GZ_SIM_SYSTEM_PLUGIN_PATH
unset GZ_SIM_RESOURCE_PATH

source /opt/ros/humble/setup.bash
source "${project_root}/ros2_ws/install/setup.bash"
set -u

export PX4_AUTOPILOT_DIR="${PX4_AUTOPILOT_DIR:-${project_root}/PX4-Autopilot}"

exec ros2 launch vtol_bringup single_vehicle.launch.py "$@"

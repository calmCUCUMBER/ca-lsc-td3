#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"

if ! command -v vcs >/dev/null 2>&1; then
  echo "Missing 'vcs'. Install vcstool in the active environment first." >&2
  exit 1
fi

cd "${repo_root}"
vcs import . < config/dependencies.repos

apply_once() {
  local checkout="$1"
  local patch_file="$2"

  if git -C "${checkout}" apply --check "${patch_file}"; then
    git -C "${checkout}" apply "${patch_file}"
    echo "Applied ${patch_file}"
  elif git -C "${checkout}" apply --reverse --check "${patch_file}"; then
    echo "Already applied: ${patch_file}"
  else
    echo "Patch does not match the pinned checkout: ${patch_file}" >&2
    exit 1
  fi
}

apply_once \
  "${repo_root}/PX4-Autopilot" \
  "${repo_root}/patches/px4-ca-lsc-transition-allocation.patch"
apply_once \
  "${repo_root}/ros2_ws/src/px4_msgs" \
  "${repo_root}/patches/px4-msgs-ca-lsc-topics.patch"

git -C "${repo_root}/PX4-Autopilot" submodule update --init --recursive

echo "Pinned dependencies and CA-LSC patches are ready."

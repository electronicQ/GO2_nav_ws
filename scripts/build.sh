#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(dirname "${script_dir}")"

if [[ "$(uname -m)" != "x86_64" ]]; then
  echo "This repository currently ships x86_64 third-party binaries." >&2
  echo "Rebuild third_party/install/gtsam and teaserpp on $(uname -m) first." >&2
  exit 1
fi

if [[ ! -d "${workspace}/third_party/install/gtsam" ]]; then
  echo "Missing third_party/install/gtsam." >&2
  exit 1
fi
if [[ ! -d "${workspace}/third_party/install/teaserpp" ]]; then
  echo "Missing third_party/install/teaserpp." >&2
  exit 1
fi
if [[ ! -d "${workspace}/third_party/ros-humble" ]]; then
  echo "Missing third_party/ros-humble." >&2
  exit 1
fi

set +u
export GO2_SKIP_WORKSPACE_INSTALL=1
source "${workspace}/scripts/setup_env.sh"
unset GO2_SKIP_WORKSPACE_INSTALL
set -u
cd "${workspace}"

if ! ros2 pkg prefix voxel_map_tools >/dev/null 2>&1; then
  echo "voxel_map_tools binary package was not found." >&2
  echo "Install ros-humble-voxel-map-tools before deploying the workspace." >&2
  exit 1
fi

colcon build \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  "$@"

echo
echo "Build complete. Reload the environment:"
echo "  source ${workspace}/scripts/setup_env.sh"

#!/usr/bin/env bash
# Install official Unitree dependencies; never start a robot client or controller.
set -euo pipefail

unitree_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unitree_workspace="$(dirname "${unitree_script_dir}")"
unitree_install_root="${UNITREE_OFFICIAL_ROOT:-$HOME/unitree_official}"
unitree_jobs="${UNITREE_BUILD_JOBS:-2}"
unitree_skip_apt=0
unitree_check_only=0

usage() {
  cat <<'EOF'
Usage: scripts/install_unitree_sdk.sh [options]
  --prefix PATH  Installation root (default: $UNITREE_OFFICIAL_ROOT or ~/unitree_official)
  --jobs N       Build parallelism (default: $UNITREE_BUILD_JOBS or 2)
  --skip-apt     Skip system package installation (requires dependencies already installed)
  --check        Only check the installation; no downloads, builds, or robot requests
  -h, --help    Show this help

Installs pinned official ROS 2 message packages, C++ SDK2 and Python SDK2.
Python dependencies stay in a virtual environment with access to ROS system packages.
Run as your normal login user; only apt uses sudo. No network interface is changed.
EOF
}
while (($#)); do
  case "$1" in
    --prefix|--jobs)
      if (($# < 2)) || [[ -z "$2" ]]; then echo "Missing value for $1" >&2; exit 2; fi
      if [[ "$1" == --prefix ]]; then unitree_install_root="$2"; else unitree_jobs="$2"; fi
      shift 2 ;;
    --skip-apt) unitree_skip_apt=1; shift ;;
    --check) unitree_check_only=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done
if [[ ! "$unitree_jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo '--jobs must be a positive integer' >&2; exit 2
fi
unitree_install_root="$(realpath -m -- "$unitree_install_root")"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Install ROS 2 Humble first (/opt/ros/humble/setup.bash is missing).' >&2; exit 1
fi
if [[ "$unitree_check_only" == 0 ]]; then
  source /etc/os-release
  if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 22.04 ]]; then
    echo 'This installer is validated on Ubuntu 22.04 with ROS 2 Humble.' >&2; exit 1
  fi
  case "$(uname -m)" in
    x86_64|aarch64) ;;
    *) echo 'Official SDK binaries require x86_64 or aarch64.' >&2; exit 1 ;;
  esac
fi

# Revisions verified with the current GO2 SportClient navigation route.
unitree_ros_ref=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
unitree_cpp_ref=aaf69cb56c9d5ed63d8b284008ec3a8ab999bec1
unitree_python_ref=814556d15970dd2ecf1c9984e845ca02ab07e206

if [[ "$unitree_check_only" == 0 ]]; then
  if [[ "$unitree_skip_apt" == 0 ]]; then
    unitree_sudo=()
    if [[ "$(id -u)" != 0 ]]; then unitree_sudo=(sudo); fi
    "${unitree_sudo[@]}" apt-get update
    "${unitree_sudo[@]}" apt-get install -y \
      git cmake build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libfmt-dev \
      python3-venv python3-pip python3-setuptools python3-wheel python3-numpy \
      python3-colcon-common-extensions \
      ros-humble-rclpy ros-humble-geometry-msgs ros-humble-std-msgs \
      ros-humble-rosidl-default-generators ros-humble-rosidl-generator-dds-idl \
      ros-humble-rmw-cyclonedds-cpp
  fi
  mkdir -p "$unitree_install_root"

  checkout_official() {
    local repo="$1" ref="$2" directory="$unitree_install_root/$1"
    local url="https://github.com/unitreerobotics/$1.git" existing
    if [[ ! -e "$directory" ]]; then
      git clone --depth 1 "$url" "$directory"
    fi
    if [[ ! -d "$directory/.git" ]]; then
      echo "Not a Git checkout: $directory; refusing to overwrite it." >&2; return 1
    fi
    existing="$(git -C "$directory" remote get-url origin)"
    if [[ "${existing%.git}" != "${url%.git}" ]]; then
      echo "Unexpected Git origin in $directory: $existing" >&2; return 1
    fi
    if ! git -C "$directory" diff --quiet || ! git -C "$directory" diff --cached --quiet; then
      echo "Local changes in $directory; refusing to replace them." >&2; return 1
    fi
    if [[ "$(git -C "$directory" rev-parse HEAD 2>/dev/null || true)" != "$ref" ]]; then
      git -C "$directory" fetch --depth 1 origin "$ref"
      git -C "$directory" checkout --detach "$ref"
    fi
  }
  checkout_official unitree_ros2 "$unitree_ros_ref"
  checkout_official unitree_sdk2 "$unitree_cpp_ref"
  checkout_official unitree_sdk2_python "$unitree_python_ref"

  cmake -S "$unitree_install_root/unitree_sdk2" -B "$unitree_install_root/unitree_sdk2/build" \
    -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$unitree_install_root/sdk2_install"
  cmake --build "$unitree_install_root/unitree_sdk2/build" --parallel "$unitree_jobs"
  cmake --install "$unitree_install_root/unitree_sdk2/build"

  # System pip is usable inside --without-pip venv, including hosts missing ensurepip.
  if [[ ! -x "$unitree_install_root/venv/bin/python" ]]; then
    python3 -m venv --without-pip --system-site-packages "$unitree_install_root/venv"
  fi
  CYCLONEDDS_HOME="$unitree_install_root/sdk2_install" \
    "$unitree_install_root/venv/bin/python" -m pip install \
      'numpy<2' 'opencv-python<4.12' 'cyclonedds==0.10.2' \
      -e "$unitree_install_root/unitree_sdk2_python"
fi

# Avoid inheriting an older workspace overlay while compiling official messages.
set +u
GO2_SKIP_WORKSPACE_INSTALL=1 source "$unitree_workspace/scripts/setup_env.sh"
set -u
if [[ "$unitree_check_only" == 0 ]]; then
  CMAKE_BUILD_PARALLEL_LEVEL="$unitree_jobs" MAKEFLAGS="-j$unitree_jobs" \
    colcon --log-base "$unitree_install_root/ros2_ws/log" build \
      --base-paths "$unitree_install_root/unitree_ros2/cyclonedds_ws/src/unitree" \
      --build-base "$unitree_install_root/ros2_ws/build" \
      --install-base "$unitree_install_root/ros2_ws/install" \
      --packages-select unitree_api unitree_go unitree_hg --executor sequential
fi

for unitree_required in venv/bin/python sdk2_install/lib/libunitree_sdk2.a ros2_ws/install/local_setup.bash; do
  if [[ ! -e "$unitree_install_root/$unitree_required" ]]; then
    echo "Missing $unitree_install_root/$unitree_required; run this script without --check." >&2
    exit 1
  fi
done
set +u
source "$unitree_install_root/ros2_ws/install/local_setup.bash"
set -u
export CYCLONEDDS_HOME="$unitree_workspace/third_party/ros-humble/opt/ros/humble"
if [[ ! -d "$CYCLONEDDS_HOME" ]]; then export CYCLONEDDS_HOME=/opt/ros/humble; fi
"$unitree_install_root/venv/bin/python" - "$unitree_install_root" <<'PY'
import importlib.metadata
import pathlib
import sys
import rclpy
from geometry_msgs.msg import Twist
from unitree_api.msg import Request
from unitree_go.msg import SportModeState
from unitree_sdk2py.go2.sport.sport_client import SportClient

# Import only: do not initialize DDS channels or instantiate SportClient.
print('Official SDK and ROS imports OK; no robot requests sent.')
for name in ('unitree-sdk2py', 'cyclonedds', 'numpy', 'opencv-python'):
    print(f'  {name}={importlib.metadata.version(name)}')
print('  Python:', sys.executable)
PY
if [[ "$unitree_check_only" == 0 ]]; then
  python3 - "$unitree_install_root" <<'PY'
import json
import pathlib
import subprocess
import sys
root = pathlib.Path(sys.argv[1])
revisions = {name: subprocess.check_output(
    ['git', '-C', str(root / name), 'rev-parse', 'HEAD'], text=True).strip()
    for name in ('unitree_ros2', 'unitree_sdk2', 'unitree_sdk2_python')}
(root / 'installed_revisions.json').write_text(json.dumps(revisions, indent=2) + '\n')
PY
fi
echo "Official Unitree installation ready: $unitree_install_root"
echo 'No robot controller was started. The network configuration was not changed.'
if [[ "$unitree_install_root" != "$HOME/unitree_official" ]]; then
  echo "Set UNITREE_OFFICIAL_ROOT to $unitree_install_root before sourcing scripts/setup_env.sh."
fi

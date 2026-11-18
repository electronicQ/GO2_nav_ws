#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(dirname "${script_dir}")"

if [[ -f /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
    echo "This deployment is validated for Ubuntu 22.04; found ${PRETTY_NAME:-unknown}." >&2
    exit 1
  fi
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  echo "Install ROS 2 Humble on Ubuntu 22.04 first." >&2
  exit 1
fi

if [[ "$(id -u)" -eq 0 ]]; then
  sudo_cmd=()
else
  sudo_cmd=(sudo)
fi

echo "Installing build tools and system libraries..."
"${sudo_cmd[@]}" apt-get update
"${sudo_cmd[@]}" apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-numpy \
  python3-pip \
  python3-rosdep \
  python3-serial \
  python3-vcstool \
  python3-yaml \
  libboost-all-dev \
  libeigen3-dev \
  libgflags-dev \
  libgoogle-glog-dev \
  libpcap-dev \
  libpcl-dev \
  libsuitesparse-dev \
  libyaml-cpp-dev

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  "${sudo_cmd[@]}" rosdep init
fi
rosdep update

set +u
source /opt/ros/humble/setup.bash
set -u
cd "${workspace}"
# voxel_map_tools is delivered as a prebuilt Debian package. It is intentionally
# absent from the public source tree, so rosdep cannot resolve its custom key.
rosdep install \
  --from-paths src \
  --ignore-src \
  --skip-keys voxel_map_tools \
  --rosdistro humble \
  -r -y

echo
echo "Installing proprietary voxel_map_tools binary..."
"${workspace}/scripts/install_voxel_map_tools_deb.sh"

echo
echo "Installing official Unitree ROS 2 and SDK dependencies..."
"${workspace}/scripts/install_unitree_sdk.sh"

echo
echo "Dependencies installed."
echo "Next:"
echo "  source ${workspace}/scripts/setup_env.sh"
echo "  ${workspace}/scripts/build.sh"

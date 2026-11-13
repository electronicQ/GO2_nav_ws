#!/usr/bin/env bash
# Source this file in every shell:
#   source /path/to/GO2_nav_ws/scripts/setup_env.sh
#
# Override these before sourcing when needed:
#   GO2_NETWORK_INTERFACE=enp... ROS_DOMAIN_ID=... GO2_NAV_WS=...

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Source this script instead of executing it: source ${BASH_SOURCE[0]}" >&2
  exit 1
fi

setup_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
setup_workspace="$(dirname "${setup_script_dir}")"
export GO2_NAV_WORKSPACE="${GO2_NAV_WORKSPACE:-${setup_workspace}}"
unset setup_script_dir setup_workspace
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# Build a deterministic overlay. Stale ROS workspaces in the caller's shell
# must not override packages or third-party libraries from this workspace.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH
export CMAKE_PREFIX_PATH=""
export PYTHONPATH=""

vendored_gtsam="${GO2_NAV_WORKSPACE}/third_party/install/gtsam"
vendored_teaserpp="${GO2_NAV_WORKSPACE}/third_party/install/teaserpp"
if [[ -d "${vendored_gtsam}" && -d "${vendored_teaserpp}" ]]; then
  export CMAKE_PREFIX_PATH="${vendored_gtsam}:${vendored_teaserpp}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${vendored_gtsam}/lib:${vendored_teaserpp}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
unset vendored_gtsam vendored_teaserpp

if [[ -n "${GO2_NETWORK_INTERFACE:-}" ]]; then
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${GO2_NETWORK_INTERFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
fi

source /opt/ros/humble/setup.bash

vendored_dds_idl="${GO2_NAV_WORKSPACE}/third_party/ros-humble/opt/ros/humble/share/rosidl_generator_dds_idl/local_setup.bash"
if [[ -f "${vendored_dds_idl}" ]]; then
  source "${vendored_dds_idl}"
fi
vendored_cyclone="${GO2_NAV_WORKSPACE}/third_party/ros-humble/opt/ros/humble/share/rmw_cyclonedds_cpp/local_setup.bash"
if [[ -f "${vendored_cyclone}" ]]; then
  source "${vendored_cyclone}"
  vendored_ros_prefix="${GO2_NAV_WORKSPACE}/third_party/ros-humble/opt/ros/humble"
  vendored_system_lib="${GO2_NAV_WORKSPACE}/third_party/ros-humble/usr/lib"
  export LD_LIBRARY_PATH="${vendored_ros_prefix}/lib:${vendored_ros_prefix}/lib/x86_64-linux-gnu:${vendored_system_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  if [[ -d "${GO2_NAV_WORKSPACE}/third_party/ros-humble/usr/share/GraphicsMagick-1.3.38/config" ]]; then
    export MAGICK_CONFIGURE_PATH="${GO2_NAV_WORKSPACE}/third_party/ros-humble/usr/share/GraphicsMagick-1.3.38/config${MAGICK_CONFIGURE_PATH:+:${MAGICK_CONFIGURE_PATH}}"
  fi
  unset vendored_system_lib
  unset vendored_ros_prefix
fi

if [[ "${GO2_SKIP_WORKSPACE_INSTALL:-0}" == "1" ]]; then
  :
elif [[ -f "${GO2_NAV_WORKSPACE}/install/setup.bash" ]]; then
  source "${GO2_NAV_WORKSPACE}/install/setup.bash"
else
  echo "Workspace install/setup.bash not found; build the workspace first." >&2
fi

echo "GO2 navigation environment loaded"
echo "  workspace: ${GO2_NAV_WORKSPACE}"
echo "  ROS domain: ${ROS_DOMAIN_ID}"
echo "  RMW: ${RMW_IMPLEMENTATION}"
if [[ -n "${GO2_NETWORK_INTERFACE:-}" ]]; then
  echo "  Go2 network interface: ${GO2_NETWORK_INTERFACE}"
fi

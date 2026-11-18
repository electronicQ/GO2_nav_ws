#!/usr/bin/env bash
# Source in both terminals for the independent official-SDK control test.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Source this file: source scripts/setup_unitree_official.sh" >&2
  exit 1
fi
unitree_test_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export UNITREE_OFFICIAL_ROOT="${UNITREE_OFFICIAL_ROOT:-$HOME/unitree_official}"
if [[ ! -f "$UNITREE_OFFICIAL_ROOT/ros2_ws/install/local_setup.bash" ||
      ! -f "$UNITREE_OFFICIAL_ROOT/venv/bin/activate" ]]; then
  echo "Official Unitree installation missing under $UNITREE_OFFICIAL_ROOT" >&2
  echo "Run: scripts/install_unitree_sdk.sh" >&2
  unset unitree_test_script_dir
  return 1
fi
source "$unitree_test_script_dir/setup_env.sh"
source "$UNITREE_OFFICIAL_ROOT/ros2_ws/install/local_setup.bash"
source "$UNITREE_OFFICIAL_ROOT/venv/bin/activate"
# Use the same DDS library as ROS 2 when SDK and rclpy share a process.
export CYCLONEDDS_HOME="$GO2_NAV_WORKSPACE/third_party/ros-humble/opt/ros/humble"
unset unitree_test_script_dir
echo "Official Unitree ROS 2 overlay and Python SDK loaded (no control process started)"

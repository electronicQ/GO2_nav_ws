#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [finish|finish_no_ba]" >&2
  exit 2
fi

parameter="${1:-finish}"
if [[ "${parameter}" != "finish" && "${parameter}" != "finish_no_ba" ]]; then
  echo "Pipeline parameter must be finish or finish_no_ba" >&2
  exit 2
fi

ros2 param set /voxelslam "${parameter}" true
echo "Requested ${parameter}=true. Wait for: Saved optimized SC scan map"

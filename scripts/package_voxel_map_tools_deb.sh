#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(dirname "${script_dir}")"
source_dir="${VOXEL_MAP_TOOLS_SOURCE_DIR:-${workspace}/private/voxel_map_tools_src}"
output_dir_override="${VOXEL_MAP_TOOLS_DIST_DIR:-}"
package_name="ros-humble-voxel-map-tools"

if [[ ! -f "${source_dir}/package.xml" || ! -f "${source_dir}/CMakeLists.txt" ]]; then
  echo "Private voxel_map_tools source was not found at ${source_dir}." >&2
  echo "Set VOXEL_MAP_TOOLS_SOURCE_DIR to its private checkout." >&2
  exit 1
fi

version="${VOXEL_MAP_TOOLS_VERSION:-$(sed -n 's:.*<version>\([^<]*\)</version>.*:\1:p' "${source_dir}/package.xml" | head -n 1)}"
if [[ -z "${version}" ]]; then
  echo "Unable to determine voxel_map_tools version." >&2
  exit 1
fi

architecture="$(dpkg --print-architecture)"
output_dir="${output_dir_override:-${workspace}/third_party/debs/${architecture}}"
packaging_root="${workspace}/.packaging/voxel_map_tools"
build_dir="${packaging_root}/build"
stage_root="${packaging_root}/stage"
ros_prefix="${stage_root}/opt/ros/humble"
binary="${ros_prefix}/lib/voxel_map_tools/process_map"
deb_path="${output_dir}/${package_name}_${version}_${architecture}.deb"

command -v cmake >/dev/null
command -v dpkg-deb >/dev/null
command -v dpkg-shlibdeps >/dev/null
command -v strip >/dev/null

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  exit 1
fi

cmake -E remove_directory "${packaging_root}"
cmake -E make_directory "${build_dir}" "${output_dir}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

cmake \
  -S "${source_dir}" \
  -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${ros_prefix}" \
  -DBUILD_TESTING=OFF \
  -DCMAKE_CXX_FLAGS_RELEASE="-O2 -DNDEBUG" \
  -DCMAKE_EXE_LINKER_FLAGS_RELEASE="-s"
cmake --build "${build_dir}" --parallel "$(nproc)"
cmake --install "${build_dir}"

if [[ ! -x "${binary}" ]]; then
  echo "Expected binary was not installed: ${binary}" >&2
  exit 1
fi

# The public binary package must not expose the internal processing library,
# development CMake files, headers or Python caches.
cmake -E rm -f "${ros_prefix}/lib/libmap_processing.a"
cmake -E remove_directory "${ros_prefix}/share/voxel_map_tools/cmake"
cmake -E remove_directory "${ros_prefix}/share/voxel_map_tools/hook"
cmake -E remove_directory "${ros_prefix}/share/voxel_map_tools/launch/__pycache__"
cmake -E remove_directory "${ros_prefix}/share/voxel_map_tools/environment"
cmake -E remove_directory "${ros_prefix}/share/ament_index/resource_index/parent_prefix_path"
find "${ros_prefix}/share/voxel_map_tools" -maxdepth 1 -type f \
  \( -name 'local_setup.*' -o -name 'package.*' \) \
  ! -name 'package.xml' -delete
find "${ros_prefix}" -type f -path '*/__pycache__/*' -delete
find "${ros_prefix}" -depth -type d -name __pycache__ -empty -delete
find "${ros_prefix}" -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' \
  -o -name '*.cc' -o -name '*.cxx' -o -name '*.a' -o -name '*.pyc' \) -delete

strip --strip-all --remove-section=.comment --remove-section=.note.GNU-stack "${binary}"

if find "${ros_prefix}" -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.a' \) | grep -q .; then
  echo "Source or development artifacts remain in the package stage." >&2
  exit 1
fi

# Replace the development README with usage-only documentation. Internal build
# instructions and source-tree details are not part of the customer artifact.
cat > "${ros_prefix}/share/voxel_map_tools/README.md" <<'EOF'
# voxel_map_tools binary edition

This package processes a Voxel-SLAM v4 scan map and writes a Nav2 raster map,
fused point clouds, and processing statistics into the map's `processed/`
directory. No source tree is required.

```bash
source /opt/ros/humble/setup.bash
ros2 launch voxel_map_tools process_map.launch.py \
  input_map:=/absolute/path/to/map_package \
  overwrite:=false
```

The installed default configuration is:

```text
/opt/ros/humble/share/voxel_map_tools/config/default.yaml
```

Copy this YAML before editing it, then use the private copy:

```bash
ros2 launch voxel_map_tools process_map.launch.py \
  config:=/absolute/path/to/custom.yaml \
  input_map:=/absolute/path/to/map_package
```

`overwrite:=true` is required to replace outputs already owned by this tool.
The calibration argument can override the LiDAR ray origin used by dynamic
point removal:

```bash
ros2 launch voxel_map_tools process_map.launch.py \
  calibration_file:=/absolute/path/to/current_robot.yaml
```

Use `ros2 launch voxel_map_tools process_map.launch.py --show-args` for the
available launch arguments. The command exits after processing.
EOF

debian_metadata="${stage_root}/debian"
cmake -E make_directory "${debian_metadata}"
cat > "${debian_metadata}/control" <<EOF
Source: ${package_name}
Section: misc
Priority: optional
Maintainer: gzq <gzq@example.com>
Package: ${package_name}
Architecture: ${architecture}
Description: Binary GO2 voxel scan-map processing tool
 Offline processing for voxel SLAM v4 maps and Nav2 map generation.
EOF

shlib_depends="$(
  cd "${stage_root}"
  LC_ALL=C dpkg-shlibdeps -O opt/ros/humble/lib/voxel_map_tools/process_map \
    | sed -n 's/^shlibs:Depends=//p'
)"
if [[ -z "${shlib_depends}" ]]; then
  echo "Unable to determine runtime shared-library dependencies." >&2
  exit 1
fi
runtime_depends="${shlib_depends}, python3-yaml, ros-humble-launch, ros-humble-launch-ros"
installed_size="$(du -sk "${stage_root}" | awk '{print $1}')"

cmake -E remove_directory "${debian_metadata}"
cmake -E make_directory "${stage_root}/DEBIAN"
cat > "${stage_root}/DEBIAN/control" <<EOF
Package: ${package_name}
Version: ${version}
Architecture: ${architecture}
Maintainer: gzq <gzq@example.com>
Installed-Size: ${installed_size}
Depends: ${runtime_depends}
Section: misc
Priority: optional
Description: Binary GO2 voxel scan-map processing tool
 Offline processing for voxel SLAM v4 maps and Nav2 map generation.
EOF

(
  cd "${stage_root}"
  find opt -type f -print0 | sort -z | xargs -0 md5sum
) > "${stage_root}/DEBIAN/md5sums"

cmake -E rm -f "${deb_path}" "${deb_path}.sha256"
dpkg-deb --build --root-owner-group "${stage_root}" "${deb_path}"
(
  cd "${output_dir}"
  sha256sum "$(basename "${deb_path}")"
) > "${deb_path}.sha256"

echo
echo "Built proprietary binary package:"
echo "  ${deb_path}"
echo "  ${deb_path}.sha256"
echo
echo "Install on a compatible Ubuntu 22.04 target:"
echo "  sudo apt install ${deb_path}"

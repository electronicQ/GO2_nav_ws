#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(dirname "${script_dir}")"
package_name="ros-humble-voxel-map-tools"
architecture="$(dpkg --print-architecture)"

deb_path="${VOXEL_MAP_TOOLS_DEB:-}"
if [[ -z "${deb_path}" ]]; then
  deb_dir="${workspace}/third_party/debs/${architecture}"
  shopt -s nullglob
  candidates=("${deb_dir}/${package_name}"_*_"${architecture}".deb)
  shopt -u nullglob
  if [[ "${#candidates[@]}" -eq 0 ]]; then
    echo "No proprietary ${package_name} package was found for ${architecture}." >&2
    echo "Place the .deb in ${deb_dir} or set VOXEL_MAP_TOOLS_DEB." >&2
    exit 1
  fi
  deb_path="$(printf '%s\n' "${candidates[@]}" | sort -V | tail -n 1)"
fi

if [[ ! -f "${deb_path}" ]]; then
  echo "voxel_map_tools Debian package not found: ${deb_path}" >&2
  exit 1
fi

deb_name="$(dpkg-deb -f "${deb_path}" Package)"
deb_architecture="$(dpkg-deb -f "${deb_path}" Architecture)"
deb_version="$(dpkg-deb -f "${deb_path}" Version)"
if [[ "${deb_name}" != "${package_name}" ]]; then
  echo "Unexpected Debian package name: ${deb_name}" >&2
  exit 1
fi
if [[ "${deb_architecture}" != "${architecture}" ]]; then
  echo "Package architecture ${deb_architecture} does not match host ${architecture}." >&2
  exit 1
fi

if [[ "${1:-}" == "--print-package" ]]; then
  echo "${deb_path}"
  exit 0
fi

if [[ "$(id -u)" -eq 0 ]]; then
  sudo_cmd=()
else
  sudo_cmd=(sudo)
fi

echo "  package: ${deb_path}"
echo "  version: ${deb_version}"
echo "  architecture: ${deb_architecture}"

installed_status="$(
  dpkg-query -W -f='${Status}' "${package_name}" 2>/dev/null || true
)"
installed_version="$(
  dpkg-query -W -f='${Version}' "${package_name}" 2>/dev/null || true
)"

if [[ "${installed_status}" == "install ok installed" \
  && "${installed_version}" == "${deb_version}" \
  && "${VOXEL_MAP_TOOLS_FORCE_REINSTALL:-0}" != "1" ]]; then
  echo "  already installed; skipping reinstall."
else
  "${sudo_cmd[@]}" apt-get install -y "${deb_path}"
fi

installed_status="$(
  dpkg-query -W -f='${Status}' "${package_name}" 2>/dev/null || true
)"
installed_version="$(
  dpkg-query -W -f='${Version}' "${package_name}" 2>/dev/null || true
)"
if [[ "${installed_status}" != "install ok installed" \
  || "${installed_version}" != "${deb_version}" ]]; then
  echo "Failed to install ${package_name} ${deb_version}." >&2
  exit 1
fi

echo "  installed successfully."

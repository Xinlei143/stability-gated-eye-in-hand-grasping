#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd "${script_dir}/.." && pwd)
source_root=${GAZEBO_GRASP_FIX_SOURCE:-"${repo_root}/.external/gazebo-pkgs"}
build_root=${GAZEBO_GRASP_FIX_BUILD:-"${repo_root}/.external/gazebo-grasp-build"}
install_root=${GAZEBO_GRASP_FIX_INSTALL:-"${repo_root}/.external/gazebo-grasp-install"}
commit=d217a4cd3045cb9622936784658ea687e0e6b69f
remote=https://github.com/JenniferBuehler/gazebo-pkgs.git

if [[ ! -d "${source_root}/.git" ]]; then
  mkdir -p "$(dirname "${source_root}")"
  git clone "${remote}" "${source_root}"
fi
if ! git -C "${source_root}" cat-file -e "${commit}^{commit}" 2>/dev/null; then
  git -C "${source_root}" fetch --quiet --depth 1 origin "${commit}"
fi
git -C "${source_root}" checkout --detach "${commit}"

patch_file="${repo_root}/patches/gazebo_grasp_plugin/0001-standalone-gazebo11-cmake.patch"
if ! grep -q 'find_package(Protobuf REQUIRED)' "${source_root}/gazebo_grasp_plugin/CMakeLists.txt"; then
  git -C "${source_root}" apply --check "${patch_file}"
  git -C "${source_root}" apply "${patch_file}"
fi

cmake -S "${source_root}/gazebo_grasp_plugin" -B "${build_root}" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${install_root}"
cmake --build "${build_root}" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cmake --install "${build_root}"

library="${install_root}/lib/libgazebo_grasp_fix.so"
test -f "${library}"
if ldd "${library}" | rg -q 'not found'; then
  echo "gazebo_grasp_fix has unresolved shared-library dependencies" >&2
  ldd "${library}" >&2
  exit 1
fi
printf 'gazebo_grasp_fix installed at %s\n' "${library}"
printf 'export GAZEBO_PLUGIN_PATH=%q\n' "${install_root}/lib${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"

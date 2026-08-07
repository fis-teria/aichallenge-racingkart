#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
repository_root="$(cd "${package_dir}/.." && pwd)"
build_dir="${TMPDIR:-/tmp}/state_lattice_overtake_planner_standalone_build"
compiler="${CXX:-g++}"
mkdir -p "${build_dir}"

"${compiler}" -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror \
  -I"${package_dir}/include" \
  -I"${repository_root}/simple_pure_pursuit/include" \
  "${script_dir}/standalone_regression.cpp" \
  "${package_dir}/src/config.cpp" \
  "${package_dir}/src/cost_model.cpp" \
  "${package_dir}/src/frenet_frame.cpp" \
  "${package_dir}/src/grid_map.cpp" \
  "${package_dir}/src/lattice_planner.cpp" \
  "${package_dir}/src/reference_override_contract.cpp" \
  -o "${build_dir}/standalone_regression"

"${build_dir}/standalone_regression" "${repository_root}"

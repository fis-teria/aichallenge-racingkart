#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
repository_root="$(cd "${package_dir}/.." && pwd)"
build_dir="${TMPDIR:-/tmp}/state_lattice_secondary_forward_shadow_build"
if [[ $# -gt 2 ]]; then
  echo "usage: $0 [result.json] [control|first-eligible]" >&2
  exit 2
fi
snapshot="${2:-control}"
if [[ "${snapshot}" != "control" && "${snapshot}" != "first-eligible" ]]; then
  echo "invalid snapshot: ${snapshot}" >&2
  exit 2
fi
if [[ $# -ge 1 ]]; then
  result_path="$1"
else
  run_dir="$(mktemp -d "${TMPDIR:-/tmp}/state-lattice-secondary-forward-XXXXXX")"
  result_path="${run_dir}/result.json"
fi
compiler="${CXX:-g++}"
mkdir -p "${build_dir}" "$(dirname "${result_path}")"

reference_path="${repository_root}/multi_purpose_mpc_ros/env/final_ver3/traj_mincurv_manual.csv"
map_yaml_path="${repository_root}/multi_purpose_mpc_ros/env/final_ver3/occupancy_grid_map.yaml"
map_pgm_path="${repository_root}/multi_purpose_mpc_ros/env/final_ver3/occupancy_grid_map.pgm"
expected_reference_sha256="4124a0b15cd8c9d9cec91e886a587d6fb6b57430f20f37455fd588bbca2d533e"
expected_map_yaml_sha256="39d5aba44234c1e09fc57421d467d64c1769259cd3c3656032f008d2db4e6a79"
expected_map_pgm_sha256="c24af2130a8df96047a49d5b7759af7a0b29645c023e7f3bc6d4ba436275725a"
reference_sha256="$(sha256sum "${reference_path}" | awk '{print $1}')"
map_yaml_sha256="$(sha256sum "${map_yaml_path}" | awk '{print $1}')"
map_pgm_sha256="$(sha256sum "${map_pgm_path}" | awk '{print $1}')"
if [[ "${reference_sha256}" != "${expected_reference_sha256}" ||
      "${map_yaml_sha256}" != "${expected_map_yaml_sha256}" ||
      "${map_pgm_sha256}" != "${expected_map_pgm_sha256}" ]]; then
  echo "secondary-forward fixture artifact fingerprint mismatch" >&2
  exit 3
fi

timeout 60s "${compiler}" -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror \
  -DSTATE_LATTICE_TEST_ACCESS \
  -I"${package_dir}/include" \
  "${script_dir}/secondary_forward_shadow_runner.cpp" \
  "${package_dir}/src/config.cpp" \
  "${package_dir}/src/cost_model.cpp" \
  "${package_dir}/src/frenet_frame.cpp" \
  "${package_dir}/src/grid_map.cpp" \
  "${package_dir}/src/lattice_planner.cpp" \
  -o "${build_dir}/secondary_forward_shadow_runner"

echo "secondary-forward result: ${result_path}" >&2
timeout 30s "${build_dir}/secondary_forward_shadow_runner" \
  "${repository_root}/multi_purpose_mpc_ros" "${result_path}" \
  "${reference_sha256}" "${map_yaml_sha256}" "${map_pgm_sha256}" \
  "${snapshot}"

#!/usr/bin/env bash

domain_id="${ROS_DOMAIN_ID:-1}"
ts="$(date +%Y%m%d-%H%M%S)"
out_dir="/output/${ts}/d${domain_id}"

mkdir -p "${out_dir}"
trap 'bash /aichallenge/utils/fix_ownership.bash "${HOST_UID}" "${HOST_GID}" /output "$(dirname "${out_dir}")"' EXIT

cd "${out_dir}" || exit
mkdir -p "${out_dir}/ros/log"

log_file="${out_dir}/autoware.log"
export ROS_HOME="${out_dir}/ros"
export ROS_LOG_DIR="${ROS_HOME}/log"
# Keep launch output in-file while still streaming to container stdout.
exec > >(tee -a "${log_file}") 2>&1

sim_mode="${SIM_MODE:-eval}"
launch_awsim="${LAUNCH_AWSIM:-true}"
run_rviz="${RUN_RVIZ:-true}"
awsim_vehicles="${AWSIM_VEHICLES:-1}"
awsim_laps="${AWSIM_LAPS:-6}"
awsim_timeout="${AWSIM_TIMEOUT:-600}"
awsim_extra_args="${AWSIM_EXTRA_ARGS:-}"

launch_args=(
    "domain_id:=${domain_id}"
    "sim_mode:=${sim_mode}"
    "log_dir:=${out_dir}"
    "capture:=true"
    "rosbag:=true"
    "simulation:=true"
    "use_sim_time:=true"
    "run_rviz:=${run_rviz}"
    "launch_awsim:=${launch_awsim}"
    "awsim_vehicles:=${awsim_vehicles}"
    "awsim_laps:=${awsim_laps}"
    "awsim_timeout:=${awsim_timeout}"
    "awsim_extra_args:=${awsim_extra_args}"
)

if [[ -n "${CONTROL_METHOD:-}" ]]; then
    launch_args+=("control_method:=${CONTROL_METHOD}")
fi

ros2 launch aichallenge_system_launch evaluation.launch.xml "${launch_args[@]}"

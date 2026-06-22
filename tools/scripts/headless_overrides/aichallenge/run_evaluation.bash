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

resolve_nvidia_vk_icd() {
    local configured="${VK_ICD_FILENAMES-}"
    if [[ -n ${configured} ]]; then
        echo "${configured}"
        return
    fi

    local candidate
    for candidate in \
        /etc/vulkan/icd.d/nvidia_icd.json \
        /usr/share/vulkan/icd.d/nvidia_icd.json
    do
        if [[ -f ${candidate} ]]; then
            echo "${candidate}"
            return
        fi
    done
}

sim_mode="${SIM_MODE:-eval}"
launch_awsim="${LAUNCH_AWSIM:-true}"
run_rviz="${RUN_RVIZ:-true}"
awsim_vehicles="${AWSIM_VEHICLES:-1}"
awsim_laps="${AWSIM_LAPS:-6}"
awsim_timeout="${AWSIM_TIMEOUT:-600}"
awsim_extra_args="${AWSIM_EXTRA_ARGS:-}"
awsim_prime_render_offload="${__NV_PRIME_RENDER_OFFLOAD:-1}"
awsim_vk_layer_optimus="${__VK_LAYER_NV_optimus:-NVIDIA_only}"
awsim_vk_icd_filenames="$(resolve_nvidia_vk_icd)"

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
    "awsim_prime_render_offload:=${awsim_prime_render_offload}"
    "awsim_vk_layer_optimus:=${awsim_vk_layer_optimus}"
)
if [[ -n ${awsim_extra_args} ]]; then
    launch_args+=("awsim_extra_args:=${awsim_extra_args}")
fi
if [[ -n ${awsim_vk_icd_filenames} ]]; then
    launch_args+=("awsim_vk_icd_filenames:=${awsim_vk_icd_filenames}")
fi

if [[ -n "${CONTROL_METHOD:-}" ]]; then
    launch_args+=("control_method:=${CONTROL_METHOD}")
fi

ros2 launch aichallenge_system_launch evaluation.launch.xml "${launch_args[@]}"

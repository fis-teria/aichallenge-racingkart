#!/bin/bash
AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
mode="${1:-${SIM_MODE:-eval}}"
[[ ${mode} == "eval" ]] && mode="1p"

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

case "${mode}" in
"dev")
    start_mode="off"
    vehicles=1
    laps=600
    timeout=60000000
    ;;
"test")
    start_mode="sync"
    vehicles=1
    laps=1
    timeout=90
    ;;
"1p" | "2p" | "3p" | "4p")
    start_mode="sync"
    vehicles="${mode%p}"
    laps=6
    timeout=600
    ;;
*)
    echo "invalid mode: ${mode}"
    echo "supported: dev, test, eval, 1p, 2p, 3p, 4p"
    exit 1
    ;;
esac

start_mode="${AWSIM_START_MODE:-${start_mode}}"
vehicles="${AWSIM_VEHICLES:-${vehicles}}"
laps="${AWSIM_LAPS:-${laps}}"
timeout="${AWSIM_TIMEOUT:-${timeout}}"
start_count_seconds="${AWSIM_START_COUNT_SECONDS:-}"

awsim_extra_args="${AWSIM_EXTRA_ARGS-}"
if [[ -z ${awsim_extra_args} && ! -e /dev/nvidia0 && ${mode} =~ ^(dev|test|[1-4]p)$ ]]; then
    awsim_extra_args="--camera false --lidar false"
fi
awsim_prime_render_offload="${__NV_PRIME_RENDER_OFFLOAD:-1}"
awsim_vk_layer_optimus="${__VK_LAYER_NV_optimus:-NVIDIA_only}"
awsim_vk_icd_filenames="$(resolve_nvidia_vk_icd)"

echo "[INFO] Starting AWSIM in '${mode}' mode"
echo "[INFO] AWSIM Vulkan env: __NV_PRIME_RENDER_OFFLOAD=${awsim_prime_render_offload} __VK_LAYER_NV_optimus=${awsim_vk_layer_optimus} VK_ICD_FILENAMES=${awsim_vk_icd_filenames:-<unset>}"

declare -a opts=("-force-vulkan" "--start-mode" "${start_mode}" "--vehicles" "${vehicles}" "--laps" "${laps}" "--timeout" "${timeout}")
if [[ -n ${start_count_seconds} ]]; then
    opts+=("--start-count-seconds" "${start_count_seconds}")
fi
declare -a extra_args
read -r -a extra_args <<<"${awsim_extra_args}"
opts+=("${extra_args[@]}")
printf "[INFO] AWSIM options:"
printf " %q" "${opts[@]}"
printf "\n"

export ROS_DOMAIN_ID=0
env_args=(
    "__NV_PRIME_RENDER_OFFLOAD=${awsim_prime_render_offload}"
    "__VK_LAYER_NV_optimus=${awsim_vk_layer_optimus}"
)
if [[ -n ${awsim_vk_icd_filenames} ]]; then
    env_args+=("VK_ICD_FILENAMES=${awsim_vk_icd_filenames}")
fi

env "${env_args[@]}" "$AWSIM_DIRECTORY/AWSIM.x86_64" "${opts[@]}"

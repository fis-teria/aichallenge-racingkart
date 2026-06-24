#!/bin/bash
AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
SCRIPT_DIR="$(dirname "$0")/simulator_scripts"
mode="${1:-${SIM_MODE:-eval}}"
[ $# -gt 0 ] && shift
[[ ${mode} == "eval" ]] && mode="1p"

# Preserve the existing GUI/dev override path for dev<N>, while still exposing
# the official 2026 script modes such as gate<N>, parallel, and multiplay-*.
if [[ ${mode} =~ ^dev([0-9]+)$ ]]; then
    mode="${BASH_REMATCH[1]}p"
fi

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

script_mode="${mode}"
if [[ ${mode} =~ ^gate([0-9]+)$ ]]; then
    script_mode="gate"
    set -- "${BASH_REMATCH[1]}" "$@"
fi

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
    script="${SCRIPT_DIR}/${script_mode}.sh"
    if [[ ! -f ${script} ]]; then
        echo "invalid mode: ${mode}"
        echo "supported: dev, test, eval, 1p, 2p, 3p, 4p, gate<N>, $(basename -s .sh "${SCRIPT_DIR}"/*.sh | xargs)"
        exit 1
    fi
    awsim_prime_render_offload="${__NV_PRIME_RENDER_OFFLOAD:-1}"
    awsim_vk_layer_optimus="${__VK_LAYER_NV_optimus:-NVIDIA_only}"
    awsim_vk_icd_filenames="$(resolve_nvidia_vk_icd)"
    echo "[INFO] Starting AWSIM script '${script_mode}.sh' for mode '${mode}'"
    echo "[INFO] AWSIM Vulkan env: __NV_PRIME_RENDER_OFFLOAD=${awsim_prime_render_offload} __VK_LAYER_NV_optimus=${awsim_vk_layer_optimus} VK_ICD_FILENAMES=${awsim_vk_icd_filenames:-<unset>}"
    env_args=(
        "__NV_PRIME_RENDER_OFFLOAD=${awsim_prime_render_offload}"
        "__VK_LAYER_NV_optimus=${awsim_vk_layer_optimus}"
    )
    if [[ -n ${awsim_vk_icd_filenames} ]]; then
        env_args+=("VK_ICD_FILENAMES=${awsim_vk_icd_filenames}")
    fi
    export ROS_DOMAIN_ID=0
    exec env "${env_args[@]}" bash "${script}" "$@"
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

export ROS_DOMAIN_ID=0
env_args=(
    "__NV_PRIME_RENDER_OFFLOAD=${awsim_prime_render_offload}"
    "__VK_LAYER_NV_optimus=${awsim_vk_layer_optimus}"
)
if [[ -n ${awsim_vk_icd_filenames} ]]; then
    env_args+=("VK_ICD_FILENAMES=${awsim_vk_icd_filenames}")
fi

env "${env_args[@]}" "$AWSIM_DIRECTORY/AWSIM.x86_64" "${opts[@]}"

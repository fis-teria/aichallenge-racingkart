#!/bin/bash
AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
mode="${1:-${SIM_MODE:-eval}}"
[[ ${mode} == "eval" ]] && mode="1p"

case "${mode}" in
"dev")
    start_mode="off"
    vehicles=1
    laps=unlimited
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

awsim_extra_args="${AWSIM_EXTRA_ARGS-}"
headless="${AWSIM_HEADLESS:-${GA_EXPERIMENT_MODE:-false}}"
if [[ "$headless" == "true" || "$headless" == "1" ]]; then
    awsim_extra_args="-batchmode -nographics --camera false --lidar false ${awsim_extra_args}"
elif [[ -z ${awsim_extra_args} && ! -e /dev/nvidia0 && ${mode} =~ ^(dev|test|[1-4]p)$ ]]; then
    awsim_extra_args="--camera false --lidar false"
fi

echo "[INFO] Starting AWSIM in '${mode}' mode (headless=${headless})"

declare -a opts=("--start-mode" "${start_mode}" "--vehicles" "${vehicles}" "--laps" "${laps}" "--timeout" "${timeout}")
declare -a extra_args
read -r -a extra_args <<<"${awsim_extra_args}"
opts+=("${extra_args[@]}")

export ROS_DOMAIN_ID=0
$AWSIM_DIRECTORY/AWSIM.x86_64 "${opts[@]}"

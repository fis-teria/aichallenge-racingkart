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
"ghost4")
    # Four independent ROS domains in one AWSIM process. The race config
    # disables vehicle-to-vehicle contacts and the scenario deliberately
    # overlaps every vehicle at the D1 start pose.
    race_config="ga-ghost-4"
    scenario="${AWSIM_DIRECTORY}/AWSIM_Data/StreamingAssets/Scenarios/ga-ghost-d1-4.yaml"
    ;;
"ghost4-separated")
    # Diagnostic variant: keep vehicle 1 at the legacy GA start while placing
    # the other three at the corresponding D2-D4 grid offsets.
    race_config="ga-ghost-4"
    scenario="${AWSIM_DIRECTORY}/AWSIM_Data/StreamingAssets/Scenarios/ga-ghost-separated-4.yaml"
    ;;
"ghost1-separated" | "ghost2-separated" | "ghost3-separated")
    vehicle_count="${mode#ghost}"
    vehicle_count="${vehicle_count%%-*}"
    race_config="ga-ghost-${vehicle_count}"
    scenario="${AWSIM_DIRECTORY}/AWSIM_Data/StreamingAssets/Scenarios/ga-ghost-separated-${vehicle_count}.yaml"
    ;;
"ghost1-nohandicap")
    race_config="ga-ghost-1-nohandicap"
    scenario="${AWSIM_DIRECTORY}/AWSIM_Data/StreamingAssets/Scenarios/ga-ghost-separated-1.yaml"
    ;;
*)
    echo "invalid mode: ${mode}"
    echo "supported: dev, test, eval, 1p, 2p, 3p, 4p, ghost4, ghost1-separated..ghost4-separated, ghost1-nohandicap"
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

if [[ ${mode} == "ghost4" || ${mode} =~ ^ghost[1-4]-separated$ || ${mode} == "ghost1-nohandicap" ]]; then
    declare -a opts=("--race-config" "${race_config}" "--scenario" "${scenario}")
else
    declare -a opts=("--start-mode" "${start_mode}" "--vehicles" "${vehicles}" "--laps" "${laps}" "--timeout" "${timeout}")
fi
declare -a extra_args
read -r -a extra_args <<<"${awsim_extra_args}"
opts+=("${extra_args[@]}")

export ROS_DOMAIN_ID=0
$AWSIM_DIRECTORY/AWSIM.x86_64 "${opts[@]}"

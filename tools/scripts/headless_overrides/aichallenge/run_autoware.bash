#!/bin/bash

mode="${1}"
id="${2:-${ROS_DOMAIN_ID:-0}}"
out_dir="${3:+${3}/d${id}}"
out_dir="${out_dir:-/output/$(date +%Y%m%d-%H%M%S)/d${id}}"

case "${mode}" in
"awsim")
    opts=("simulation:=true" "use_sim_time:=true" "run_rviz:=true")
    ;;
"awsim-no-viz")
    opts=("simulation:=true" "use_sim_time:=true" "run_rviz:=false")
    ;;
"vehicle")
    opts=("simulation:=false" "use_sim_time:=false" "run_rviz:=false")
    ;;
"rosbag")
    opts=("simulation:=false" "use_sim_time:=true" "run_rviz:=true")
    ;;
*)
    echo "invalid argument (use 'awsim' or 'vehicle' or 'rosbag')"
    exit 1
    ;;
esac

export ROS_DOMAIN_ID=$id
if [[ -n "${CONTROL_METHOD:-}" ]]; then
    opts+=("control_method:=${CONTROL_METHOD}")
fi
if [[ -n "${RACE_ARM_ON_VEHICLE_STATE:-}" ]]; then
    opts+=("race_arm_on_vehicle_state:=${RACE_ARM_ON_VEHICLE_STATE}")
fi
opts+=("autostart_debug_visualization:=${AUTOSTART_DEBUG_VISUALIZATION:-true}")
capture="${CAPTURE:-false}"
rosbag="${ROSBAG:-false}"
opts+=("capture:=${capture}" "rosbag:=${rosbag}")

mkdir -p "${out_dir}"
exec >"${out_dir}/autoware.log" 2>&1
trap 'bash /aichallenge/utils/fix_ownership.bash "${HOST_UID}" "${HOST_GID}" /output "$(dirname "${out_dir}")"' EXIT

cd "${out_dir}" || exit
# Persist ROS node logs under the run output directory (so autostart_orchestrator logs are collectible).
export ROS_HOME="${out_dir}/ros"
export ROS_LOG_DIR="${ROS_HOME}/log"
mkdir -p "${ROS_LOG_DIR}"

launch_pid=""
shutdown_requested=false

forward_shutdown_to_launch() {
    shutdown_requested=true
    if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
        # ros2 launch performs the coordinated node shutdown on SIGINT.  In
        # particular this lets autostart_orchestrator finalize rosbag metadata
        # before the container exits.
        echo "[run_autoware] forwarding SIGINT to ros2 launch pid=${launch_pid}"
        kill -INT "${launch_pid}" 2>/dev/null || true
    fi
}

trap forward_shutdown_to_launch INT TERM

launch_sigterm_timeout_sec="${AUTOWARE_LAUNCH_SIGTERM_TIMEOUT_SEC:-120}"
launch_sigkill_timeout_sec="${AUTOWARE_LAUNCH_SIGKILL_TIMEOUT_SEC:-30}"

(
    # Non-interactive bash normally starts asynchronous commands with SIGINT
    # ignored.  Reset it in the child so the forwarded SIGINT reaches the ROS
    # launch shutdown handler.
    trap - INT TERM
    exec ros2 launch --noninteractive \
        aichallenge_system_launch \
        aichallenge_system.launch.xml \
        "${opts[@]}" \
        "domain_id:=$id" \
        "sigterm_timeout:=${launch_sigterm_timeout_sec}" \
        "sigkill_timeout:=${launch_sigkill_timeout_sec}"
) &
launch_pid=$!

# Preserve a shutdown request received in the narrow interval before the child
# PID was assigned.
if [[ "${shutdown_requested}" == true ]]; then
    forward_shutdown_to_launch
fi

# A signal interrupts bash's wait before ros2 launch necessarily exits.  Keep
# waiting after forwarding it so Docker's grace period applies to the actual
# ROS shutdown instead of terminating this wrapper early.
while true; do
    wait "${launch_pid}"
    launch_status=$?
    if ! kill -0 "${launch_pid}" 2>/dev/null; then
        break
    fi
done

launch_pid=""
exit "${launch_status}"

#!/bin/bash

# Bounded no-authority State Lattice observation. It starts only its own
# observer and rosbag processes; it never starts/stops AWSIM or Autoware.
set -euo pipefail

readonly OBSERVER_NODE="/test_only_state_lattice_observer"
readonly TEST_ONLY_RUN_ID="${TEST_ONLY_RUN_ID:?TEST_ONLY_RUN_ID is required}"
readonly TEST_ONLY_TOPIC_TOKEN="run_${TEST_ONLY_RUN_ID//[^A-Za-z0-9_]/_}"
readonly TEST_PREFIX="/test_only/${TEST_ONLY_TOPIC_TOKEN}"
readonly ARTIFACT_DIR="${TEST_ONLY_ARTIFACT_DIR:?TEST_ONLY_ARTIFACT_DIR is required}"
readonly DURATION_SEC="${TEST_ONLY_DURATION_SEC:-12}"
readonly INPUT_ODOM="${TEST_ONLY_INPUT_ODOM:-/localization/kinematic_state}"
readonly LAUNCH_FILE="/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/test_only/state_lattice_observer.launch.xml"
readonly GRAPH_QUERY_TIMEOUT_SEC=5

readonly -a FORBIDDEN_TOPICS=(
    "/control/command/control_cmd"
    "/control/command/control_cmd_raw"
    "/overtake/reference_override"
    "/hybrid_control/state_lattice/control_cmd"
    "/hybrid_control/motion_authority_grant"
    "/overtake/race_armed"
    "/admin/awsim/start"
    "/admin/awsim/reset"
)
readonly -a AUTHORITY_TEST_OUTPUT_TOPICS=(
    "${TEST_PREFIX}/state_lattice/instant_control_cmd"
    "${TEST_PREFIX}/state_lattice/reference_override"
)
readonly -a TEST_OUTPUT_TOPICS=(
    "${TEST_PREFIX}/state_lattice/instant_control_cmd"
    "${TEST_PREFIX}/state_lattice/reference_override"
    "${TEST_PREFIX}/debug/overtake/mode"
    "${TEST_PREFIX}/debug/overtake/metrics"
    "${TEST_PREFIX}/debug/overtake/wall_map"
    "${TEST_PREFIX}/debug/overtake/wall_costmap"
    "${TEST_PREFIX}/debug/overtake/opponent_costmap"
    "${TEST_PREFIX}/debug/overtake/costmap"
    "${TEST_PREFIX}/debug/overtake/trajectory_candidates"
    "${TEST_PREFIX}/debug/overtake/selected_trajectory"
    "${TEST_PREFIX}/debug/overtake/target_states"
    "${TEST_PREFIX}/debug/overtake/rear_safety_paths"
    "${TEST_PREFIX}/debug/overtake/planning_geometry"
)

observer_pid=""
bag_pid=""

if [[ ! "${TEST_ONLY_RUN_ID}" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "TEST_ONLY_RUN_ID must contain only [A-Za-z0-9._-]" >&2
    exit 2
fi
if [[ ! "${ARTIFACT_DIR}" =~ ^/output/ ]]; then
    echo "TEST_ONLY_ARTIFACT_DIR must be below /output" >&2
    exit 2
fi
if [[ ! "${DURATION_SEC}" =~ ^[1-9][0-9]*$ ]] || (( DURATION_SEC > 60 )); then
    echo "TEST_ONLY_DURATION_SEC must be an integer in [1, 60]" >&2
    exit 2
fi
if [[ ! -f "${LAUNCH_FILE}" ]]; then
    echo "test-only launch file is missing: ${LAUNCH_FILE}" >&2
    exit 2
fi

cleanup_process() {
    local pid="$1"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
        kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
        wait "${pid}" 2>/dev/null || true
    fi
}

cleanup() {
    cleanup_process "${bag_pid}"
    cleanup_process "${observer_pid}"
}
trap cleanup EXIT INT TERM

# ROS 2 Humble's generated setup references optional trace variables before it
# assigns them, so source it under nounset-off and immediately restore it.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
set -u
export ROS2CLI_NO_DAEMON=1
mkdir -p "${ARTIFACT_DIR}"

snapshot_graph() {
    local snapshot="$1"
    {
        echo "# nodes"
        timeout "${GRAPH_QUERY_TIMEOUT_SEC}s" ros2 node list 2>&1 | LC_ALL=C sort || true
        echo "# topics"
        timeout "${GRAPH_QUERY_TIMEOUT_SEC}s" ros2 topic list -t 2>&1 | LC_ALL=C sort || true
    } >"${snapshot}"
}

write_manifest() {
    local verdict="$1"
    {
        echo "test_only_run_id=${TEST_ONLY_RUN_ID}"
        echo "test_only_topic_token=${TEST_ONLY_TOPIC_TOKEN}"
        echo "ros_domain_id=${ROS_DOMAIN_ID:-}"
        echo "duration_sec=${DURATION_SEC}"
        echo "input_odom=${INPUT_ODOM}"
        echo "live_control_output_enabled=false"
        echo "instant_control_enabled=false"
        echo "safety_evaluation_enabled=true"
        echo "instant_control_topic=${TEST_PREFIX}/state_lattice/instant_control_cmd"
        echo "reference_override_topic=${TEST_PREFIX}/state_lattice/reference_override"
        echo "verdict=${verdict}"
    } >"${ARTIFACT_DIR}/manifest.env"
}

fail_closed() {
    local reason="$1"
    echo "TEST_ONLY_OBSERVER_FAIL_CLOSED: ${reason}" >&2
    write_manifest "FAIL:${reason}"
    exit 1
}

if timeout "${GRAPH_QUERY_TIMEOUT_SEC}s" ros2 node list | grep -Fxq "${OBSERVER_NODE}"; then
    fail_closed "observer_node_already_exists"
fi

snapshot_graph "${ARTIFACT_DIR}/ros_graph_before.txt"
write_manifest "STARTED"
setsid ros2 launch --noninteractive "${LAUNCH_FILE}" \
    "run_id:=${TEST_ONLY_RUN_ID}" "topic_token:=${TEST_ONLY_TOPIC_TOKEN}" \
    "input_odom:=${INPUT_ODOM}" \
    >"${ARTIFACT_DIR}/state_lattice_observer.log" 2>&1 &
observer_pid=$!

deadline=$((SECONDS + 15))
until timeout "${GRAPH_QUERY_TIMEOUT_SEC}s" ros2 node list | grep -Fxq "${OBSERVER_NODE}"; do
    if ! kill -0 "${observer_pid}" 2>/dev/null; then
        fail_closed "observer_exited_before_graph_ready"
    fi
    if (( SECONDS >= deadline )); then
        fail_closed "observer_graph_timeout"
    fi
    sleep 0.2
done

if ! timeout 8s ros2 node info "${OBSERVER_NODE}" >"${ARTIFACT_DIR}/observer_node_info.txt" 2>&1; then
    fail_closed "observer_node_info_unavailable"
fi
for topic in "${FORBIDDEN_TOPICS[@]}"; do
    if grep -Fq "${topic}" "${ARTIFACT_DIR}/observer_node_info.txt"; then
        fail_closed "observer_connected_to_forbidden_topic:${topic}"
    fi
done
for topic in "${AUTHORITY_TEST_OUTPUT_TOPICS[@]}"; do
    if ! timeout 8s ros2 topic info -v "${topic}" >"${ARTIFACT_DIR}/test_output_$(basename "${topic}").txt" 2>&1; then
        fail_closed "test_only_output_graph_unavailable:${topic}"
    fi
    if grep -Eq 'Subscription count: [1-9]' "${ARTIFACT_DIR}/test_output_$(basename "${topic}").txt"; then
        fail_closed "test_only_output_has_subscriber_before_recording:${topic}"
    fi
done
snapshot_graph "${ARTIFACT_DIR}/ros_graph_active.txt"

setsid ros2 bag record -s mcap --compression-format zstd --compression-mode file \
    -o "${ARTIFACT_DIR}/rosbag2_test_only" \
    /clock "${INPUT_ODOM}" /v2x/vehicle_positions /mpc/speed_profile_debug \
    "${TEST_OUTPUT_TOPICS[@]}" \
    >"${ARTIFACT_DIR}/rosbag_record.log" 2>&1 &
bag_pid=$!
sleep "${DURATION_SEC}"
cleanup_process "${bag_pid}"
bag_pid=""

snapshot_graph "${ARTIFACT_DIR}/ros_graph_before_observer_shutdown.txt"
cleanup_process "${observer_pid}"
observer_pid=""

deadline=$((SECONDS + 10))
while timeout "${GRAPH_QUERY_TIMEOUT_SEC}s" ros2 node list | grep -Fxq "${OBSERVER_NODE}"; do
    if (( SECONDS >= deadline )); then
        fail_closed "observer_node_remained_after_shutdown"
    fi
    sleep 0.2
done

snapshot_graph "${ARTIFACT_DIR}/ros_graph_after.txt"
diff -u "${ARTIFACT_DIR}/ros_graph_before.txt" \
    "${ARTIFACT_DIR}/ros_graph_after.txt" \
    >"${ARTIFACT_DIR}/ros_graph_before_after.diff" || true

if [[ ! -f "${ARTIFACT_DIR}/rosbag2_test_only/metadata.yaml" ]] || \
    ! find "${ARTIFACT_DIR}/rosbag2_test_only" -type f -name '*.mcap*' -size +0c | grep -q .; then
    fail_closed "rosbag_missing_or_empty"
fi
ros2 bag info "${ARTIFACT_DIR}/rosbag2_test_only" >"${ARTIFACT_DIR}/rosbag_info.txt" 2>&1 || fail_closed "rosbag_info_failed"
write_manifest "PASS"
echo "TEST_ONLY_OBSERVER_PASS artifact=${ARTIFACT_DIR}"

#!/bin/bash
# Bounded, localhost-only, no-AWSIM replay into a private production-algorithm graph.
set -euo pipefail

readonly RUN_ID="${TEST_ONLY_REPLAY_RUN_ID:?TEST_ONLY_REPLAY_RUN_ID is required}"
readonly MODE="${TEST_ONLY_REPLAY_MODE:?TEST_ONLY_REPLAY_MODE is required}"
readonly SOURCE_BAG="${TEST_ONLY_REPLAY_SOURCE_BAG:?TEST_ONLY_REPLAY_SOURCE_BAG is required}"
readonly ARTIFACT_DIR="${TEST_ONLY_REPLAY_ARTIFACT_DIR:?TEST_ONLY_REPLAY_ARTIFACT_DIR is required}"
readonly TIMEOUT_SEC="${TEST_ONLY_REPLAY_TIMEOUT_SEC:-45}"
readonly REPLAY_DOMAIN_ID="${TEST_ONLY_REPLAY_ROS_DOMAIN_ID:?TEST_ONLY_REPLAY_ROS_DOMAIN_ID is required}"
readonly TOPIC_TOKEN="run_${RUN_ID//[^A-Za-z0-9_]/_}"
readonly PRIVATE_ROOT="/test_only/offline_replay/${TOPIC_TOKEN}"
readonly LAUNCH_FILE="/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/test_only/state_lattice_pp_mux_offline_replay.launch.xml"
readonly CONTRACT="/aichallenge/state_lattice_pp_mux_offline_contract.py"
readonly SOURCE_METADATA="${SOURCE_BAG}/metadata.yaml"
readonly OUTPUT_BAG="${ARTIFACT_DIR}/rosbag2_private_output"
readonly OUTPUT_METADATA="${OUTPUT_BAG}/metadata.yaml"
readonly GRAPH_TIMEOUT_SEC=5

readonly -a SOURCE_PLAY_TOPICS=(
    /clock
    /tf
    /tf_static
    /localization/kinematic_state
    /v2x/vehicle_positions
    /planning/scenario_planning/trajectory
    /overtake/race_armed
    /awsim/state
    /vehicle/status/steering_status
    /mpc/speed_profile_debug
)
readonly -a SOURCE_REMAPS=(
    "/clock:=${PRIVATE_ROOT}/input/clock"
    "/tf:=${PRIVATE_ROOT}/input/tf"
    "/tf_static:=${PRIVATE_ROOT}/input/tf_static"
    "/localization/kinematic_state:=${PRIVATE_ROOT}/input/kinematics"
    "/v2x/vehicle_positions:=${PRIVATE_ROOT}/input/v2x"
    "/planning/scenario_planning/trajectory:=${PRIVATE_ROOT}/input/trajectory"
    "/overtake/race_armed:=${PRIVATE_ROOT}/input/race_armed"
    "/awsim/state:=${PRIVATE_ROOT}/input/awsim_state"
    "/vehicle/status/steering_status:=${PRIVATE_ROOT}/input/steering_status"
    "/mpc/speed_profile_debug:=${PRIVATE_ROOT}/input/mpc_health"
)
readonly -a RECORDED_TOPICS=(
    "${PRIVATE_ROOT}/planner/reference_override"
    "${PRIVATE_ROOT}/planner/plan"
    "${PRIVATE_ROOT}/planner/safety_constraint"
    "${PRIVATE_ROOT}/pp/control_cmd"
    "${PRIVATE_ROOT}/pp/tracking_status"
    "${PRIVATE_ROOT}/pp/command_envelope"
    "${PRIVATE_ROOT}/pp/execution_envelope"
    "${PRIVATE_ROOT}/pp/free_run_execution_ack"
    "${PRIVATE_ROOT}/mux/debug"
    "${PRIVATE_ROOT}/mux/tracking_status"
    "${PRIVATE_ROOT}/mux/motion_authority_grant"
    "${PRIVATE_ROOT}/output/control_cmd"
)
readonly -a FORBIDDEN_LIVE_TOPICS=(
    /control/command/control_cmd
    /control/command/control_cmd_raw
    /overtake/reference_override
    /overtake/plan
    /overtake/safety_constraint
    /overtake/race_armed
    /hybrid_control/pure_pursuit/free_run_execution_ack
    /hybrid_control/motion_authority_grant
    /admin/awsim/start
    /admin/awsim/reset
)
readonly -a FORBIDDEN_LIVE_PREFIXES=(
    /admin/
    /control/
    /overtake/
    /hybrid_control/
)
readonly -a HASH_INPUTS=(
    "combined_launch=${LAUNCH_FILE}"
    "state_lattice_config=/aichallenge/workspace/src/aichallenge_submit/state_lattice_overtake_planner/config/state_lattice_overtake_planner.param.yaml"
    "overtake_config=/aichallenge/workspace/src/aichallenge_submit/overtake_planner/config/overtake_planner.param.yaml"
    "pure_pursuit_launch=/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/control/pure_pursuit.launch.xml"
    "mux_config=/aichallenge/workspace/src/aichallenge_submit/hybrid_control_mux/config/hybrid_control_mux.param.yaml"
)

launch_pid=0
record_pid=0
play_pid=0
cleanup_complete=false

fail_closed() {
    echo "TEST_ONLY_OFFLINE_REPLAY_HOLD: $1" >&2
    exit 1
}

group_exists() {
    local pid="$1"
    kill -0 -- "-${pid}" 2>/dev/null
}

stop_group() {
    local pid="$1"
    local deadline
    if (( pid <= 0 )) || ! group_exists "${pid}"; then
        if (( pid > 0 )); then wait "${pid}" 2>/dev/null || true; fi
        return 0
    fi
    kill -INT -- "-${pid}" 2>/dev/null || true
    deadline=$((SECONDS + 3))
    while (( SECONDS < deadline )) && group_exists "${pid}"; do sleep 0.05; done
    if group_exists "${pid}"; then
        kill -TERM -- "-${pid}" 2>/dev/null || true
        sleep 0.2
    fi
    if group_exists "${pid}"; then
        kill -KILL -- "-${pid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null || true
}

assert_group_gone() {
    local pid="$1"
    local label="$2"
    if (( pid > 0 )) && group_exists "${pid}"; then
        fail_closed "process_group_residue:${label}:${pid}"
    fi
}

cleanup() {
    if [[ "${cleanup_complete}" == true ]]; then return 0; fi
    stop_group "${play_pid}"
    stop_group "${record_pid}"
    stop_group "${launch_pid}"
    cleanup_complete=true
}
trap cleanup EXIT INT TERM

if [[ ! "${RUN_ID}" =~ ^[A-Za-z0-9._-]+$ ]]; then fail_closed "invalid_run_id"; fi
if [[ "${MODE}" != baseline && "${MODE}" != regenerated ]]; then fail_closed "invalid_mode"; fi
if [[ ! "${TIMEOUT_SEC}" =~ ^[1-9][0-9]*$ ]] || (( TIMEOUT_SEC > 60 )); then
    fail_closed "invalid_timeout"
fi
if [[ ! "${REPLAY_DOMAIN_ID}" =~ ^[1-9][0-9]*$ ]] || (( REPLAY_DOMAIN_ID > 232 )); then
    fail_closed "invalid_ros_domain"
fi
IFS=',' read -r -a live_domains <<<"${AWSIM_READY_DOMAINS:-1}"
for live_domain in "${live_domains[@]}"; do
    if [[ "${REPLAY_DOMAIN_ID}" == "${live_domain}" ]]; then
        fail_closed "replay_domain_overlaps_live_domain:${live_domain}"
    fi
done
if [[ ! "${SOURCE_BAG}" =~ ^/output/ ]] || [[ ! "${ARTIFACT_DIR}" =~ ^/output/ ]]; then
    fail_closed "paths_must_be_below_output"
fi
if [[ ! -f "${SOURCE_METADATA}" ]] || [[ -e "${ARTIFACT_DIR}" ]]; then
    fail_closed "missing_source_or_stale_artifact_directory"
fi
mapfile -t source_mcaps < <(find "${SOURCE_BAG}" -maxdepth 1 -type f -name '*.mcap' -print)
if (( ${#source_mcaps[@]} != 1 )); then fail_closed "source_mcap_count_not_one"; fi
readonly SOURCE_MCAP="${source_mcaps[0]}"
for required in "${LAUNCH_FILE}" "${CONTRACT}"; do
    [[ -f "${required}" ]] || fail_closed "missing_required_file:${required}"
done

# Isolation is supplied by the runtime CycloneDDS profile. Validate it before
# disabling ROS_LOCALHOST_ONLY's duplicate automatic `lo` selection.
[[ "${CYCLONEDDS_URI:-}" == file://* ]] ||
    fail_closed "cyclonedds_loopback_profile_required"
readonly CYCLONEDDS_CONFIG="${CYCLONEDDS_URI#file://}"
[[ -f "${CYCLONEDDS_CONFIG}" ]] || fail_closed "cyclonedds_profile_missing"
[[ "$(grep -c '<NetworkInterface ' "${CYCLONEDDS_CONFIG}")" == 1 ]] ||
    fail_closed "cyclonedds_interface_count_not_one"
grep -Eq '<NetworkInterface [^>]*autodetermine="false"[^>]*name="lo"|<NetworkInterface [^>]*name="lo"[^>]*autodetermine="false"' \
    "${CYCLONEDDS_CONFIG}" || fail_closed "cyclonedds_not_loopback_only"

set +u
source /opt/ros/humble/setup.bash
source /aichallenge/workspace/install/setup.bash
set -u
export ROS_DOMAIN_ID="${REPLAY_DOMAIN_ID}"
# The runtime CycloneDDS profile already selects only the loopback interface.
# ROS_LOCALHOST_ONLY=1 would select `lo` a second time and CycloneDDS rejects
# the domain with "the same interface may not be selected twice".
export ROS_LOCALHOST_ONLY=0
export ROS2CLI_NO_DAEMON=1
mkdir -p "${ARTIFACT_DIR}"

config_args=()
for item in "${HASH_INPUTS[@]}"; do config_args+=(--config "${item}"); done
python3 "${CONTRACT}" --phase preflight --mode "${MODE}"     --private-root "${PRIVATE_ROOT}" --source-metadata "${SOURCE_METADATA}"     --source-mcap "${SOURCE_MCAP}" --result "${ARTIFACT_DIR}/preflight.json"     "${config_args[@]}" || fail_closed "source_preflight_failed"

snapshot_graph() {
    local path="$1"
    {
        echo "# nodes"
        timeout "${GRAPH_TIMEOUT_SEC}s" ros2 node list 2>&1 | LC_ALL=C sort
        echo "# topics"
        timeout "${GRAPH_TIMEOUT_SEC}s" ros2 topic list -t 2>&1 | LC_ALL=C sort
        echo "# services"
        timeout "${GRAPH_TIMEOUT_SEC}s" ros2 service list -t 2>&1 | LC_ALL=C sort
        echo "# actions"
        timeout "${GRAPH_TIMEOUT_SEC}s" ros2 action list -t 2>&1 | LC_ALL=C sort
    } >"${path}"
}

assert_exact_nodes() {
    local label="$1"
    shift
    local actual="${ARTIFACT_DIR}/nodes_${label}_actual.txt"
    local expected="${ARTIFACT_DIR}/nodes_${label}_expected.txt"
    timeout "${GRAPH_TIMEOUT_SEC}s" ros2 node list | LC_ALL=C sort -u >"${actual}"
    printf '%s\n' "$@" | LC_ALL=C sort -u >"${expected}"
    diff -u "${expected}" "${actual}" >"${ARTIFACT_DIR}/nodes_${label}.diff" ||
        fail_closed "unexpected_node_set:${label}"
}

assert_all_topics_private() {
    local topic allowed_topic allowed=false
    local allowed_topics=("$@")
    while IFS= read -r topic; do
        [[ -z "${topic}" || "${topic}" == /rosout || "${topic}" == /parameter_events ]] && continue
        allowed=false
        for allowed_topic in "${allowed_topics[@]}"; do
            [[ "${topic}" == "${allowed_topic}" ]] && allowed=true && break
        done
        [[ "${allowed}" == true ]] && continue
        [[ "${topic}" == "${PRIVATE_ROOT}"/* ]] || fail_closed "non_private_topic:${topic}"
    done < <(timeout "${GRAPH_TIMEOUT_SEC}s" ros2 topic list)
}

assert_no_live_interfaces() {
    local kind name prefix
    for kind in topic service action; do
        while IFS= read -r name; do
            [[ -z "${name}" ]] && continue
            for prefix in "${FORBIDDEN_LIVE_PREFIXES[@]}"; do
                [[ "${name}" == "${prefix}"* ]] && fail_closed "live_${kind}_interface:${name}"
            done
        done < <(timeout "${GRAPH_TIMEOUT_SEC}s" ros2 "${kind}" list)
    done
}

assert_topic_contract() {
    local label="$1"
    local topic="$2"
    local expected_publishers="$3"
    local expected_publisher_name="$4"
    shift 4
    local expected_subscribers=("$@")
    local info="${ARTIFACT_DIR}/topic_contract_${label}.txt"
    local publisher_section subscriber_section subscriber
    timeout "${GRAPH_TIMEOUT_SEC}s" ros2 topic info -v "${topic}" >"${info}" 2>&1 ||
        fail_closed "topic_info_failed:${label}"
    grep -Fxq "Publisher count: ${expected_publishers}" "${info}" ||
        fail_closed "publisher_count_mismatch:${label}"
    grep -Fxq "Subscription count: ${#expected_subscribers[@]}" "${info}" ||
        fail_closed "subscriber_count_mismatch:${label}"
    publisher_section="$(sed -n '/^Publisher count:/,/^Subscription count:/p' "${info}")"
    subscriber_section="$(sed -n '/^Subscription count:/,$p' "${info}")"
    if (( expected_publishers == 1 )); then
        grep -Fq "Node name: ${expected_publisher_name}" <<<"${publisher_section}" ||
            fail_closed "publisher_identity_mismatch:${label}"
    elif grep -Fq 'Node name:' <<<"${publisher_section}"; then
        fail_closed "unexpected_publisher_identity:${label}"
    fi
    for subscriber in "${expected_subscribers[@]}"; do
        grep -Fq "Node name: ${subscriber}" <<<"${subscriber_section}" ||
            fail_closed "subscriber_identity_mismatch:${label}:${subscriber}"
    done
}

snapshot_graph "${ARTIFACT_DIR}/ros_graph_before.txt"
if timeout "${GRAPH_TIMEOUT_SEC}s" ros2 node list | grep -q .; then
    fail_closed "dedicated_domain_not_unused"
fi

regenerated_flag=false
[[ "${MODE}" == regenerated ]] && regenerated_flag=true
setsid ros2 launch --noninteractive "${LAUNCH_FILE}"     "topic_token:=${TOPIC_TOKEN}" "regenerated_mode:=${regenerated_flag}"     "use_sim_time:=true" >"${ARTIFACT_DIR}/combined_launch.log" 2>&1 &
launch_pid=$!

expected_nodes=(
    /test_only_offline_state_lattice_planner
    /test_only_offline_pure_pursuit
    /test_only_offline_hybrid_control_mux
)
[[ "${MODE}" == regenerated ]] && expected_nodes+=(/test_only_offline_overtake_planner)
ready_deadline=$((SECONDS + 15))
for expected in "${expected_nodes[@]}"; do
    until timeout "${GRAPH_TIMEOUT_SEC}s" ros2 node list | grep -Fxq "${expected}"; do
        kill -0 "${launch_pid}" 2>/dev/null || fail_closed "launch_exited_before_ready"
        (( SECONDS < ready_deadline )) || fail_closed "graph_ready_timeout:${expected}"
        sleep 0.1
    done
done
# tf2 creates one private implementation node with a per-process hexadecimal
# suffix. Admit exactly that known shape, then retain the exact-set audit for
# every other node and for all later graph phases.
mapfile -t transform_listener_nodes < <(
    timeout "${GRAPH_TIMEOUT_SEC}s" ros2 node list |
        grep -E '^/transform_listener_impl_[0-9a-f]+$' || true
)
(( ${#transform_listener_nodes[@]} == 1 )) ||
    fail_closed "unexpected_transform_listener_count:${#transform_listener_nodes[@]}"
expected_nodes+=("${transform_listener_nodes[0]}")
assert_exact_nodes launch "${expected_nodes[@]}"
assert_all_topics_private
assert_no_live_interfaces
snapshot_graph "${ARTIFACT_DIR}/ros_graph_active.txt"
for topic in "${FORBIDDEN_LIVE_TOPICS[@]}"; do
    if timeout "${GRAPH_TIMEOUT_SEC}s" ros2 topic list | grep -Fxq "${topic}"; then
        fail_closed "live_endpoint_present:${topic}"
    fi
done

setsid ros2 bag record -s mcap -o "${OUTPUT_BAG}" "${RECORDED_TOPICS[@]}" \
    >"${ARTIFACT_DIR}/output_record.log" 2>&1 &
record_pid=$!
recorder_deadline=$((SECONDS + 10))
final_topic="${PRIVATE_ROOT}/output/control_cmd"
recorder_node=/rosbag2_recorder
until timeout "${GRAPH_TIMEOUT_SEC}s" ros2 node list | grep -Fxq "${recorder_node}"; do
    kill -0 "${record_pid}" 2>/dev/null || fail_closed "recorder_exited_before_ready"
    (( SECONDS < recorder_deadline )) || fail_closed "final_output_graph_timeout"
    sleep 0.1
done
expected_recording_nodes=("${expected_nodes[@]}" "${recorder_node}")
assert_exact_nodes recording "${expected_recording_nodes[@]}"
assert_all_topics_private /events/write_split
assert_no_live_interfaces
snapshot_graph "${ARTIFACT_DIR}/ros_graph_recording.txt"

recorder_name=rosbag2_recorder
assert_topic_contract reference_override "${PRIVATE_ROOT}/planner/reference_override" 1 \
    test_only_offline_state_lattice_planner test_only_offline_pure_pursuit "${recorder_name}"
if [[ "${MODE}" == regenerated ]]; then
    assert_topic_contract plan "${PRIVATE_ROOT}/planner/plan" 1 \
        test_only_offline_overtake_planner test_only_offline_pure_pursuit \
        test_only_offline_hybrid_control_mux "${recorder_name}"
    assert_topic_contract safety "${PRIVATE_ROOT}/planner/safety_constraint" 1 \
        test_only_offline_overtake_planner test_only_offline_hybrid_control_mux "${recorder_name}"
    grant_subscribers=(test_only_offline_overtake_planner "${recorder_name}")
else
    assert_topic_contract plan "${PRIVATE_ROOT}/planner/plan" 0 "" \
        test_only_offline_pure_pursuit test_only_offline_hybrid_control_mux
    assert_topic_contract safety "${PRIVATE_ROOT}/planner/safety_constraint" 0 "" \
        test_only_offline_hybrid_control_mux
    grant_subscribers=("${recorder_name}")
fi
assert_topic_contract pp_ack "${PRIVATE_ROOT}/pp/free_run_execution_ack" 1 \
    test_only_offline_pure_pursuit test_only_offline_hybrid_control_mux "${recorder_name}"
assert_topic_contract mux_grant "${PRIVATE_ROOT}/mux/motion_authority_grant" 1 \
    test_only_offline_hybrid_control_mux "${grant_subscribers[@]}"
assert_topic_contract final_control "${final_topic}" 1 \
    test_only_offline_hybrid_control_mux "${recorder_name}"

play_command=(ros2 bag play "${SOURCE_BAG}" --topics)
play_command+=("${SOURCE_PLAY_TOPICS[@]}")
play_command+=(--remap)
play_command+=("${SOURCE_REMAPS[@]}")
play_command+=(
    "__node:=test_only_offline_player"
    "__ns:=${PRIVATE_ROOT}/player"
    "/events/read_split:=${PRIVATE_ROOT}/player/events/read_split"
    "/rosbag2_player/status:=${PRIVATE_ROOT}/player/status"
)
printf '%q ' "${play_command[@]}" >"${ARTIFACT_DIR}/exact_replay_command.txt"
printf '\n' >>"${ARTIFACT_DIR}/exact_replay_command.txt"
setsid timeout --kill-after=2 "${TIMEOUT_SEC}" "${play_command[@]}"     >"${ARTIFACT_DIR}/bag_play.log" 2>&1 &
play_pid=$!
player_node="${PRIVATE_ROOT}/player/test_only_offline_player"
player_deadline=$((SECONDS + 5))
until timeout "${GRAPH_TIMEOUT_SEC}s" ros2 node list | grep -Fxq "${player_node}"; do
    kill -0 "${play_pid}" 2>/dev/null || fail_closed "player_exited_before_graph_audit"
    (( SECONDS < player_deadline )) || fail_closed "player_graph_ready_timeout"
    sleep 0.05
done
expected_playing_nodes=("${expected_recording_nodes[@]}" "${player_node}")
assert_exact_nodes playing "${expected_playing_nodes[@]}"
assert_all_topics_private /events/write_split
assert_no_live_interfaces
snapshot_graph "${ARTIFACT_DIR}/ros_graph_playing.txt"
if wait "${play_pid}"; then
    play_status=0
else
    play_status=$?
fi
completed_play_pid="${play_pid}"
play_pid=0
assert_group_gone "${completed_play_pid}" play
if (( play_status != 0 )); then
    fail_closed "bag_play_failed:${play_status}"
fi
sleep 1
completed_record_pid="${record_pid}"
stop_group "${record_pid}"
record_pid=0
assert_group_gone "${completed_record_pid}" recorder
completed_launch_pid="${launch_pid}"
stop_group "${launch_pid}"
launch_pid=0
assert_group_gone "${completed_launch_pid}" launch

cleanup_graph_deadline=$((SECONDS + 30))
while true; do
    snapshot_graph "${ARTIFACT_DIR}/ros_graph_after.txt"
    if diff -u "${ARTIFACT_DIR}/ros_graph_before.txt" \
        "${ARTIFACT_DIR}/ros_graph_after.txt" \
        >"${ARTIFACT_DIR}/ros_graph_before_after.diff"; then
        break
    fi
    (( SECONDS < cleanup_graph_deadline )) || fail_closed "graph_residue_after_replay"
    sleep 0.2
done
[[ -f "${OUTPUT_METADATA}" ]] || fail_closed "output_metadata_missing"

python3 "${CONTRACT}" --phase final --mode "${MODE}"     --private-root "${PRIVATE_ROOT}" --source-metadata "${SOURCE_METADATA}"     --source-mcap "${SOURCE_MCAP}"     --preflight-result "${ARTIFACT_DIR}/preflight.json"     --output-metadata "${OUTPUT_METADATA}"     --result "${ARTIFACT_DIR}/result.json" "${config_args[@]}" ||     fail_closed "output_contract_hold"
cleanup_complete=true
echo "TEST_ONLY_OFFLINE_REPLAY_PASS mode=${MODE} artifact=${ARTIFACT_DIR}"

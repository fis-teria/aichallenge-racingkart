#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [ "${AWSIM_HELPER_REAPER_ACTIVE:-}" != "1" ]; then
    export AWSIM_HELPER_REAPER_ACTIVE=1
    exec /usr/bin/python3 "${script_dir}/helper_process_reaper.py" \
        --grace-sec 2 \
        --deadline-monotonic-ns "${AIC_GATE_DEADLINE_MONOTONIC_NS:-0}" \
        -- /usr/bin/bash "${script_dir}/$(basename -- "${BASH_SOURCE[0]}")" "$@"
fi

source /opt/ros/humble/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

admin_state_topic="${AWSIM_ADMIN_STATE_TOPIC:-/admin/awsim/state}"
admin_start_topic="${AWSIM_ADMIN_START_TOPIC:-/admin/awsim/start}"
admin_reset_topic="${AWSIM_ADMIN_RESET_TOPIC:-/admin/awsim/reset}"
initialization_ready_topic="${AUTOSTART_INITIALIZATION_READY_TOPIC:-/autostart/initialization_ready}"
official_start_service="${AUTOSTART_OFFICIAL_START_SERVICE:-/autostart/official_start}"
race_arm_topic="${AUTOSTART_RACE_ARM_TOPIC:-/overtake/race_armed}"
ready_domains_csv="${AWSIM_READY_DOMAINS:-1}"
start_mode="$(tr '[:upper:]' '[:lower:]' <<<"${AWSIM_START_MODE:-sync}")"
timeout_sec="${AWSIM_START_TIMEOUT_SEC:-180}"
poll_interval_sec="${AWSIM_START_POLL_INTERVAL_SEC:-0.5}"
publisher_timeout_sec="${AWSIM_START_PUBLISH_TIMEOUT_SEC:-10}"
race_arm_timeout_sec="${AWSIM_RACE_ARM_TIMEOUT_SEC:-10}"
official_start_call_timeout_sec="${AWSIM_OFFICIAL_START_CALL_TIMEOUT_SEC:-10}"
official_start_rollback_timeout_sec="${AWSIM_OFFICIAL_START_ROLLBACK_TIMEOUT_SEC:-5}"
child_reap_timeout_sec="${AWSIM_CHILD_REAP_TIMEOUT_SEC:-15}"
gate_deadline_monotonic_ns="${AIC_GATE_DEADLINE_MONOTONIC_NS:-}"
kill_after_grace_sec=2
official_start_service_waiter="${script_dir}/wait_for_typed_service.py"
admin_state_observer="${script_dir}/admin_state_observer.py"
race_arm_state_observer="${script_dir}/race_arm_observer.py"
vehicle_readiness_observer="${script_dir}/vehicle_readiness_observer.py"
admin_observer_pid=0
admin_observer_dir=""
admin_observer_token=""
admin_observer_watermark_seq=0
admin_observer_state=""
admin_observer_seq=0
admin_observer_launch_monotonic_ns=0
declare -A race_arm_observer_pids=()
declare -A race_arm_observer_dirs=()
declare -A race_arm_observer_tokens=()
declare -A race_arm_observer_seqs=()
declare -A race_arm_observer_states=()
declare -A race_arm_observer_monotonic_ns=()
declare -A race_arm_observer_seen_true=()
declare -A race_arm_observer_saw_forbidden_state=()
declare -A race_arm_observer_launch_monotonic_ns=()
declare -A race_arm_observer_watermark_seqs=()
declare -A race_arm_observer_startup_watermark_seqs=()
declare -A race_arm_observer_pre_pulse_watermark_seqs=()
declare -A vehicle_readiness_observer_pids=()
declare -A vehicle_readiness_observer_dirs=()
declare -A vehicle_readiness_observer_tokens=()
declare -A vehicle_readiness_observer_launch_monotonic_ns=()
declare -A vehicle_readiness_observer_seqs=()
declare -A vehicle_readiness_observer_states=()
declare -A vehicle_readiness_observer_state_seqs=()
declare -A vehicle_readiness_observer_initialization=()
declare -A vehicle_readiness_observer_initialization_seqs=()
declare -A vehicle_readiness_observer_monotonic_ns=()
declare -A vehicle_readiness_observer_seen_start=()
declare -A vehicle_readiness_observer_saw_invalid_vehicle=()
declare -A vehicle_readiness_observer_saw_initialization_false=()
declare -A vehicle_readiness_observer_watermark_seqs=()
declare -A vehicle_readiness_initialization_watermark_seqs=()
vehicle_readiness_watermarks_saved=false
declare -A official_start_call_begin_monotonic_ns=()
official_start_last_dispatch_monotonic_ns=0

if ! [[ "${timeout_sec}" =~ ^[0-9]+$ ]] || [ "${timeout_sec}" -le 0 ]; then
    echo "[awsim-start][ERROR] AWSIM_START_TIMEOUT_SEC must be a positive integer: ${timeout_sec}" >&2
    exit 2
fi
if ! [[ "${publisher_timeout_sec}" =~ ^[0-9]+$ ]] || \
    [ "${publisher_timeout_sec}" -le 0 ]; then
    echo "[awsim-start][ERROR] AWSIM_START_PUBLISH_TIMEOUT_SEC must be a positive integer: ${publisher_timeout_sec}" >&2
    exit 2
fi
if ! [[ "${race_arm_timeout_sec}" =~ ^[0-9]+$ ]] || \
    [ "${race_arm_timeout_sec}" -le 0 ]; then
    echo "[awsim-start][ERROR] AWSIM_RACE_ARM_TIMEOUT_SEC must be a positive integer: ${race_arm_timeout_sec}" >&2
    exit 2
fi
if ! [[ "${official_start_call_timeout_sec}" =~ ^[0-9]+$ ]] || \
    [ "${official_start_call_timeout_sec}" -le 0 ]; then
    echo "[awsim-start][ERROR] AWSIM_OFFICIAL_START_CALL_TIMEOUT_SEC must be a positive integer: ${official_start_call_timeout_sec}" >&2
    exit 2
fi
if ! [[ "${official_start_rollback_timeout_sec}" =~ ^[0-9]+$ ]] || \
    [ "${official_start_rollback_timeout_sec}" -le 0 ]; then
    echo "[awsim-start][ERROR] AWSIM_OFFICIAL_START_ROLLBACK_TIMEOUT_SEC must be a positive integer: ${official_start_rollback_timeout_sec}" >&2
    exit 2
fi
if ! [[ "${child_reap_timeout_sec}" =~ ^[0-9]+$ ]] || \
    [ "${child_reap_timeout_sec}" -le 0 ]; then
    echo "[awsim-start][ERROR] AWSIM_CHILD_REAP_TIMEOUT_SEC must be a positive integer: ${child_reap_timeout_sec}" >&2
    exit 2
fi

IFS=',' read -r -a ready_domains <<<"${ready_domains_csv}"
if [ "${#ready_domains[@]}" -eq 0 ]; then
    echo "[awsim-start][ERROR] AWSIM_READY_DOMAINS must not be empty" >&2
    exit 2
fi
for domain in "${ready_domains[@]}"; do
    if ! [[ "${domain}" =~ ^[1-9][0-9]*$ ]]; then
        echo "[awsim-start][ERROR] invalid vehicle domain in AWSIM_READY_DOMAINS: ${domain}" >&2
        exit 2
    fi
done
declare -A seen_ready_domains=()
for domain in "${ready_domains[@]}"; do
    if [ -n "${seen_ready_domains[${domain}]:-}" ]; then
        echo "[awsim-start][ERROR] duplicate vehicle domain in AWSIM_READY_DOMAINS: ${domain}" >&2
        exit 2
    fi
    seen_ready_domains["${domain}"]=1
done
case "${start_mode}" in
sync | count)
    ;;
*)
    echo "[awsim-start][ERROR] unsupported AWSIM_START_MODE for automated start: ${start_mode}" >&2
    exit 2
    ;;
esac

now_monotonic_ns() {
    /usr/bin/python3 -c 'import time; print(time.monotonic_ns())'
}

current_monotonic_ns="$(now_monotonic_ns)"
if [ -n "${gate_deadline_monotonic_ns}" ]; then
    if ! [[ "${gate_deadline_monotonic_ns}" =~ ^[1-9][0-9]*$ ]] || \
        [ "${gate_deadline_monotonic_ns}" -le "${current_monotonic_ns}" ]; then
        echo "[awsim-start][ERROR] invalid or expired AIC_GATE_DEADLINE_MONOTONIC_NS" >&2
        exit 2
    fi
    helper_deadline_monotonic_ns="${gate_deadline_monotonic_ns}"
else
    helper_deadline_monotonic_ns=$((current_monotonic_ns + timeout_sec * 1000000000))
fi
# Each domain rollback worker must receive an accepted false service response
# before it can accept a later false observation.  Domain workers and reset run
# concurrently, while service + causal confirmation are serial within a worker.
service_false_bound_sec=$((official_start_rollback_timeout_sec + kill_after_grace_sec))
reset_bound_sec=$((10 + kill_after_grace_sec))
rollback_domain_bound_sec=$((service_false_bound_sec + race_arm_timeout_sec))
rollback_parallel_bound_sec="${rollback_domain_bound_sec}"
if [ "${reset_bound_sec}" -gt "${rollback_parallel_bound_sec}" ]; then
    rollback_parallel_bound_sec="${reset_bound_sec}"
fi
pulse_parallel_bound_sec="${reset_bound_sec}"
if [ "${race_arm_timeout_sec}" -gt "${pulse_parallel_bound_sec}" ]; then
    pulse_parallel_bound_sec="${race_arm_timeout_sec}"
fi
# Process groups receive TERM together.  Waiting longer than the transport's
# TERM->KILL grace cannot improve ownership and only makes the 60-second Gate
# infeasible, so the configured legacy child bound is a cap, not a serial
# per-child reservation.
global_reap_timeout_sec="${child_reap_timeout_sec}"
if [ "${kill_after_grace_sec}" -lt "${global_reap_timeout_sec}" ]; then
    global_reap_timeout_sec="${kill_after_grace_sec}"
fi
# After the Helper's internal global reap, the Linux subreaper has one final
# TERM grace plus a one-second KILL/reap pass.  Reserve both serial phases and
# constrain the subreaper with the same absolute Gate deadline.
subreaper_drain_reserve_sec=$((kill_after_grace_sec + 1))
observer_reap_reserve_sec=$((global_reap_timeout_sec + subreaper_drain_reserve_sec))
rollback_reserve_sec=$((rollback_parallel_bound_sec + observer_reap_reserve_sec + 1))
pulse_rollback_reserve_sec=$((pulse_parallel_bound_sec + observer_reap_reserve_sec + 1))
required_failure_reserve_sec=0

bounded_operation_timeout() {
    local configured_timeout_sec="$1"
    local reserve_sec="${2:-0}"
    local kill_grace_sec="${3:-0}"
    if [ "${required_failure_reserve_sec}" -gt "${reserve_sec}" ]; then
        reserve_sec="${required_failure_reserve_sec}"
    fi
    /usr/bin/python3 - "${helper_deadline_monotonic_ns}" \
        "${configured_timeout_sec}" "${reserve_sec}" "${kill_grace_sec}" <<'PY'
import sys
import time

deadline_ns = int(sys.argv[1])
configured = float(sys.argv[2])
reserve = float(sys.argv[3])
kill_grace = float(sys.argv[4])
remaining = (
    (deadline_ns - time.monotonic_ns()) / 1_000_000_000
    - reserve
    - kill_grace
)
if remaining <= 0:
    raise SystemExit(1)
print(f"{min(configured, remaining):.6f}")
PY
}

deadline_has_budget() {
    bounded_operation_timeout 86400 "${1:-0}" >/dev/null
}

read_admin_state() {
    local output read_timeout
    read_timeout="$(bounded_operation_timeout 4 0 "${kill_after_grace_sec}")" || return 0
    output="$(
        timeout --kill-after=2 "${read_timeout}" ros2 topic echo --no-daemon --once \
            --qos-reliability reliable \
            --qos-durability transient_local \
            --qos-history keep_last \
            --qos-depth 1 \
            "${admin_state_topic}" 2>/dev/null || true
    )"
    awk -F ': *' '/^data:/ {print $2; exit}' <<<"${output}" \
        | tr -cd '[:alnum:]' \
        | tr '[:upper:]' '[:lower:]'
}

admin_observer_snapshot() {
    if [ "${admin_observer_pid}" -le 0 ] || ! kill -0 "${admin_observer_pid}" 2>/dev/null; then
        return 1
    fi
    local snapshot
    local -a transition_args=()
    if [ "${admin_observer_watermark_seq}" -gt 0 ]; then
        transition_args=(--watermark-seq "${admin_observer_watermark_seq}")
    fi
    if ! snapshot="$(python3 "${admin_state_observer}" --snapshot --token "${admin_observer_token}" \
        --expected-pid "${admin_observer_pid}" --min-started-monotonic-ns "${admin_observer_launch_monotonic_ns}" \
        "${transition_args[@]}" \
        --jsonl "${admin_observer_dir}/events.jsonl" --ready "${admin_observer_dir}/lifecycle.json")"; then
        return 1
    fi
    read -r admin_observer_seq admin_observer_state _ <<<"${snapshot}"
    [[ "${admin_observer_seq}" =~ ^[0-9]+$ ]] || return 1
    [[ "${admin_observer_state}" =~ ^[a-z0-9]+$ ]] || return 1
}

start_admin_observer() {
    admin_observer_dir="$(mktemp -d /tmp/awsim-admin-observer.XXXXXX)"
    admin_observer_token="$(od -An -N16 -tx1 /dev/urandom | tr -d ' \n')"
    admin_observer_launch_monotonic_ns="$(/usr/bin/python3 -c 'import time; print(time.monotonic_ns())')"
    setsid python3 "${admin_state_observer}" --token "${admin_observer_token}" \
        --jsonl "${admin_observer_dir}/events.jsonl" \
        --ready "${admin_observer_dir}/lifecycle.json" --topic "${admin_state_topic}" &
    admin_observer_pid=$!
    local ready_deadline=$((SECONDS + publisher_timeout_sec))
    while [ "${SECONDS}" -lt "${ready_deadline}" ] && deadline_has_budget 0; do
        if admin_observer_snapshot; then
            return 0
        fi
        if ! kill -0 "${admin_observer_pid}" 2>/dev/null; then
            return 1
        fi
        sleep 0.1
    done
    return 1
}

save_start_observer_watermark() {
    local expected_state="$1"
    case "${expected_state}" in
    ready | waitstart)
        ;;
    *)
        return 1
        ;;
    esac
    if ! admin_observer_snapshot || \
        [ "${admin_observer_state}" != "${expected_state}" ]; then
        echo "[awsim-start][ERROR] refusing pulse without same-token observer ${expected_state}" >&2
        return 1
    fi
    admin_observer_watermark_seq="${admin_observer_seq}"
}

cleanup_observer_dir() {
    local observer_dir="$1"
    local expected_prefix="$2"
    case "${observer_dir}" in
    "/tmp/${expected_prefix}."??????)
        ;;
    *)
        echo "[awsim-start][ERROR] refusing unsafe observer directory cleanup: ${observer_dir}" >&2
        return 1
        ;;
    esac
    [ -d "${observer_dir}" ] || return 0
    (
        shopt -s nullglob dotglob
        local entry basename
        for entry in "${observer_dir}"/*; do
            basename="${entry##*/}"
            case "${basename}" in
            events.jsonl | lifecycle.json | .lifecycle.json.*.tmp)
                ;;
            *)
                echo "[awsim-start][ERROR] refusing observer directory cleanup with unexpected entry: ${entry}" >&2
                return 1
                ;;
            esac
            if [ ! -f "${entry}" ] && [ ! -L "${entry}" ]; then
                echo "[awsim-start][ERROR] refusing observer directory cleanup of non-file entry: ${entry}" >&2
                return 1
            fi
        done
        for entry in "${observer_dir}"/*; do
            rm -f -- "${entry}"
        done
        rmdir -- "${observer_dir}"
    )
}

race_arm_observer_snapshot() {
    local domain="$1"
    local history_watermark_seq="${2:-}"
    local forbidden_state="${3:-}"
    local pid="${race_arm_observer_pids[${domain}]:-0}"
    if [ "${pid}" -le 0 ] || ! kill -0 "${pid}" 2>/dev/null; then
        return 1
    fi
    local snapshot
    local -a history_args=()
    if [ -n "${history_watermark_seq}" ] || [ -n "${forbidden_state}" ]; then
        if ! [[ "${history_watermark_seq}" =~ ^[0-9]+$ ]] || \
            [ -z "${forbidden_state}" ]; then
            return 1
        fi
        history_args=(
            --history-watermark-seq "${history_watermark_seq}"
            --forbid-state-after-watermark "${forbidden_state}"
        )
    fi
    if ! snapshot="$(ROS_DOMAIN_ID="${domain}" python3 "${race_arm_state_observer}" \
        --snapshot --token "${race_arm_observer_tokens[${domain}]}" \
        --expected-pid "${pid}" \
        --expected-domain "${domain}" \
        --min-started-monotonic-ns "${race_arm_observer_launch_monotonic_ns[${domain}]}" \
        "${history_args[@]}" \
        --jsonl "${race_arm_observer_dirs[${domain}]}/events.jsonl" \
        --ready "${race_arm_observer_dirs[${domain}]}/lifecycle.json")"; then
        return 1
    fi
    local seq state monotonic_ns seen_true saw_forbidden_state
    read -r seq state monotonic_ns seen_true saw_forbidden_state <<<"${snapshot}"
    [[ "${seq}" =~ ^[0-9]+$ ]] || return 1
    [[ "${monotonic_ns}" =~ ^[0-9]+$ ]] || return 1
    case "${seen_true}:${saw_forbidden_state}" in
    false:false | false:true | true:false | true:true)
        ;;
    *)
        return 1
        ;;
    esac
    case "${state}" in
    empty | false | true)
        ;;
    *)
        return 1
        ;;
    esac
    race_arm_observer_seqs["${domain}"]="${seq}"
    race_arm_observer_states["${domain}"]="${state}"
    race_arm_observer_monotonic_ns["${domain}"]="${monotonic_ns}"
    race_arm_observer_seen_true["${domain}"]="${seen_true}"
    race_arm_observer_saw_forbidden_state["${domain}"]="${saw_forbidden_state}"
}

start_race_arm_observer_domain() {
    local domain="$1"
    local observer_dir observer_pid ready_deadline
    observer_dir="$(mktemp -d "/tmp/awsim-race-arm-observer.${domain}.XXXXXX")"
    race_arm_observer_dirs["${domain}"]="${observer_dir}"
    race_arm_observer_tokens["${domain}"]="$(od -An -N16 -tx1 /dev/urandom | tr -d ' \n')"
    race_arm_observer_launch_monotonic_ns["${domain}"]="$(/usr/bin/python3 -c 'import time; print(time.monotonic_ns())')"
    setsid env ROS_DOMAIN_ID="${domain}" python3 "${race_arm_state_observer}" \
        --domain-id "${domain}" \
        --token "${race_arm_observer_tokens[${domain}]}" \
        --jsonl "${observer_dir}/events.jsonl" \
        --ready "${observer_dir}/lifecycle.json" --topic "${race_arm_topic}" &
    observer_pid=$!
    race_arm_observer_pids["${domain}"]="${observer_pid}"
    race_arm_observer_watermark_seqs["${domain}"]=0
    race_arm_observer_startup_watermark_seqs["${domain}"]=0
    race_arm_observer_pre_pulse_watermark_seqs["${domain}"]=0
    official_start_call_begin_monotonic_ns["${domain}"]=0
    ready_deadline=$((SECONDS + publisher_timeout_sec))
    while [ "${SECONDS}" -lt "${ready_deadline}" ] && deadline_has_budget 0; do
        if race_arm_observer_snapshot "${domain}" \
            "${race_arm_observer_startup_watermark_seqs[${domain}]}" true; then
            return 0
        fi
        if ! kill -0 "${observer_pid}" 2>/dev/null; then
            return 1
        fi
        sleep 0.1
    done
    return 1
}

start_race_arm_observers() {
    local domain
    for domain in "${ready_domains[@]}"; do
        if ! start_race_arm_observer_domain "${domain}"; then
            echo "[awsim-start][ERROR] domain ${domain} persistent race-arm observer failed to become ready" >&2
            return 1
        fi
    done
}

wait_for_initial_race_arm_false() {
    local domain arm_state all_false
    local initial_deadline=$((SECONDS + race_arm_timeout_sec))
    while true; do
        all_false=true
        for domain in "${ready_domains[@]}"; do
            if [ "${race_arm_observer_states[${domain}]+present}" != "present" ]; then
                echo "[awsim-start][ERROR] domain ${domain} startup race-arm evidence is missing" >&2
                return 1
            fi
            arm_state="${race_arm_observer_states[${domain}]}"
            if [ "${race_arm_observer_seen_true[${domain}]:-false}" = true ] || \
                [ "${race_arm_observer_saw_forbidden_state[${domain}]:-false}" = true ]; then
                echo "[awsim-start][ERROR] domain ${domain} startup race-arm history observed forbidden true after startup watermark" >&2
                return 1
            fi
            case "${arm_state}" in
            false)
                ;;
            true)
                echo "[awsim-start][ERROR] domain ${domain} retained race arm is true at Helper startup; causal disarm precondition is impossible" >&2
                return 1
                ;;
            empty)
                all_false=false
                ;;
            *)
                echo "[awsim-start][ERROR] domain ${domain} startup race-arm evidence is invalid: ${arm_state:-missing}" >&2
                return 1
                ;;
            esac
        done
        if [ "${all_false}" = true ]; then
            return 0
        fi
        if [ "${SECONDS}" -ge "${initial_deadline}" ] || \
            ! deadline_has_budget "${observer_reap_reserve_sec}"; then
            echo "[awsim-start][ERROR] startup race-arm evidence did not become exact false in all domains within ${race_arm_timeout_sec}s" >&2
            return 1
        fi
        sleep 0.1
        for domain in "${ready_domains[@]}"; do
            if ! race_arm_observer_snapshot "${domain}" \
                "${race_arm_observer_startup_watermark_seqs[${domain}]}" true; then
                echo "[awsim-start][ERROR] domain ${domain} startup race-arm observer exited or emitted invalid/missing evidence" >&2
                return 1
            fi
        done
    done
}

save_race_arm_false_watermarks() {
    local domain
    for domain in "${ready_domains[@]}"; do
        if ! race_arm_observer_snapshot "${domain}" \
            "${race_arm_observer_pre_pulse_watermark_seqs[${domain}]:-0}" true || \
            [ "${race_arm_observer_states[${domain}]}" != "false" ] || \
            [ "${race_arm_observer_saw_forbidden_state[${domain}]}" = true ] || \
            [ "${race_arm_observer_seqs[${domain}]}" -le 0 ]; then
            echo "[awsim-start][ERROR] refusing official Start after pre-pulse race-arm history was not continuously false in domain ${domain}" >&2
            return 1
        fi
        race_arm_observer_watermark_seqs["${domain}"]="${race_arm_observer_seqs[${domain}]}"
    done
}

save_pre_pulse_race_arm_false_watermarks() {
    local domain
    for domain in "${ready_domains[@]}"; do
        if ! race_arm_observer_snapshot "${domain}" || \
            [ "${race_arm_observer_states[${domain}]}" != "false" ] || \
            [ "${race_arm_observer_seqs[${domain}]}" -le 0 ]; then
            echo "[awsim-start][ERROR] refusing Start pulse without observer-bound race arm false in domain ${domain}" >&2
            return 1
        fi
        race_arm_observer_pre_pulse_watermark_seqs["${domain}"]="${race_arm_observer_seqs[${domain}]}"
    done
}

verify_pre_pulse_race_arm_stayed_false() {
    local domain
    for domain in "${ready_domains[@]}"; do
        if ! race_arm_observer_snapshot "${domain}" \
            "${race_arm_observer_pre_pulse_watermark_seqs[${domain}]:-0}" true; then
            echo "[awsim-start][ERROR] pre-pulse race-arm history observer failed in domain ${domain}" >&2
            return 1
        fi
        if [ "${race_arm_observer_states[${domain}]}" != "false" ] || \
            [ "${race_arm_observer_saw_forbidden_state[${domain}]}" = true ]; then
            echo "[awsim-start][ERROR] race arm did not remain continuously false after the pre-pulse watermark in domain ${domain}" >&2
            return 1
        fi
    done
}

vehicle_readiness_observer_snapshot() {
    local domain="$1"
    local history_watermark_seq="${2:-}"
    local initialization_watermark_seq="${3:-}"
    local pid="${vehicle_readiness_observer_pids[${domain}]:-0}"
    if [ "${pid}" -le 0 ] || ! kill -0 "${pid}" 2>/dev/null; then
        return 1
    fi
    local snapshot
    local -a history_args=()
    if [ -n "${history_watermark_seq}" ] || [ -n "${initialization_watermark_seq}" ]; then
        if ! [[ "${history_watermark_seq}" =~ ^[0-9]+$ ]] || \
            ! [[ "${initialization_watermark_seq}" =~ ^[1-9][0-9]*$ ]]; then
            return 1
        fi
        history_args=(
            --history-watermark-seq "${history_watermark_seq}"
            --initialization-watermark-seq "${initialization_watermark_seq}"
        )
    fi
    if ! snapshot="$(ROS_DOMAIN_ID="${domain}" python3 "${vehicle_readiness_observer}" \
        --snapshot --token "${vehicle_readiness_observer_tokens[${domain}]}" \
        --expected-pid "${pid}" \
        --expected-domain "${domain}" \
        --min-started-monotonic-ns "${vehicle_readiness_observer_launch_monotonic_ns[${domain}]}" \
        "${history_args[@]}" \
        --jsonl "${vehicle_readiness_observer_dirs[${domain}]}/events.jsonl" \
        --ready "${vehicle_readiness_observer_dirs[${domain}]}/lifecycle.json")"; then
        return 1
    fi
    local seq vehicle_state vehicle_seq initialization_ready initialization_seq
    local monotonic_ns seen_start saw_invalid_vehicle saw_initialization_false
    local previous_seq
    read -r seq vehicle_state vehicle_seq initialization_ready initialization_seq \
        monotonic_ns seen_start saw_invalid_vehicle saw_initialization_false <<<"${snapshot}"
    [[ "${seq}" =~ ^[0-9]+$ ]] || return 1
    [[ "${vehicle_seq}" =~ ^[0-9]+$ ]] || return 1
    [[ "${initialization_seq}" =~ ^[0-9]+$ ]] || return 1
    [[ "${monotonic_ns}" =~ ^[0-9]+$ ]] || return 1
    case "${initialization_ready}" in empty | true | false) ;; *) return 1 ;; esac
    case "${seen_start}" in true | false) ;; *) return 1 ;; esac
    case "${saw_invalid_vehicle}" in true | false) ;; *) return 1 ;; esac
    case "${saw_initialization_false}" in true | false) ;; *) return 1 ;; esac
    case "${vehicle_state}" in
    empty | spawned | grounded | ready | start | lapcomplete | finish | finished | \
        finishall | finishedall | terminate | terminated)
        ;;
    *)
        return 1
        ;;
    esac
    previous_seq="${vehicle_readiness_observer_seqs[${domain}]:-0}"
    if [ "${seq}" -lt "${previous_seq}" ]; then
        return 1
    fi
    vehicle_readiness_observer_seqs["${domain}"]="${seq}"
    vehicle_readiness_observer_states["${domain}"]="${vehicle_state}"
    vehicle_readiness_observer_state_seqs["${domain}"]="${vehicle_seq}"
    vehicle_readiness_observer_initialization["${domain}"]="${initialization_ready}"
    vehicle_readiness_observer_initialization_seqs["${domain}"]="${initialization_seq}"
    vehicle_readiness_observer_monotonic_ns["${domain}"]="${monotonic_ns}"
    vehicle_readiness_observer_seen_start["${domain}"]="${seen_start}"
    vehicle_readiness_observer_saw_invalid_vehicle["${domain}"]="${saw_invalid_vehicle}"
    vehicle_readiness_observer_saw_initialization_false["${domain}"]="${saw_initialization_false}"
}

start_vehicle_readiness_observer_domain() {
    local domain="$1"
    local observer_dir observer_pid ready_deadline
    observer_dir="$(mktemp -d "/tmp/awsim-vehicle-readiness-observer.${domain}.XXXXXX")"
    vehicle_readiness_observer_dirs["${domain}"]="${observer_dir}"
    vehicle_readiness_observer_tokens["${domain}"]="$(od -An -N16 -tx1 /dev/urandom | tr -d ' \n')"
    vehicle_readiness_observer_launch_monotonic_ns["${domain}"]="$(/usr/bin/python3 -c 'import time; print(time.monotonic_ns())')"
    setsid env ROS_DOMAIN_ID="${domain}" python3 "${vehicle_readiness_observer}" \
        --domain-id "${domain}" \
        --token "${vehicle_readiness_observer_tokens[${domain}]}" \
        --jsonl "${observer_dir}/events.jsonl" \
        --ready "${observer_dir}/lifecycle.json" \
        --vehicle-topic /awsim/state \
        --initialization-topic "${initialization_ready_topic}" &
    observer_pid=$!
    vehicle_readiness_observer_pids["${domain}"]="${observer_pid}"
    vehicle_readiness_observer_watermark_seqs["${domain}"]=0
    vehicle_readiness_initialization_watermark_seqs["${domain}"]=0
    ready_deadline=$((SECONDS + publisher_timeout_sec))
    while [ "${SECONDS}" -lt "${ready_deadline}" ] && deadline_has_budget 0; do
        if vehicle_readiness_observer_snapshot "${domain}"; then
            return 0
        fi
        if ! kill -0 "${observer_pid}" 2>/dev/null; then
            return 1
        fi
        sleep 0.1
    done
    return 1
}

start_vehicle_readiness_observers() {
    local domain
    for domain in "${ready_domains[@]}"; do
        if ! start_vehicle_readiness_observer_domain "${domain}"; then
            echo "[awsim-start][ERROR] domain ${domain} persistent vehicle readiness observer failed to become ready" >&2
            return 1
        fi
    done
}

save_vehicle_readiness_watermarks() {
    [ "${vehicle_readiness_watermarks_saved}" = false ] || return 0
    local domain vehicle_state initialization_ready
    for domain in "${ready_domains[@]}"; do
        if ! vehicle_readiness_observer_snapshot "${domain}"; then
            echo "[awsim-start][ERROR] domain ${domain} vehicle readiness observer failed before Start pulse" >&2
            return 1
        fi
        vehicle_state="${vehicle_readiness_observer_states[${domain}]}"
        initialization_ready="${vehicle_readiness_observer_initialization[${domain}]}"
        case "${vehicle_state}" in
        grounded | ready)
            ;;
        *)
            echo "[awsim-start][ERROR] refusing Start pulse without pre-pulse startable vehicle domain ${domain}: ${vehicle_state:-empty}" >&2
            return 1
            ;;
        esac
        if [ "${initialization_ready}" != true ] || \
            [ "${vehicle_readiness_observer_initialization_seqs[${domain}]}" -le 0 ]; then
            echo "[awsim-start][ERROR] refusing Start pulse without pre-pulse initialization true in domain ${domain}" >&2
            return 1
        fi
        vehicle_readiness_observer_watermark_seqs["${domain}"]="${vehicle_readiness_observer_seqs[${domain}]}"
        vehicle_readiness_initialization_watermark_seqs["${domain}"]="${vehicle_readiness_observer_initialization_seqs[${domain}]}"
    done
    vehicle_readiness_watermarks_saved=true
}

active_child_pids=()
active_publisher_pid=0
launched_process_group_pid=0
official_start_attempted=false
official_start_confirmed=false
official_start_rollback_attempted=false
current_run_start_transition_proved=false
start_pulse_may_have_been_delivered=false
start_pulse_rollback_attempted=false
start_pulse_terminal_failed=false
startup_service_preflight_failed=false

launch_process_group() {
    # Non-interactive bash otherwise places every background function in the
    # helper's own process group.  Briefly enabling job control gives this
    # one job a distinct PGID equal to $!, so TERM/KILL covers its timeout
    # wrapper and every descendant rather than only the shell wrapper.
    set -m
    "$@" &
    launched_process_group_pid=$!
    set +m
}

terminate_and_reap_process_groups() {
    local reserve_sec="$1"
    shift
    local pid alive reap_deadline
    local -a pids=("$@")
    [ "${#pids[@]}" -gt 0 ] || return 0
    for pid in "${pids[@]}"; do
        if [ "${pid}" -gt 0 ]; then
            kill -TERM -- "-${pid}" 2>/dev/null || true
        fi
    done
    reap_deadline=$((SECONDS + global_reap_timeout_sec))
    while [ "${SECONDS}" -lt "${reap_deadline}" ] && \
        deadline_has_budget "${reserve_sec}"; do
        alive=false
        for pid in "${pids[@]}"; do
            if [ "${pid}" -gt 0 ] && kill -0 -- "-${pid}" 2>/dev/null; then
                alive=true
                break
            fi
        done
        [ "${alive}" = true ] || break
        sleep 0.05
    done
    for pid in "${pids[@]}"; do
        if [ "${pid}" -gt 0 ] && kill -0 -- "-${pid}" 2>/dev/null; then
            kill -KILL -- "-${pid}" 2>/dev/null || true
        fi
    done
    for pid in "${pids[@]}"; do
        if [ "${pid}" -gt 0 ]; then
            wait "${pid}" 2>/dev/null || true
        fi
    done
}

reap_active_children() {
    if [ "${#active_child_pids[@]}" -gt 0 ]; then
        terminate_and_reap_process_groups 0 "${active_child_pids[@]}"
    fi
    active_child_pids=()
}

stop_active_publisher() {
    if [ "${active_publisher_pid}" -gt 0 ]; then
        terminate_and_reap_process_groups 0 "${active_publisher_pid}"
        active_publisher_pid=0
    fi
}

stop_all_observers() {
    local domain pid
    local -a observer_pids=()
    if [ "${admin_observer_pid}" -gt 0 ]; then
        observer_pids+=("${admin_observer_pid}")
    fi
    for domain in "${ready_domains[@]}"; do
        pid="${race_arm_observer_pids[${domain}]:-0}"
        if [ "${pid}" -gt 0 ]; then
            observer_pids+=("${pid}")
        fi
        pid="${vehicle_readiness_observer_pids[${domain}]:-0}"
        if [ "${pid}" -gt 0 ]; then
            observer_pids+=("${pid}")
        fi
    done
    # This is the sole observer reap window regardless of domain count.
    required_failure_reserve_sec=0
    if [ "${#observer_pids[@]}" -gt 0 ]; then
        terminate_and_reap_process_groups 0 "${observer_pids[@]}"
    fi
    admin_observer_pid=0
    for domain in "${ready_domains[@]}"; do
        race_arm_observer_pids["${domain}"]=0
        vehicle_readiness_observer_pids["${domain}"]=0
    done
    if [ -n "${admin_observer_dir}" ]; then
        cleanup_observer_dir "${admin_observer_dir}" "awsim-admin-observer" || \
            echo "[awsim-start][WARN] failed to remove bounded admin observer directory: ${admin_observer_dir}" >&2
        admin_observer_dir=""
    fi
    for domain in "${ready_domains[@]}"; do
        if [ -n "${race_arm_observer_dirs[${domain}]:-}" ]; then
            cleanup_observer_dir "${race_arm_observer_dirs[${domain}]}" \
                "awsim-race-arm-observer.${domain}" || \
                echo "[awsim-start][WARN] failed to remove bounded race-arm observer directory for domain ${domain}" >&2
            race_arm_observer_dirs["${domain}"]=""
        fi
        if [ -n "${vehicle_readiness_observer_dirs[${domain}]:-}" ]; then
            cleanup_observer_dir "${vehicle_readiness_observer_dirs[${domain}]}" \
                "awsim-vehicle-readiness-observer.${domain}" || \
                echo "[awsim-start][WARN] failed to remove bounded vehicle readiness observer directory for domain ${domain}" >&2
            vehicle_readiness_observer_dirs["${domain}"]=""
        fi
    done
}

publish_start() {
    local pre_publish_state=""
    local publisher_pid=0
    local publisher_status=0
    local transport_timeout
    pre_publish_state="$(read_admin_state)"
    case "${pre_publish_state}" in
    ready | waitstart)
        ;;
    *)
        echo "[awsim-start][ERROR] refusing start publish from non-authoritative admin pre-state=${pre_publish_state:-empty}" >&2
        return 1
        ;;
    esac
    if ! save_start_observer_watermark "${pre_publish_state}"; then
        return 1
    fi
    if ! save_vehicle_readiness_watermarks; then
        return 1
    fi
    echo "[awsim-start] publish exactly one ${admin_start_topic}=true pulse with reliable/transient_local QoS"
    transport_timeout="$(bounded_operation_timeout "${publisher_timeout_sec}" "${pulse_rollback_reserve_sec}" "${kill_after_grace_sec}")" || return 1
    required_failure_reserve_sec="${pulse_rollback_reserve_sec}"
    start_pulse_may_have_been_delivered=true
    setsid timeout --kill-after=2 "${transport_timeout}" ros2 topic pub \
        -1 \
        --wait-matching-subscriptions 1 \
        --qos-reliability reliable \
        --qos-durability transient_local \
        "${admin_start_topic}" std_msgs/msg/Bool '{data: true}' &
    publisher_pid=$!
    active_publisher_pid="${publisher_pid}"
    if wait "${publisher_pid}"; then
        publisher_status=0
    else
        publisher_status=$?
    fi
    active_publisher_pid=0
    if [ "${publisher_status}" -ne 0 ]; then
        echo "[awsim-start][ERROR] one-shot Start pulse failed status=${publisher_status}; delivery is ambiguous" >&2
        start_pulse_terminal_failed=true
        rollback_start_pulse || true
        return 1
    fi
    echo "[awsim-start] one-shot Start pulse publisher completed"
}

publish_reset() {
    local reset_timeout
    reset_timeout="$(bounded_operation_timeout 10 0 "${kill_after_grace_sec}")" || return 1
    echo "[awsim-start] publish ${admin_reset_topic} to stop a partial start"
    timeout --kill-after=2 "${reset_timeout}" ros2 topic pub \
        --qos-reliability reliable \
        --qos-durability transient_local \
        -1 "${admin_reset_topic}" std_msgs/msg/Empty '{}'
}

rollback_start_pulse() {
    local pid failed=0
    local -a pids=()
    if [ "${start_pulse_rollback_attempted}" = true ]; then
        return 0
    fi
    start_pulse_rollback_attempted=true
    required_failure_reserve_sec="${observer_reap_reserve_sec}"
    launch_process_group publish_reset
    pids+=("${launched_process_group_pid}")
    active_child_pids=("${pids[@]}")
    launch_process_group wait_race_arm_all false false "${observer_reap_reserve_sec}"
    pids+=("${launched_process_group_pid}")
    active_child_pids=("${pids[@]}")
    for pid in "${pids[@]}"; do
        if ! wait "${pid}"; then
            failed=1
        fi
    done
    active_child_pids=()
    [ "${failed}" -eq 0 ]
}

check_official_start_service() {
    local domain="$1"
    local reserve_sec="${2:-0}"
    local service_timeout
    service_timeout="$(bounded_operation_timeout "${official_start_call_timeout_sec}" "${reserve_sec}" "${kill_after_grace_sec}")" || return 1
    if ! ROS_DOMAIN_ID="${domain}" \
        timeout --kill-after=2 "${service_timeout}" \
        python3 "${official_start_service_waiter}" \
        --service "${official_start_service}" \
        --timeout-sec "${service_timeout}"; then
        echo "[awsim-start][WARN] domain ${domain} typed service unavailable: ${official_start_service} expected=std_srvs/srv/SetBool" >&2
        return 1
    fi
}

check_all_official_start_services() {
    local reserve_sec="${1:-0}"
    local domain pid failed=0
    local -a pids=()
    for domain in "${ready_domains[@]}"; do
        launch_process_group check_official_start_service "${domain}" "${reserve_sec}"
        pids+=("${launched_process_group_pid}")
        active_child_pids=("${pids[@]}")
    done
    active_child_pids=("${pids[@]}")
    for pid in "${pids[@]}"; do
        if ! wait "${pid}"; then
            failed=1
        fi
    done
    active_child_pids=()
    [ "${failed}" -eq 0 ]
}

call_official_start_domain() {
    local domain="$1"
    local requested="$2"
    local output service_timeout configured_timeout_sec reserve_sec=0
    if [ "${requested}" = true ]; then
        reserve_sec="${rollback_reserve_sec}"
        configured_timeout_sec="${official_start_call_timeout_sec}"
    else
        configured_timeout_sec="${official_start_rollback_timeout_sec}"
    fi
    service_timeout="$(bounded_operation_timeout "${configured_timeout_sec}" "${reserve_sec}" "${kill_after_grace_sec}")" || {
        echo "[awsim-start][ERROR] domain ${domain} insufficient absolute deadline budget for official Start ${requested}" >&2
        return 1
    }
    official_start_last_dispatch_monotonic_ns="$(now_monotonic_ns)"
    if ! output="$(
        ROS_DOMAIN_ID="${domain}" timeout --kill-after=2 "${service_timeout}" ros2 service call \
            "${official_start_service}" std_srvs/srv/SetBool \
            "{data: ${requested}}" 2>&1
    )"; then
        echo "[awsim-start][ERROR] domain ${domain} official Start service call failed: ${output}" >&2
        return 1
    fi
    if ! grep -Eq 'success=(True|true)' <<<"${output}"; then
        echo "[awsim-start][ERROR] domain ${domain} official Start rejected: ${output}" >&2
        return 1
    fi
    echo "[awsim-start] domain ${domain} official Start ${requested}: accepted"
}

wait_race_arm_domain_after_monotonic() {
    local domain="$1"
    local expected="$2"
    local dispatch_monotonic_ns="$3"
    local deadline_reserve_sec="${4:-0}"
    local arm_state arm_monotonic_ns
    if [ "${dispatch_monotonic_ns}" -le 0 ]; then
        echo "[awsim-start][ERROR] domain ${domain} race-arm ${expected} wait has no service dispatch marker" >&2
        return 1
    fi
    local arm_deadline=$((SECONDS + race_arm_timeout_sec))
    while [ "${SECONDS}" -lt "${arm_deadline}" ] && \
        deadline_has_budget "${deadline_reserve_sec}"; do
        if ! race_arm_observer_snapshot "${domain}"; then
            echo "[awsim-start][ERROR] domain ${domain} race-arm observer exited or emitted invalid evidence" >&2
            return 1
        fi
        arm_state="${race_arm_observer_states[${domain}]}"
        arm_monotonic_ns="${race_arm_observer_monotonic_ns[${domain}]}"
        if [ "${arm_state}" = "${expected}" ] && \
            [ "${arm_monotonic_ns}" -gt "${dispatch_monotonic_ns}" ]; then
            return 0
        fi
        sleep 0.1
    done
    echo "[awsim-start][ERROR] domain ${domain} ${race_arm_topic} did not report post-dispatch ${expected}" >&2
    return 1
}

rollback_official_start_domain() {
    local domain="$1"
    local false_dispatch_monotonic_ns
    if ! call_official_start_domain "${domain}" false; then
        return 1
    fi
    false_dispatch_monotonic_ns="${official_start_last_dispatch_monotonic_ns}"
    wait_race_arm_domain_after_monotonic \
        "${domain}" false "${false_dispatch_monotonic_ns}" "${observer_reap_reserve_sec}"
}

call_official_start_all() {
    local requested="$1"
    local domain pid failed=0
    local -a pids=()
    for domain in "${ready_domains[@]}"; do
        if [ "${requested}" = true ]; then
            official_start_call_begin_monotonic_ns["${domain}"]="$(/usr/bin/python3 -c 'import time; print(time.monotonic_ns())')"
        fi
        launch_process_group call_official_start_domain "${domain}" "${requested}"
        pids+=("${launched_process_group_pid}")
        active_child_pids=("${pids[@]}")
    done
    active_child_pids=("${pids[@]}")
    for pid in "${pids[@]}"; do
        if ! wait "${pid}"; then
            failed=1
        fi
    done
    active_child_pids=()
    [ "${failed}" -eq 0 ]
}

wait_race_arm_domain() {
    local domain="$1"
    local expected="$2"
    local require_post_watermark="${3:-false}"
    local arm_state arm_seq arm_monotonic_ns watermark_seq=0 service_begin_monotonic_ns=0
    local deadline_reserve_sec="${4:-0}"
    if [ "${require_post_watermark}" = true ]; then
        if [ "${rollback_reserve_sec}" -gt "${deadline_reserve_sec}" ]; then
            deadline_reserve_sec="${rollback_reserve_sec}"
        fi
        watermark_seq="${race_arm_observer_watermark_seqs[${domain}]:-0}"
        service_begin_monotonic_ns="${official_start_call_begin_monotonic_ns[${domain}]:-0}"
        if [ "${watermark_seq}" -le 0 ] || [ "${service_begin_monotonic_ns}" -le 0 ]; then
            echo "[awsim-start][ERROR] domain ${domain} race-arm true wait has no false watermark or service dispatch marker" >&2
            return 1
        fi
    fi
    local arm_deadline=$((SECONDS + race_arm_timeout_sec))
    while [ "${SECONDS}" -lt "${arm_deadline}" ] && \
        deadline_has_budget "${deadline_reserve_sec}"; do
        if ! race_arm_observer_snapshot "${domain}"; then
            echo "[awsim-start][ERROR] domain ${domain} race-arm observer exited or emitted invalid evidence" >&2
            return 1
        fi
        arm_state="${race_arm_observer_states[${domain}]}"
        arm_seq="${race_arm_observer_seqs[${domain}]}"
        arm_monotonic_ns="${race_arm_observer_monotonic_ns[${domain}]}"
        if [ "${arm_state}" = "${expected}" ] && \
            { [ "${require_post_watermark}" != true ] || \
                { [ "${arm_seq}" -gt "${watermark_seq}" ] && \
                    [ "${arm_monotonic_ns}" -gt "${service_begin_monotonic_ns}" ]; }; }; then
            return 0
        fi
        sleep 0.1
    done
    echo "[awsim-start][ERROR] domain ${domain} ${race_arm_topic} did not become ${expected}" >&2
    return 1
}

wait_race_arm_all() {
    local expected="$1"
    local require_post_watermark="${2:-false}"
    local deadline_reserve_sec="${3:-0}"
    local domain arm_state arm_seq arm_monotonic_ns watermark_seq service_begin_monotonic_ns
    local all_matched
    if [ "${require_post_watermark}" = true ] && \
        [ "${rollback_reserve_sec}" -gt "${deadline_reserve_sec}" ]; then
        deadline_reserve_sec="${rollback_reserve_sec}"
    fi
    local arm_deadline=$((SECONDS + race_arm_timeout_sec))
    while [ "${SECONDS}" -lt "${arm_deadline}" ] && \
        deadline_has_budget "${deadline_reserve_sec}"; do
        all_matched=true
        for domain in "${ready_domains[@]}"; do
            if ! race_arm_observer_snapshot "${domain}"; then
                echo "[awsim-start][ERROR] domain ${domain} race-arm observer exited or emitted invalid evidence" >&2
                return 1
            fi
            arm_state="${race_arm_observer_states[${domain}]}"
            arm_seq="${race_arm_observer_seqs[${domain}]}"
            arm_monotonic_ns="${race_arm_observer_monotonic_ns[${domain}]}"
            if [ "${require_post_watermark}" = true ]; then
                watermark_seq="${race_arm_observer_watermark_seqs[${domain}]:-0}"
                service_begin_monotonic_ns="${official_start_call_begin_monotonic_ns[${domain}]:-0}"
                if [ "${watermark_seq}" -le 0 ] || \
                    [ "${service_begin_monotonic_ns}" -le 0 ]; then
                    echo "[awsim-start][ERROR] domain ${domain} race-arm true wait has no false watermark or service dispatch marker" >&2
                    return 1
                fi
            else
                watermark_seq=0
                service_begin_monotonic_ns=0
            fi
            if [ "${arm_state}" != "${expected}" ] || \
                { [ "${require_post_watermark}" = true ] && \
                    { [ "${arm_seq}" -le "${watermark_seq}" ] || \
                        [ "${arm_monotonic_ns}" -le "${service_begin_monotonic_ns}" ]; }; }; then
                all_matched=false
            fi
        done
        if [ "${all_matched}" = true ]; then
            return 0
        fi
        sleep 0.1
    done
    echo "[awsim-start][ERROR] one or more domains ${race_arm_topic} did not become ${expected}" >&2
    return 1
}

rollback_official_start() {
    local domain pid rollback_failed=0
    local -a pids=()
    official_start_rollback_attempted=true
    start_pulse_rollback_attempted=true
    required_failure_reserve_sec="${observer_reap_reserve_sec}"
    echo "[awsim-start][ERROR] official Start was not accepted atomically; disarm and reset all vehicles" >&2
    for domain in "${ready_domains[@]}"; do
        launch_process_group rollback_official_start_domain "${domain}"
        pids+=("${launched_process_group_pid}")
        active_child_pids=("${pids[@]}")
    done
    launch_process_group publish_reset
    pids+=("${launched_process_group_pid}")
    active_child_pids=("${pids[@]}")
    for pid in "${pids[@]}"; do
        if ! wait "${pid}"; then
            rollback_failed=1
        fi
    done
    active_child_pids=()
    return "${rollback_failed}"
}

handle_start_helper_signal() {
    local signal_name="$1"
    local exit_status=130
    if [ "${signal_name}" = "TERM" ]; then
        exit_status=143
    fi
    trap - INT TERM
    stop_active_publisher
    # Service calls are individually timeout-bounded. Reap them before
    # rollback so a delayed true request cannot race after the false request.
    reap_active_children
    if [ "${official_start_confirmed}" = false ]; then
        if [ "${official_start_attempted}" = true ] && \
            [ "${official_start_rollback_attempted}" = false ]; then
            rollback_official_start || true
        elif [ "${start_pulse_may_have_been_delivered}" = true ] && \
            [ "${start_pulse_rollback_attempted}" = false ]; then
            rollback_start_pulse || true
        fi
    fi
    # Race-arm observers remain authoritative through disarm/reset rollback,
    # then every observer process group is reaped under one global bound.
    stop_all_observers
    exit "${exit_status}"
}

handle_start_helper_exit() {
    local exit_status=$?
    trap - EXIT
    stop_active_publisher
    reap_active_children
    if [ "${exit_status}" -ne 0 ] && \
        [ "${official_start_confirmed}" = false ]; then
        if [ "${official_start_attempted}" = true ] && \
            [ "${official_start_rollback_attempted}" = false ]; then
            rollback_official_start || true
        elif [ "${start_pulse_may_have_been_delivered}" = true ] && \
            [ "${start_pulse_rollback_attempted}" = false ]; then
            rollback_start_pulse || true
        fi
    fi
    stop_all_observers
    return "${exit_status}"
}

trap 'handle_start_helper_signal INT' INT
trap 'handle_start_helper_signal TERM' TERM
trap handle_start_helper_exit EXIT

# Observer ownership is a side effect.  Refuse to spawn even the first
# observer unless both the Helper's internal reap and the outer subreaper
# drain fit inside the shared absolute deadline.
required_failure_reserve_sec="${observer_reap_reserve_sec}"
if ! deadline_has_budget "${observer_reap_reserve_sec}"; then
    echo "[awsim-start][ERROR] refusing observer startup without cleanup deadline reserve=${observer_reap_reserve_sec}s" >&2
    exit 1
fi
if ! start_admin_observer; then
    echo "[awsim-start][ERROR] persistent admin observer failed to become ready" >&2
    exit 1
fi
if ! start_vehicle_readiness_observers; then
    exit 1
fi
if ! start_race_arm_observers; then
    exit 1
fi
# Retained true is terminal.  A valid empty sample can mean DDS evidence has
# not arrived yet, so wait only inside the shared race-arm window while
# preserving the five-second cleanup reserve.  No pulse/reset/service occurs
# before every domain has reported exact false.
if ! wait_for_initial_race_arm_false; then
    exit 1
fi

last_state=""
declare -A last_vehicle_states=()
declare -A last_initialization_ready=()

observe_vehicle_states() {
    all_vehicle_startable=true
    all_vehicle_ready=true
    all_vehicle_official_startable=true
    all_vehicle_initialization_ready=true
    all_vehicle_started=true
    all_vehicle_start_history_valid=true
    local domain vehicle_state initialization_ready
    for domain in "${ready_domains[@]}"; do
        if [ "${vehicle_readiness_watermarks_saved}" = true ]; then
            if ! vehicle_readiness_observer_snapshot "${domain}" \
                "${vehicle_readiness_observer_watermark_seqs[${domain}]}" \
                "${vehicle_readiness_initialization_watermark_seqs[${domain}]}"; then
                echo "[awsim-start][ERROR] domain ${domain} persistent vehicle readiness observer died or emitted malformed/regressed evidence" >&2
                exit 1
            fi
        elif ! vehicle_readiness_observer_snapshot "${domain}"; then
            echo "[awsim-start][ERROR] domain ${domain} persistent vehicle readiness observer died or emitted malformed/regressed evidence" >&2
            exit 1
        fi
        vehicle_state="${vehicle_readiness_observer_states[${domain}]}"
        if [ -n "${vehicle_state}" ] && [ "${vehicle_state}" != "${last_vehicle_states[${domain}]:-}" ]; then
            echo "[awsim-start] vehicle domain ${domain} state: ${vehicle_state}"
            last_vehicle_states[${domain}]="${vehicle_state}"
        fi
        case "${vehicle_state}" in
        grounded | ready)
            ;;
        *)
            all_vehicle_startable=false
            ;;
        esac
        if [ "${vehicle_state}" != "ready" ]; then
            all_vehicle_ready=false
        fi
        case "${vehicle_state}" in
        ready | start)
            ;;
        *)
            all_vehicle_official_startable=false
            ;;
        esac
        initialization_ready="${vehicle_readiness_observer_initialization[${domain}]}"
        if [ -n "${initialization_ready}" ] && [ "${initialization_ready}" != "${last_initialization_ready[${domain}]:-}" ]; then
            echo "[awsim-start] vehicle domain ${domain} initialization ready: ${initialization_ready}"
            last_initialization_ready[${domain}]="${initialization_ready}"
        fi
        if [ "${vehicle_readiness_watermarks_saved}" = true ] && \
            [ "${vehicle_readiness_observer_saw_invalid_vehicle[${domain}]}" = true ]; then
            echo "[awsim-start][ERROR] domain ${domain} entered a forbidden vehicle state after the one-shot Start watermark" >&2
            exit 1
        fi
        if [ "${vehicle_readiness_watermarks_saved}" = true ] && \
            [ "${vehicle_readiness_observer_saw_initialization_false[${domain}]}" = true ]; then
            echo "[awsim-start][ERROR] domain ${domain} initialization readiness became false after the pre-pulse true watermark" >&2
            exit 1
        fi
        if [ "${initialization_ready}" != "true" ]; then
            all_vehicle_initialization_ready=false
        fi
        case "${vehicle_state}" in
        start | lapcomplete | finish)
            ;;
        *)
            all_vehicle_started=false
            ;;
        esac
        if [ "${vehicle_readiness_watermarks_saved}" != true ] || \
            [ "${vehicle_readiness_observer_seen_start[${domain}]}" != true ] || \
            [ "${vehicle_readiness_observer_saw_invalid_vehicle[${domain}]}" = true ]; then
            all_vehicle_start_history_valid=false
        fi
    done
}

wait_for_one_shot_start_barrier() {
    local barrier_deadline=$((SECONDS + publisher_timeout_sec))
    local observed_admin_state=""

    while [ "${SECONDS}" -lt "${barrier_deadline}" ] && \
        deadline_has_budget "${pulse_rollback_reserve_sec}"; do
        if ! admin_observer_snapshot; then
            echo "[awsim-start][ERROR] admin observer exited or emitted invalid evidence during one-shot Start barrier" >&2
            return 1
        fi
        observed_admin_state="${admin_observer_state}"
        observe_vehicle_states
        if [ "${all_vehicle_initialization_ready}" != true ]; then
            echo "[awsim-start][ERROR] initialization readiness was lost during one-shot Start barrier" >&2
            return 1
        fi
        if ! verify_pre_pulse_race_arm_stayed_false; then
            return 1
        fi
        # Individual ROS reads are timeout-bounded but may consume the
        # remaining barrier budget.  Never accept admin Start after the
        # overall deadline merely because this sample began before it.
        if [ "${SECONDS}" -ge "${barrier_deadline}" ]; then
            break
        fi
        case "${observed_admin_state}" in
        start)
            if [ "${admin_observer_seq}" -le "${admin_observer_watermark_seq}" ]; then
                echo "[awsim-start][ERROR] rejecting pre-watermark observer Start" >&2
                return 1
            fi
            if [ "${all_vehicle_start_history_valid}" = true ]; then
                current_run_start_transition_proved=true
                return 0
            fi
            ;;
        ready | waitstart)
            ;;
        *)
            echo "[awsim-start][ERROR] one-shot Start admin barrier left Ready/WaitStart/Start: observed=${observed_admin_state:-empty}" >&2
            return 1
            ;;
        esac
        sleep 0.1
    done
    echo "[awsim-start][ERROR] one-shot Start did not produce post-watermark vehicle Start history and admin Start within ${publisher_timeout_sec}s" >&2
    return 1
}

publish_official_one_shot_start() {
    observe_vehicle_states
    if [ "${all_vehicle_startable}" != true ] || \
        [ "${all_vehicle_initialization_ready}" != true ]; then
        return 1
    fi
    if ! check_all_official_start_services "${pulse_rollback_reserve_sec}"; then
        startup_service_preflight_failed=true
        echo "[awsim-start][ERROR] official Start service preflight failed before any Start pulse; terminal" >&2
        return 1
    fi
    if ! wait_race_arm_all false false "${pulse_rollback_reserve_sec}"; then
        echo "[awsim-start][ERROR] refusing one-shot Start while any vehicle race arm is not false" >&2
        return 1
    fi
    # Service discovery and DDS polling may consume time.  Recheck the exact
    # pre-pulse authority immediately before the only Start publication.
    observe_vehicle_states
    if [ "${all_vehicle_startable}" != true ] || \
        [ "${all_vehicle_initialization_ready}" != true ] || \
        ! wait_race_arm_all false false "${pulse_rollback_reserve_sec}"; then
        echo "[awsim-start][ERROR] one-shot Start pre-publish authority barrier was lost" >&2
        return 1
    fi
    if ! save_pre_pulse_race_arm_false_watermarks; then
        return 1
    fi
    if ! publish_start; then
        return 1
    fi
    if ! wait_for_one_shot_start_barrier; then
        rollback_start_pulse || true
        return 1
    fi
    if arm_after_authoritative_start; then
        echo "[awsim-start] AWSIM race start confirmed"
        return 0
    fi
    return 1
}

arm_after_authoritative_start() {
    local observed_admin_state=""
    required_failure_reserve_sec="${rollback_reserve_sec}"
    if ! admin_observer_snapshot || \
        [ "${admin_observer_seq}" -le "${admin_observer_watermark_seq}" ]; then
        echo "[awsim-start][ERROR] refusing official Start without a live post-watermark observer sample" >&2
        rollback_official_start || true
        return 1
    fi
    observed_admin_state="${admin_observer_state}"
    if [ "${observed_admin_state}" != "start" ]; then
        echo "[awsim-start][ERROR] refusing official Start outside current admin Start state: ${observed_admin_state:-empty}" >&2
        rollback_official_start || true
        return 1
    fi
    observe_vehicle_states
    if [ "${all_vehicle_start_history_valid}" != true ] || \
        [ "${all_vehicle_initialization_ready}" != true ]; then
        echo "[awsim-start][ERROR] admin Start arrived without valid post-pulse Start history and initialization" >&2
        rollback_official_start || true
        return 1
    fi
    if ! check_all_official_start_services "${rollback_reserve_sec}"; then
        echo "[awsim-start][ERROR] admin Start arrived without all official Start services available" >&2
        rollback_official_start || true
        return 1
    fi
    if ! verify_pre_pulse_race_arm_stayed_false; then
        echo "[awsim-start][ERROR] race arm history was not continuously false before official Start commit" >&2
        rollback_official_start || true
        return 1
    fi
    # The service wait and race-arm poll can outlive a transient AWSIM state.
    # Re-read every authority input immediately before issuing true so a
    # Ready/Start weakening, unavailable service, or early arm cannot commit.
    if ! admin_observer_snapshot || \
        [ "${admin_observer_seq}" -le "${admin_observer_watermark_seq}" ]; then
        echo "[awsim-start][ERROR] official Start pre-commit observer binding was lost" >&2
        rollback_official_start || true
        return 1
    fi
    observed_admin_state="${admin_observer_state}"
    observe_vehicle_states
    if [ "${observed_admin_state}" != "start" ] || \
        [ "${all_vehicle_start_history_valid}" != true ] || \
        [ "${all_vehicle_initialization_ready}" != true ] || \
        ! check_all_official_start_services "${rollback_reserve_sec}" || \
        ! save_race_arm_false_watermarks; then
        echo "[awsim-start][ERROR] official Start pre-commit authority barrier was lost" >&2
        rollback_official_start || true
        return 1
    fi
    if ! deadline_has_budget "${rollback_reserve_sec}"; then
        echo "[awsim-start][ERROR] refusing official Start commit without rollback/reap deadline reserve=${rollback_reserve_sec}s" >&2
        rollback_official_start || true
        return 1
    fi
    official_start_attempted=true
    if ! call_official_start_all true || ! wait_race_arm_all true true; then
        rollback_official_start || true
        return 1
    fi
    if ! admin_observer_snapshot || \
        [ "${admin_observer_seq}" -le "${admin_observer_watermark_seq}" ]; then
        echo "[awsim-start][ERROR] final post-arm admin Start proof was lost" >&2
        rollback_official_start || true
        return 1
    fi
    observed_admin_state="${admin_observer_state}"
    observe_vehicle_states
    if [ "${observed_admin_state}" != "start" ] || \
        [ "${all_vehicle_start_history_valid}" != true ] || \
        [ "${all_vehicle_initialization_ready}" != true ]; then
        echo "[awsim-start][ERROR] final post-arm vehicle/admin authority recheck failed" >&2
        rollback_official_start || true
        return 1
    fi
    official_start_confirmed=true
    required_failure_reserve_sec="${observer_reap_reserve_sec}"
    echo "[awsim-start] authoritative admin Start accepted by all vehicle domains"
}

while deadline_has_budget 0; do
    state="$(read_admin_state)"
    if [ -n "${state}" ] && [ "${state}" != "${last_state}" ]; then
        echo "[awsim-start] admin state: ${state}"
        last_state="${state}"
    fi

    if [ "${start_mode}" = "count" ]; then
        observe_vehicle_states
        if [ "${all_vehicle_started}" = true ] && wait_race_arm_all true; then
            echo "[awsim-start] AWSIM count start confirmed from vehicle states"
            exit 0
        fi
        sleep "${poll_interval_sec}"
        continue
    fi

    case "${state}" in
    start)
        echo "[awsim-start][ERROR] refusing retained admin Start without a helper-owned Ready/WaitStart one-shot transition" >&2
        exit 1
        ;;

    lapcomplete)
        echo "[awsim-start][ERROR] refusing LapComplete before final vehicle Start confirmation" >&2
        rollback_start_pulse || true
        exit 1
        ;;
    ready | waitstart)
        current_run_start_transition_proved=false
        if publish_official_one_shot_start; then
            exit 0
        fi
        if [ "${start_pulse_may_have_been_delivered}" = true ] || \
            [ "${startup_service_preflight_failed}" = true ]; then
            exit 1
        fi
        ;;
    selectmode)
        current_run_start_transition_proved=false
        ;;
    playstart)
        current_run_start_transition_proved=false
        ;;
    "")
        current_run_start_transition_proved=false
        ;;
    finish | finishall | finishedall | terminate | terminated)
        echo "[awsim-start][ERROR] terminal state reached before race start: ${state}" >&2
        exit 1
        ;;
    *)
        current_run_start_transition_proved=false
        echo "[awsim-start][WARN] unknown admin state while waiting: ${state}" >&2
        ;;
    esac

    if [ "${start_pulse_terminal_failed}" = true ]; then
        echo "[awsim-start][ERROR] helper-owned Start publisher failed after delivery became ambiguous; terminal" >&2
        exit 1
    fi

    sleep "${poll_interval_sec}"
done

echo "[awsim-start][ERROR] timed out after ${timeout_sec}s; last admin state=${last_state:-unknown}" >&2
exit 1

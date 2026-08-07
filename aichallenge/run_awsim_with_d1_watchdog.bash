#!/usr/bin/env bash
set -uo pipefail

EXIT_ARTIFACT_CONTRACT=45

repo_root="${REPO_ROOT:-$(pwd)}"
run_id="${RUN_ID:?RUN_ID is required}"
log_dir="${LOG_DIR:?LOG_DIR is required}"
run_host_dir="${RUN_HOST_DIR:?RUN_HOST_DIR is required}"
ready_domains="${AWSIM_READY_DOMAINS:-1}"
stop_timeout_sec="${D1_STALL_TIMEOUT_SEC:-15}"
stop_enter_speed_mps="${D1_STALL_ENTER_SPEED_MPS:-0.05}"
stop_exit_speed_mps="${D1_STALL_EXIT_SPEED_MPS:-0.10}"
velocity_freshness_sec="${D1_VELOCITY_FRESHNESS_SEC:-1.0}"
evidence_failure_sec="${D1_VELOCITY_EVIDENCE_FAILURE_SEC:-15}"
autoware_command_service="${AUTOWARE_COMMAND_SERVICE:-autoware-command}"
autoware_command_mode="${AUTOWARE_COMMAND_MODE:-run}"
autoware_runtime_image="${AUTOWARE_RUNTIME_IMAGE:-aichallenge-2025-dev}"
aic_test_scoped_project="${AIC_TEST_SCOPED_PROJECT:-false}"
gate_deadline_monotonic_ns="${AIC_GATE_DEADLINE_MONOTONIC_NS:-}"

if ! [[ "${autoware_command_service}" =~ ^[a-z0-9][a-z0-9-]*$ ]]; then
    echo "[d1-progress-supervisor][ERROR] invalid AUTOWARE_COMMAND_SERVICE" >&2
    exit 2
fi
if [ "${autoware_command_mode}" != "run" ] && [ "${autoware_command_mode}" != "exec" ]; then
    echo "[d1-progress-supervisor][ERROR] invalid AUTOWARE_COMMAND_MODE" >&2
    exit 2
fi

if ! [[ "${run_id}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
    echo "[d1-progress-supervisor][ERROR] invalid RUN_ID=${run_id}" >&2
    exit 2
fi
if ! [[ "${ready_domains}" =~ ^[1-9][0-9]*(,[1-9][0-9]*)*$ ]]; then
    echo "[d1-progress-supervisor][ERROR] invalid AWSIM_READY_DOMAINS=${ready_domains}" >&2
    exit 2
fi
declare -A seen_ready_domains=()
IFS=',' read -r -a validated_ready_domains <<<"${ready_domains}"
for domain in "${validated_ready_domains[@]}"; do
    if [ -n "${seen_ready_domains[${domain}]:-}" ]; then
        echo "[d1-progress-supervisor][ERROR] duplicate AWSIM_READY_DOMAINS entry=${domain}" >&2
        exit 2
    fi
    seen_ready_domains["${domain}"]=1
done
if [ -n "${gate_deadline_monotonic_ns}" ]; then
    if ! [[ "${gate_deadline_monotonic_ns}" =~ ^[1-9][0-9]*$ ]]; then
        echo "[d1-progress-supervisor][ERROR] invalid AIC_GATE_DEADLINE_MONOTONIC_NS" >&2
        exit 2
    fi
    now_monotonic_ns="$(/usr/bin/python3 -c 'import time; print(time.monotonic_ns())')"
    if [ "${gate_deadline_monotonic_ns}" -le "${now_monotonic_ns}" ]; then
        echo "[d1-progress-supervisor][ERROR] AIC gate deadline already expired" >&2
        exit 2
    fi
fi
for numeric_value in \
    "${stop_timeout_sec}" \
    "${stop_enter_speed_mps}" \
    "${stop_exit_speed_mps}" \
    "${velocity_freshness_sec}" \
    "${evidence_failure_sec}"; do
    if ! [[ "${numeric_value}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        echo "[d1-progress-supervisor][ERROR] invalid numeric watchdog setting" >&2
        exit 2
    fi
done
if [ "${log_dir%/}" != "/output/${run_id}" ] || \
    [ "$(basename "${run_host_dir%/}")" != "${run_id}" ]; then
    echo "[d1-progress-supervisor][ERROR] RUN_ID/output path mismatch" >&2
    exit 2
fi

host_d1_dir="${run_host_dir%/}/d1"
if ! mkdir -p "${host_d1_dir}"; then
    echo "[d1-progress-supervisor][ERROR] cannot initialize run artifact directory" >&2
    exit "${EXIT_ARTIFACT_CONTRACT}"
fi

supervisor_log_path="${host_d1_dir}/start_progress_supervisor.log"
watchdog_verdict_path="${log_dir%/}/d1/start_progress_verdict.json"
host_watchdog_verdict_path="${host_d1_dir}/start_progress_verdict.json"
supervisor_verdict_path="${host_d1_dir}/start_progress_supervisor_verdict.json"
authority_path="${host_d1_dir}/start_progress_authority.json"
fingerprint_path="${log_dir%/}/provenance/artifact-fingerprint.sha256"

if [ -e "${supervisor_log_path}" ] || \
    [ -e "${host_watchdog_verdict_path}" ] || \
    [ -e "${supervisor_verdict_path}" ] || \
    [ -e "${authority_path}" ]; then
    echo "[d1-progress-supervisor][ERROR] terminal/authority artifact already exists for RUN_ID=${run_id}" >&2
    exit "${EXIT_ARTIFACT_CONTRACT}"
fi
if ! : >>"${supervisor_log_path}"; then
    echo "[d1-progress-supervisor][ERROR] cannot initialize supervisor log" >&2
    exit "${EXIT_ARTIFACT_CONTRACT}"
fi
exec > >(tee -a "${supervisor_log_path}") 2>&1

phase="pre_start"
watchdog_pid=""

write_atomic_json() {
    local output_path="$1"
    local payload="$2"
    local temporary_path="${output_path}.tmp.$$"
    if ! printf '%s\n' "${payload}" >"${temporary_path}" || \
        ! mv -f "${temporary_path}" "${output_path}"; then
        echo "[d1-progress-supervisor][ERROR] failed atomic artifact write: ${output_path}" >&2
        return 1
    fi
}

write_supervisor_verdict() {
    local reason="$1"
    local exit_code="$2"
    local detected_utc
    detected_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    write_atomic_json "${supervisor_verdict_path}" \
        "{\"schema\":\"D1_START_PROGRESS_SUPERVISOR_VERDICT_V1\",\"run_id\":\"${run_id}\",\"reason\":\"${reason}\",\"exit_code\":${exit_code},\"passed\":false,\"phase\":\"${phase}\",\"detected_utc\":\"${detected_utc}\"}"
}

write_start_authority() {
    local confirmed_utc
    confirmed_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    write_atomic_json "${authority_path}" \
        "{\"schema\":\"D1_START_AUTHORITY_V1\",\"run_id\":\"${run_id}\",\"scope\":\"D1_ONLY\",\"source\":\"request_awsim_start_exit_0\",\"ready_domains\":\"${ready_domains}\",\"confirmed_utc\":\"${confirmed_utc}\"}"
}

watchdog_verdict_matches_run() {
    local expected_status="$1"
    [ -s "${host_watchdog_verdict_path}" ] || return 1
    python3 - "${host_watchdog_verdict_path}" "${run_id}" "${expected_status}" <<'PY'
import json
import sys

path, expected_run_id, expected_status_text = sys.argv[1:]
expected_status = int(expected_status_text)
with open(path, encoding="utf-8") as stream:
    payload = json.load(stream)
valid = (
    payload.get("schema") == "D1_START_PROGRESS_WATCHDOG_V1"
    and payload.get("run_id") == expected_run_id
    and payload.get("exit_code") == expected_status
    and payload.get("passed") is (expected_status == 0)
)
raise SystemExit(0 if valid else 1)
PY
}

stop_scoped_run() {
    if [ "${aic_test_scoped_project}" = "true" ]; then
        docker compose down --remove-orphans || true
        return
    fi
    local domain
    IFS=',' read -r -a domains <<<"${ready_domains}"
    for domain in "${domains[@]}"; do
        docker compose -p "${domain}" down --remove-orphans || true
    done
    docker compose down --remove-orphans || true
}

verify_runtime_identity() {
    local mode="$1"
    AUTOWARE_RUNTIME_IMAGE="${autoware_runtime_image}" \
        python3 aichallenge/capture_run_fingerprint.py \
            --repo-root "${repo_root}" --output-root "$(dirname "${run_host_dir%/}")" \
            --run-dir "${run_host_dir}" "${mode}"
}

resolve_attested_command_container() {
    AUTOWARE_RUNTIME_IMAGE="${autoware_runtime_image}" \
        python3 aichallenge/capture_run_fingerprint.py \
            --repo-root "${repo_root}" --output-root "$(dirname "${run_host_dir%/}")" \
            --run-dir "${run_host_dir}" \
            --resolve-attested-service "${autoware_command_service}"
}

on_signal() {
    phase="external_shutdown"
    echo "[d1-progress-supervisor] external shutdown requested" >&2
    if [ -n "${watchdog_pid}" ]; then
        # Non-interactive bash starts asynchronous children with SIGINT ignored.
        # TERM is forwarded by docker compose and handled by rclpy shutdown.
        kill -TERM "${watchdog_pid}" 2>/dev/null || true
        abort_deadline=$((SECONDS + 10))
        while kill -0 "${watchdog_pid}" 2>/dev/null && [ "${SECONDS}" -lt "${abort_deadline}" ]; do
            sleep 0.1
        done
        if kill -0 "${watchdog_pid}" 2>/dev/null; then
            echo "[d1-progress-supervisor][ERROR] watchdog abort exceeded 10s bound" >&2
        else
            wait "${watchdog_pid}" 2>/dev/null || true
        fi
    fi
    if ! watchdog_verdict_matches_run 130; then
        write_supervisor_verdict "ABORTED_BY_EXTERNAL_SHUTDOWN" 130 || true
    fi
    verify_runtime_identity --seal-running-attestation || true
    stop_scoped_run
    exit 130
}
trap on_signal INT TERM

cd "${repo_root}"
echo "[d1-progress-supervisor] run_id=${run_id} log_dir=${log_dir} phase=${phase}"
if ! verify_runtime_identity --verify-running-attestation; then
    phase="runtime_identity_changed_before_start"
    write_supervisor_verdict "RUNTIME_IDENTITY_CHANGED_BEFORE_START" 46 || true
    exit 46
fi
command_container_id=""
if [ "${autoware_command_mode}" = "exec" ]; then
    if ! command_container_id="$(resolve_attested_command_container)" || \
        ! [[ "${command_container_id}" =~ ^[0-9a-f]{12,64}$ ]]; then
        phase="attested_command_container_unavailable"
        write_supervisor_verdict "ATTESTED_COMMAND_CONTAINER_UNAVAILABLE" 46 || true
        exit 46
    fi
fi
echo "[d1-progress-supervisor] confirm authoritative Start before monitoring D1"
set +e
if [ "${autoware_command_mode}" = "exec" ]; then
    helper_deadline_args=()
    if [ -n "${gate_deadline_monotonic_ns}" ]; then
        helper_deadline_args=(-e "AIC_GATE_DEADLINE_MONOTONIC_NS=${gate_deadline_monotonic_ns}")
    fi
    docker exec -i -e AWSIM_READY_DOMAINS="${ready_domains}" \
        "${helper_deadline_args[@]}" \
        "${command_container_id}" env ROS_DOMAIN_ID=0 bash /aichallenge/request_awsim_start.bash
else
    helper_deadline_assignment=""
    if [ -n "${gate_deadline_monotonic_ns}" ]; then
        helper_deadline_assignment="AIC_GATE_DEADLINE_MONOTONIC_NS=${gate_deadline_monotonic_ns} "
    fi
    CMD="env ROS_DOMAIN_ID=0 AWSIM_READY_DOMAINS=${ready_domains} ${helper_deadline_assignment}bash /aichallenge/request_awsim_start.bash" \
        docker compose run --rm --no-deps "${autoware_command_service}"
fi
status=$?
set -e
if [ "${status}" -ne 0 ]; then
    phase="authoritative_start_failed"
    echo "[d1-progress-supervisor][ERROR] authoritative Start failed status=${status}" >&2
    write_supervisor_verdict "AUTHORITATIVE_START_FAILED" "${status}" || true
    verify_runtime_identity --seal-running-attestation || true
    stop_scoped_run
    exit "${status}"
fi
if ! write_start_authority; then
    phase="authority_artifact_failed"
    write_supervisor_verdict "START_AUTHORITY_ARTIFACT_FAILED" "${EXIT_ARTIFACT_CONTRACT}" || true
    stop_scoped_run
    exit "${EXIT_ARTIFACT_CONTRACT}"
fi

phase="watchdog"
echo "[d1-progress-supervisor] authoritative Start confirmed; monitor D1 timeout=${stop_timeout_sec}s"
set +e
watchdog_args=(
    env ROS_DOMAIN_ID=1 python3 /aichallenge/d1_start_progress_watchdog.py
    --output "${watchdog_verdict_path}" --run-id "${run_id}"
    --authoritative-start-confirmed --stop-timeout-sec "${stop_timeout_sec}"
    --stop-enter-speed-mps "${stop_enter_speed_mps}" --stop-exit-speed-mps "${stop_exit_speed_mps}"
    --velocity-freshness-sec "${velocity_freshness_sec}" --evidence-failure-sec "${evidence_failure_sec}"
    --fingerprint "${fingerprint_path}"
)
if [ "${autoware_command_mode}" = "exec" ]; then
    docker exec -i -e LOG_DIR="${log_dir}" "${command_container_id}" \
        bash -c 'set -e; source /autoware/install/setup.bash; source /aichallenge/workspace/install/setup.bash; exec "$@"' \
        bash "${watchdog_args[@]}" &
else
    printf -v watchdog_command '%q ' "${watchdog_args[@]}"
    LOG_DIR="${log_dir}" CMD="${watchdog_command}" \
        docker compose run --rm --no-deps "${autoware_command_service}" &
fi
watchdog_pid=$!
wait "${watchdog_pid}"
status=$?
watchdog_pid=""
set -e

if ! watchdog_verdict_matches_run "${status}"; then
    phase="watchdog_verdict_invalid"
    echo "[d1-progress-supervisor][ERROR] watchdog verdict missing or RUN_ID mismatch" >&2
    write_supervisor_verdict "WATCHDOG_VERDICT_MISSING_OR_MISMATCH" "${status}" || true
    if [ "${status}" -eq 0 ]; then
        status="${EXIT_ARTIFACT_CONTRACT}"
    fi
fi

if ! verify_runtime_identity --seal-running-attestation; then
    echo "[d1-progress-supervisor][ERROR] runtime identity continuity failed before shutdown" >&2
    [ "${status}" -ne 0 ] || status=46
fi

if [ "${status}" -ne 0 ]; then
    echo "[d1-progress-supervisor][ERROR] D1 progress watchdog failed status=${status}; preserving verdict before shutdown" >&2
    stop_scoped_run
else
    echo "[d1-progress-supervisor] D1 reached Finish; global run completion remains with AWSIM"
fi
exit "${status}"

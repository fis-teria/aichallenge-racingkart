from __future__ import annotations

import importlib.util
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).with_name("d1_start_progress_watchdog.py")
SPEC = importlib.util.spec_from_file_location("d1_start_progress_watchdog", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
watchdog_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = watchdog_module
SPEC.loader.exec_module(watchdog_module)

D1ProgressWatchdog = watchdog_module.D1ProgressWatchdog
WatchdogConfig = watchdog_module.WatchdogConfig
WatchdogVerdict = watchdog_module.WatchdogVerdict


def active_watchdog() -> D1ProgressWatchdog:
    watchdog = D1ProgressWatchdog(
        WatchdogConfig(), authoritative_start_confirmed=True
    )
    watchdog.observe_race_arm(True, 0.0)
    watchdog.observe_vehicle_state("Start", 0.0)
    return watchdog


def observe_velocity(
    watchdog: D1ProgressWatchdog,
    speed_mps: float,
    stamp_sec: float,
    now_sec: float,
    *,
    clock_sec: float | None = None,
):
    effective_clock_sec = stamp_sec if clock_sec is None else clock_sec
    if effective_clock_sec == effective_clock_sec:
        watchdog.observe_clock(effective_clock_sec, now_sec)
    return watchdog.observe_velocity(speed_mps, stamp_sec, now_sec)


def test_start_wait_does_not_count_stationary_time() -> None:
    watchdog = D1ProgressWatchdog(WatchdogConfig())
    watchdog.observe_race_arm(True, 0.0)
    watchdog.observe_vehicle_state("Ready", 0.0)
    observe_velocity(watchdog, 0.0, 1.0, 0.0)
    assert watchdog.evaluate(30.0) is None
    assert watchdog.stationary_since_sec is None


def test_authoritative_start_allows_ready_to_monitor_stationary_time() -> None:
    watchdog = D1ProgressWatchdog(
        WatchdogConfig(), authoritative_start_confirmed=True
    )
    watchdog.observe_race_arm(True, 0.0)
    watchdog.observe_vehicle_state("Ready", 0.0)
    observe_velocity(watchdog, 0.0, 1.0, 0.0)
    for index in range(1, 150):
        observe_velocity(
            watchdog, 0.0, 1.0 + index * 0.1, index * 0.1
        )
    assert watchdog.evaluate(14.99) is None
    verdict = observe_velocity(watchdog, 0.0, 16.0, 15.0)
    assert verdict is not None
    assert verdict.reason == "D1_CONTINUOUS_STOP_15S"


def test_missing_race_arm_after_authority_is_bounded_failure() -> None:
    watchdog = D1ProgressWatchdog(
        WatchdogConfig(),
        authoritative_start_confirmed=True,
        authority_started_sec=0.0,
    )
    watchdog.observe_vehicle_state("Ready", 0.0)
    assert watchdog.evaluate(4.99) is None
    verdict = watchdog.evaluate(5.0)
    assert verdict is not None
    assert verdict.reason == "RACE_ARM_EVIDENCE_STALE_AFTER_AUTHORITY"
    assert verdict.exit_code == 44


def test_ready_after_start_is_reset_failure() -> None:
    watchdog = active_watchdog()
    assert watchdog.observe_vehicle_state("Ready", 1.0) is None
    assert watchdog.evaluate(1.99) is None
    verdict = watchdog.evaluate(2.0)
    assert verdict is not None
    assert verdict.reason == "D1_RESET_BEFORE_FINISH"


def test_unknown_state_after_authority_is_bounded_failure() -> None:
    watchdog = active_watchdog()
    assert watchdog.observe_vehicle_state("Unexpected", 1.0) is None
    verdict = watchdog.evaluate(2.0)
    assert verdict is not None
    assert verdict.reason == "D1_STATE_INVALID_AFTER_START"


def test_continuous_stop_latches_only_at_15_seconds() -> None:
    watchdog = active_watchdog()
    observe_velocity(watchdog, 0.0, 1.0, 10.0)
    for index in range(1, 150):
        observe_velocity(
            watchdog, 0.0, 1.0 + index * 0.1, 10.0 + index * 0.1
        )
    assert watchdog.evaluate(24.99) is None
    verdict = observe_velocity(watchdog, 0.0, 16.0, 25.0)
    assert verdict is not None
    assert verdict.reason == "D1_CONTINUOUS_STOP_15S"
    assert verdict.exit_code == 42


def test_exit_threshold_resets_continuous_stop_timer() -> None:
    watchdog = active_watchdog()
    observe_velocity(watchdog, 0.0, 1.0, 0.0)
    observe_velocity(watchdog, 0.11, 2.0, 14.9)
    observe_velocity(watchdog, 0.0, 3.0, 15.0)
    assert observe_velocity(watchdog, 0.0, 4.0, 29.9) is None
    verdict = observe_velocity(watchdog, 0.0, 5.0, 30.0)
    assert verdict is not None
    assert verdict.reason == "D1_CONTINUOUS_STOP_15S"


def test_hysteresis_band_preserves_stationary_state() -> None:
    watchdog = active_watchdog()
    observe_velocity(watchdog, 0.04, 1.0, 0.0)
    observe_velocity(watchdog, 0.07, 2.0, 5.0)
    assert watchdog.stationary_since_sec == 0.0


def test_finish_wins_before_stop_timeout() -> None:
    watchdog = active_watchdog()
    observe_velocity(watchdog, 0.0, 1.0, 0.0)
    verdict = watchdog.observe_vehicle_state("Finish", 15.0)
    assert verdict is not None
    assert verdict.reason == "D1_FINISH"
    assert verdict.passed is True
    assert verdict.exit_code == 0


def test_finish_before_confirmed_start_is_not_pass() -> None:
    watchdog = D1ProgressWatchdog(WatchdogConfig())
    verdict = watchdog.observe_vehicle_state("Finish", 0.0)
    assert verdict is not None
    assert verdict.reason == "FINISH_WITHOUT_CONFIRMED_START"
    assert verdict.passed is False


def test_latched_arm_and_start_without_supervisor_authority_cannot_pass() -> None:
    watchdog = D1ProgressWatchdog(WatchdogConfig())
    watchdog.observe_race_arm(True, 0.0)
    watchdog.observe_vehicle_state("Start", 0.0)
    verdict = watchdog.observe_vehicle_state("Finish", 1.0)
    assert verdict is not None
    assert verdict.reason == "FINISH_WITHOUT_CONFIRMED_START"
    assert verdict.passed is False


@pytest.mark.parametrize("state", ["Finish", "Finished", "FinishAll", "FinishedAll"])
def test_finish_aliases_require_and_accept_confirmed_start(state: str) -> None:
    watchdog = active_watchdog()
    verdict = watchdog.observe_vehicle_state(state, 1.0)
    assert verdict is not None
    assert verdict.reason == "D1_FINISH"
    assert verdict.passed is True


@pytest.mark.parametrize("state", ["Terminate", "Terminated"])
def test_terminate_aliases_are_non_pass(state: str) -> None:
    watchdog = active_watchdog()
    verdict = watchdog.observe_vehicle_state(state, 1.0)
    assert verdict is not None
    assert verdict.reason == "D1_TERMINATED_BEFORE_FINISH"
    assert verdict.passed is False


@pytest.mark.parametrize(
    ("speed_mps", "stamp_sec"),
    [(float("nan"), 1.0), (float("inf"), 1.0), (0.0, float("nan"))],
)
def test_invalid_velocity_is_separate_evidence_failure(
    speed_mps: float, stamp_sec: float
) -> None:
    watchdog = active_watchdog()
    assert observe_velocity(watchdog, speed_mps, stamp_sec, 0.0) is None
    verdict = watchdog.evaluate(15.0)
    assert verdict is not None
    assert verdict.reason == "VELOCITY_EVIDENCE_STALE"
    assert verdict.exit_code == 43


def test_non_monotonic_stamp_cannot_prove_continuous_stop() -> None:
    watchdog = active_watchdog()
    observe_velocity(watchdog, 0.0, 2.0, 0.0)
    watchdog.observe_clock(2.5, 0.5)
    watchdog.observe_velocity(0.0, 2.0, 0.5)
    assert watchdog.stationary_since_sec is None
    verdict = watchdog.evaluate(15.5)
    assert verdict is not None
    assert verdict.reason == "VELOCITY_EVIDENCE_STALE"
    assert watchdog.non_monotonic_stamp_count == 1


def test_missing_velocity_after_start_is_not_called_stop() -> None:
    watchdog = active_watchdog()
    watchdog.observe_clock(0.0, 0.0)
    assert watchdog.evaluate(0.0) is None
    assert watchdog.evaluate(14.99) is None
    verdict = watchdog.evaluate(15.0)
    assert verdict is not None
    assert verdict.reason == "VELOCITY_EVIDENCE_STALE"


def test_disarm_and_reset_are_non_pass_failures() -> None:
    disarmed = active_watchdog()
    assert disarmed.observe_race_arm(False, 1.0) is None
    assert disarmed.evaluate(1.99) is None
    verdict = disarmed.evaluate(2.0)
    assert verdict is not None
    assert verdict.reason == "RACE_DISARMED_BEFORE_FINISH"
    assert verdict.passed is False

    reset = active_watchdog()
    assert reset.observe_vehicle_state("Grounded", 1.0) is None
    assert reset.evaluate(1.99) is None
    verdict = reset.evaluate(2.0)
    assert verdict is not None
    assert verdict.reason == "D1_RESET_BEFORE_FINISH"
    assert verdict.passed is False


@pytest.mark.parametrize("event", ["disarm", "reset"])
def test_finish_can_follow_terminal_event_within_ordering_grace(event: str) -> None:
    watchdog = active_watchdog()
    if event == "disarm":
        assert watchdog.observe_race_arm(False, 1.0) is None
    else:
        assert watchdog.observe_vehicle_state("Grounded", 1.0) is None

    verdict = watchdog.observe_vehicle_state("Finish", 1.5)
    assert verdict is not None
    assert verdict.reason == "D1_FINISH"
    assert verdict.passed is True


def test_finish_after_ordering_grace_cannot_override_abort() -> None:
    watchdog = active_watchdog()
    assert watchdog.observe_race_arm(False, 1.0) is None
    abort_verdict = watchdog.evaluate(2.0)
    assert abort_verdict is not None
    assert abort_verdict.reason == "RACE_DISARMED_BEFORE_FINISH"

    finish_verdict = watchdog.observe_vehicle_state("Finish", 2.01)
    assert finish_verdict is abort_verdict
    assert finish_verdict.passed is False


def test_verdict_artifact_is_atomic_and_contains_contract(tmp_path: Path) -> None:
    watchdog = active_watchdog()
    observe_velocity(watchdog, 0.0, 1.0, 0.0)
    verdict = WatchdogVerdict("D1_CONTINUOUS_STOP_15S", 42, False, 15.0)
    output = tmp_path / "d1" / "start_progress_verdict.json"
    watchdog_module._write_verdict(output, "unit-run", watchdog, verdict, None)

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["schema"] == "D1_START_PROGRESS_WATCHDOG_V1"
    assert payload["run_id"] == "unit-run"
    assert payload["reason"] == "D1_CONTINUOUS_STOP_15S"
    assert payload["exit_code"] == 42
    assert payload["passed"] is False
    assert payload["velocity_topic"] == "/vehicle/status/velocity_status"
    assert not output.with_suffix(".json.tmp").exists()


def test_old_source_stamp_is_evidence_failure_not_stop() -> None:
    watchdog = active_watchdog()
    watchdog.observe_clock(10.0, 0.0)
    assert watchdog.observe_velocity(0.0, 1.0, 0.0) is None
    watchdog.observe_clock(25.0, 15.0)
    verdict = watchdog.evaluate(15.0)
    assert verdict is not None
    assert verdict.reason == "VELOCITY_EVIDENCE_STALE"


def test_cli_requires_authoritative_start_confirmation(tmp_path: Path) -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(MODULE_PATH),
            "--output",
            str(tmp_path / "verdict.json"),
            "--run-id",
            "unit-run",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "--authoritative-start-confirmed" in result.stderr


def write_fixture_attestation(run_host_dir: Path) -> None:
    provenance_dir = run_host_dir / "provenance"
    provenance_dir.mkdir(parents=True)
    attestation = {
        "run_id": "fixture-run",
        "expected_image_id": "sha256:fixture-image",
        "services": [
            {
                "service": "autoware-command",
                "container_id": "aaaaaaaaaaaa",
                "actual_image_id": "sha256:fixture-image",
                "compose_project": "fixture",
                "mount_destinations": [],
                "mounts": [],
            },
            {
                "service": "fixture-peer",
                "container_id": "bbbbbbbbbbbb",
                "actual_image_id": "sha256:fixture-image",
                "compose_project": "fixture",
                "mount_destinations": [],
                "mounts": [],
            },
        ],
    }
    (provenance_dir / "runtime-container-attestation.json").write_text(
        json.dumps(attestation), encoding="utf-8"
    )


def run_supervisor_fixture(
    tmp_path: Path,
    *,
    start_status: int = 0,
    monitor_status: int = 0,
    write_monitor_verdict: bool = True,
    monitor_verdict_schema: str = "D1_START_PROGRESS_WATCHDOG_V1",
    command_mode: str = "run",
    bootstrap_status: int = 0,
    ready_domains: str = "1,2",
    gate_deadline_offset_sec: float | None = None,
    runtime_image: str = "aichallenge-2025-eval",
    write_attestation: bool = True,
) -> tuple[subprocess.CompletedProcess[str], list[str]]:
    repo_root = Path(__file__).resolve().parents[1]
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_docker = fake_bin / "docker"
    fake_docker.write_text(
        """#!/usr/bin/env bash
set -u
case "$*" in
"compose ps -q autoware-command") printf 'aaaaaaaaaaaa\\n'; exit 0 ;;
"compose ps -q fixture-peer") printf 'bbbbbbbbbbbb\\n'; exit 0 ;;
inspect\\ aaaaaaaaaaaa*)
    printf '[{"Image":"sha256:fixture-image","Mounts":[],"Config":{"Labels":{"com.docker.compose.service":"autoware-command","com.docker.compose.project":"fixture"}}}]\\n'
    exit 0
    ;;
inspect\\ bbbbbbbbbbbb*)
    printf '[{"Image":"sha256:fixture-image","Mounts":[],"Config":{"Labels":{"com.docker.compose.service":"fixture-peer","com.docker.compose.project":"fixture"}}}]\\n'
    exit 0
    ;;
esac
printf '%s|%s\\n' "$*" "${CMD:-}" >>"${FAKE_DOCKER_LOG}"
if [[ "$*" == "exec "*d1_start_progress_watchdog.py* ]]; then
    printf '%s\\0' "$@" >"${FAKE_EXEC_ARGV_LOG}"
    if [ "${FAKE_BOOTSTRAP_STATUS}" -ne 0 ]; then
        exit "${FAKE_BOOTSTRAP_STATUS}"
    fi
fi
printf 'fake-docker-stdout\\n'
printf 'fake-docker-stderr\\n' >&2
case "$*|${CMD:-}" in
*request_awsim_start.bash*) exit "${FAKE_START_STATUS}" ;;
*d1_start_progress_watchdog.py*)
    if [ "${FAKE_WRITE_MONITOR_VERDICT}" = 1 ]; then
        mkdir -p "$(dirname "${FAKE_MONITOR_VERDICT}")"
        printf '{"schema": "%s", "run_id": "fixture-run", "exit_code": %s, "passed": %s}\\n' \
            "${FAKE_MONITOR_SCHEMA}" "${FAKE_MONITOR_STATUS}" "${FAKE_MONITOR_PASSED}" >"${FAKE_MONITOR_VERDICT}"
    fi
    exit "${FAKE_MONITOR_STATUS}"
    ;;
*) exit 0 ;;
esac
""",
        encoding="utf-8",
    )
    fake_docker.chmod(0o755)
    log_path = tmp_path / "docker.log"
    run_host_dir = tmp_path / "output" / "fixture-run"
    if write_attestation:
        write_fixture_attestation(run_host_dir)
    else:
        (run_host_dir / "provenance").mkdir(parents=True)
    environment = dict(os.environ)
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "REPO_ROOT": str(repo_root),
            "RUN_ID": "fixture-run",
            "LOG_DIR": "/output/fixture-run",
            "RUN_HOST_DIR": str(run_host_dir),
            "AWSIM_READY_DOMAINS": ready_domains,
            "AUTOWARE_COMMAND_MODE": command_mode,
            "AUTOWARE_RUNTIME_IMAGE": runtime_image,
            "FAKE_DOCKER_LOG": str(log_path),
            "FAKE_EXEC_ARGV_LOG": str(tmp_path / "exec-argv.bin"),
            "FAKE_BOOTSTRAP_STATUS": str(bootstrap_status),
            "FAKE_START_STATUS": str(start_status),
            "FAKE_MONITOR_STATUS": str(monitor_status),
            "FAKE_WRITE_MONITOR_VERDICT": "1" if write_monitor_verdict else "0",
            "FAKE_MONITOR_VERDICT": str(
                run_host_dir / "d1" / "start_progress_verdict.json"
            ),
            "FAKE_MONITOR_PASSED": "true" if monitor_status == 0 else "false",
            "FAKE_MONITOR_SCHEMA": monitor_verdict_schema,
        }
    )
    if gate_deadline_offset_sec is not None:
        environment["AIC_GATE_DEADLINE_MONOTONIC_NS"] = str(
            time.monotonic_ns() + int(gate_deadline_offset_sec * 1_000_000_000)
        )
    result = subprocess.run(
        ["bash", "aichallenge/run_awsim_with_d1_watchdog.bash"],
        cwd=repo_root,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )
    calls = log_path.read_text(encoding="utf-8").splitlines() if log_path.exists() else []
    return result, calls


def test_dev_runtime_does_not_require_packaged_eval_attestation(
    tmp_path: Path,
) -> None:
    result, calls = run_supervisor_fixture(
        tmp_path,
        runtime_image="aichallenge-2025-dev",
        write_attestation=False,
    )
    assert result.returncode == 0
    assert any("request_awsim_start.bash" in call for call in calls)


@pytest.mark.parametrize("command_mode", ["run", "exec"])
def test_supervisor_passes_shared_absolute_deadline_to_start_helper(
    tmp_path: Path, command_mode: str
) -> None:
    result, calls = run_supervisor_fixture(
        tmp_path, command_mode=command_mode, gate_deadline_offset_sec=60
    )
    assert result.returncode == 0
    helper_call = next(call for call in calls if "request_awsim_start.bash" in call)
    assert "AIC_GATE_DEADLINE_MONOTONIC_NS=" in helper_call


def test_supervisor_rejects_duplicate_ready_domains_before_docker(tmp_path: Path) -> None:
    result, calls = run_supervisor_fixture(tmp_path, ready_domains="1,1")
    assert result.returncode == 2
    assert "duplicate AWSIM_READY_DOMAINS entry=1" in result.stderr
    assert calls == []


def test_supervisor_preserves_watchdog_failure_and_stops_scoped_run(
    tmp_path: Path,
) -> None:
    result, calls = run_supervisor_fixture(tmp_path, monitor_status=42)
    assert result.returncode == 42
    assert "request_awsim_start.bash" in calls[0]
    assert "d1_start_progress_watchdog.py" in calls[1]
    assert any("compose -p 1 down --remove-orphans" in call for call in calls)
    assert any("compose -p 2 down --remove-orphans" in call for call in calls)
    assert calls[-1].startswith("compose down --remove-orphans|")
    supervisor_log = (
        tmp_path / "output" / "fixture-run" / "d1" / "start_progress_supervisor.log"
    )
    assert supervisor_log.is_file()
    supervisor_log_text = supervisor_log.read_text(encoding="utf-8")
    assert "run_id=fixture-run" in supervisor_log_text
    assert "fake-docker-stdout" in supervisor_log_text
    assert "fake-docker-stderr" in supervisor_log_text
    assert "--authoritative-start-confirmed" in calls[1]


def test_supervisor_finish_does_not_stop_other_vehicles(tmp_path: Path) -> None:
    result, calls = run_supervisor_fixture(tmp_path, monitor_status=0)
    assert result.returncode == 0
    assert "D1 reached Finish" in result.stdout
    assert not any(" down --remove-orphans" in call for call in calls)
    authority = json.loads(
        (
            tmp_path
            / "output"
            / "fixture-run"
            / "d1"
            / "start_progress_authority.json"
        ).read_text(encoding="utf-8")
    )
    assert authority["run_id"] == "fixture-run"
    assert authority["scope"] == "D1_ONLY"


def test_supervisor_preserves_start_failure_without_starting_watchdog(
    tmp_path: Path,
) -> None:
    result, calls = run_supervisor_fixture(tmp_path, start_status=7)
    assert result.returncode == 7
    assert "request_awsim_start.bash" in calls[0]
    assert not any("d1_start_progress_watchdog.py" in call for call in calls)
    assert calls[-1].startswith("compose down --remove-orphans|")
    supervisor_verdict = json.loads(
        (
            tmp_path
            / "output"
            / "fixture-run"
            / "d1"
            / "start_progress_supervisor_verdict.json"
        ).read_text(encoding="utf-8")
    )
    assert supervisor_verdict["reason"] == "AUTHORITATIVE_START_FAILED"
    assert supervisor_verdict["run_id"] == "fixture-run"


def test_supervisor_missing_watchdog_verdict_fails_closed(tmp_path: Path) -> None:
    result, calls = run_supervisor_fixture(
        tmp_path, monitor_status=0, write_monitor_verdict=False
    )
    assert result.returncode == 45
    assert any("compose down --remove-orphans" in call for call in calls)
    supervisor_verdict = json.loads(
        (
            tmp_path
            / "output"
            / "fixture-run"
            / "d1"
            / "start_progress_supervisor_verdict.json"
        ).read_text(encoding="utf-8")
    )
    assert supervisor_verdict["reason"] == "WATCHDOG_VERDICT_MISSING_OR_MISMATCH"


def test_supervisor_rejects_wrong_watchdog_verdict_schema(tmp_path: Path) -> None:
    result, calls = run_supervisor_fixture(
        tmp_path,
        monitor_status=0,
        monitor_verdict_schema="UNTRUSTED_SCHEMA",
    )
    assert result.returncode == 45
    assert any("compose down --remove-orphans" in call for call in calls)
    supervisor_verdict = json.loads(
        (
            tmp_path
            / "output"
            / "fixture-run"
            / "d1"
            / "start_progress_supervisor_verdict.json"
        ).read_text(encoding="utf-8")
    )
    assert supervisor_verdict["reason"] == "WATCHDOG_VERDICT_MISSING_OR_MISMATCH"


def test_exec_mode_bootstraps_ros_environment_before_unchanged_watchdog_argv(
    tmp_path: Path,
) -> None:
    result, _calls = run_supervisor_fixture(tmp_path, command_mode="exec")

    assert result.returncode == 0
    argv = (tmp_path / "exec-argv.bin").read_bytes().split(b"\0")[:-1]
    decoded_argv = [item.decode("utf-8") for item in argv]
    assert decoded_argv[:8] == [
        "exec",
        "-i",
        "-e",
        "LOG_DIR=/output/fixture-run",
        "aaaaaaaaaaaa",
        "bash",
        "-c",
        'set -e; source /autoware/install/setup.bash; source /aichallenge/workspace/install/setup.bash; exec "$@"',
    ]
    assert decoded_argv[8] == "bash"
    assert decoded_argv[9:] == [
        "env",
        "ROS_DOMAIN_ID=1",
        "python3",
        "/aichallenge/d1_start_progress_watchdog.py",
        "--output",
        "/output/fixture-run/d1/start_progress_verdict.json",
        "--run-id",
        "fixture-run",
        "--authoritative-start-confirmed",
        "--stop-timeout-sec",
        "15",
        "--stop-enter-speed-mps",
        "0.05",
        "--stop-exit-speed-mps",
        "0.10",
        "--velocity-freshness-sec",
        "1.0",
        "--evidence-failure-sec",
        "15",
        "--fingerprint",
        "/output/fixture-run/provenance/artifact-fingerprint.sha256",
    ]


def test_exec_mode_bootstrap_failure_fails_closed_and_stops_scoped_run(
    tmp_path: Path,
) -> None:
    result, calls = run_supervisor_fixture(
        tmp_path, command_mode="exec", bootstrap_status=47
    )

    assert result.returncode == 47
    assert any("compose -p 1 down --remove-orphans" in call for call in calls)
    assert any("compose -p 2 down --remove-orphans" in call for call in calls)
    supervisor_verdict = json.loads(
        (
            tmp_path
            / "output"
            / "fixture-run"
            / "d1"
            / "start_progress_supervisor_verdict.json"
        ).read_text(encoding="utf-8")
    )
    assert supervisor_verdict["reason"] == "WATCHDOG_VERDICT_MISSING_OR_MISMATCH"


def test_supervisor_rejects_run_id_path_mismatch_before_start(
    tmp_path: Path,
) -> None:
    repo_root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        ["bash", "aichallenge/run_awsim_with_d1_watchdog.bash"],
        cwd=repo_root,
        env={
            **os.environ,
            "RUN_ID": "fixture-run",
            "LOG_DIR": "/output/other-run",
            "RUN_HOST_DIR": str(tmp_path / "output" / "fixture-run"),
        },
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "RUN_ID/output path mismatch" in result.stderr


def test_supervisor_signal_waits_for_abort_verdict_before_cleanup(
    tmp_path: Path,
) -> None:
    repo_root = Path(__file__).resolve().parents[1]
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_docker = fake_bin / "docker"
    fake_docker.write_text(
"""#!/usr/bin/env bash
set -u
case "$*" in
"compose ps -q autoware-command") printf 'aaaaaaaaaaaa\\n'; exit 0 ;;
"compose ps -q fixture-peer") printf 'bbbbbbbbbbbb\\n'; exit 0 ;;
inspect\\ aaaaaaaaaaaa*)
    printf '[{"Image":"sha256:fixture-image","Mounts":[],"Config":{"Labels":{"com.docker.compose.service":"autoware-command","com.docker.compose.project":"fixture"}}}]\\n'
    exit 0
    ;;
inspect\\ bbbbbbbbbbbb*)
    printf '[{"Image":"sha256:fixture-image","Mounts":[],"Config":{"Labels":{"com.docker.compose.service":"fixture-peer","com.docker.compose.project":"fixture"}}}]\\n'
    exit 0
    ;;
esac
printf '%s|%s\\n' "$*" "${CMD:-}" >>"${FAKE_DOCKER_LOG}"
case "${CMD:-}" in
*request_awsim_start.bash*) exit 0 ;;
*d1_start_progress_watchdog.py*)
    trap 'mkdir -p "$(dirname "${FAKE_ABORT_VERDICT}")"; printf "{\\"schema\\": \\"D1_START_PROGRESS_WATCHDOG_V1\\", \\"run_id\\": \\"fixture-run\\", \\"reason\\": \\"ABORTED_BY_EXTERNAL_SHUTDOWN\\", \\"exit_code\\": 130, \\"passed\\": false}\\n" >"${FAKE_ABORT_VERDICT}"; printf "verdict-written|\\n" >>"${FAKE_DOCKER_LOG}"; exit 130' INT TERM
    while true; do read -r -t 0.1 _unused || true; done
    ;;
*) exit 0 ;;
esac
""",
        encoding="utf-8",
    )
    fake_docker.chmod(0o755)
    log_path = tmp_path / "docker.log"
    run_host_dir = tmp_path / "output" / "fixture-run"
    write_fixture_attestation(run_host_dir)
    verdict_path = run_host_dir / "d1" / "start_progress_verdict.json"
    environment = dict(os.environ)
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "REPO_ROOT": str(repo_root),
            "RUN_ID": "fixture-run",
            "LOG_DIR": "/output/fixture-run",
            "RUN_HOST_DIR": str(run_host_dir),
            "AWSIM_READY_DOMAINS": "1",
            "FAKE_DOCKER_LOG": str(log_path),
            "FAKE_ABORT_VERDICT": str(verdict_path),
        }
    )
    process = subprocess.Popen(
        ["bash", "aichallenge/run_awsim_with_d1_watchdog.bash"],
        cwd=repo_root,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if log_path.is_file() and "d1_start_progress_watchdog.py" in log_path.read_text(
            encoding="utf-8"
        ):
            break
        time.sleep(0.02)
    else:
        process.kill()
        raise AssertionError("watchdog fixture did not start")

    process.send_signal(signal.SIGINT)
    stdout, stderr = process.communicate(timeout=10)

    assert process.returncode == 130
    assert verdict_path.is_file()
    assert "ABORTED_BY_EXTERNAL_SHUTDOWN" in verdict_path.read_text(
        encoding="utf-8"
    )
    assert "abort verdict was not finalized" not in stderr
    calls = log_path.read_text(encoding="utf-8").splitlines()
    verdict_index = calls.index("verdict-written|")
    cleanup_index = next(
        index
        for index, call in enumerate(calls)
        if "compose down --remove-orphans" in call
    )
    assert verdict_index < cleanup_index
    assert "external shutdown requested" in stdout
    assert stdout


def test_start_helper_bounds_one_shot_publisher_and_requires_state_proof() -> None:
    source = Path(__file__).with_name("request_awsim_start.bash").read_text(
        encoding="utf-8"
    )
    waiter_source = Path(__file__).with_name(
        "wait_for_typed_service.py"
    ).read_text(encoding="utf-8")

    assert "setsid timeout --kill-after=2" in source
    assert "ros2 topic pub" in source
    assert "--rate" not in source
    assert "-1" in source
    assert "--wait-matching-subscriptions 1" in source
    assert "publisher_pid=$!" in source
    assert "pre_publish_state=\"$(read_admin_state)\"" in source
    assert "save_pre_pulse_race_arm_false_watermarks" in source
    assert "verify_pre_pulse_race_arm_stayed_false" in source
    assert "race_arm_observer_pre_pulse_watermark_seqs" in source
    assert "final post-arm vehicle/admin authority recheck failed" in source
    assert "wait_for_typed_service.py" in source
    assert "ros2 service type" not in source
    assert "from std_srvs.srv import SetBool" in waiter_source
    assert "create_client(SetBool" in waiter_source
    assert "wait_for_service(timeout_sec=" in waiter_source
    assert "call_async" not in waiter_source


def run_start_helper_with_fake_ros(
    tmp_path: Path,
    initial_admin_state: str,
    *,
    start_behavior: str = "transition",
    service_behavior: str = "accept",
    race_arm_state: str = "true",
    initial_race_arm_state: str = "false",
    startup_race_arm_behavior: str = "stable",
    domain_2_initial_race_arm_state: str = "",
    vehicle_state: str = "Ready",
    vehicle_state_after_start: str = "Start",
    domain_2_vehicle_state_after_start: str = "",
    initialization_ready_after_start: str = "true",
    admin_waitstart_reads_before_start: int = 2,
    service_missing_after_start: bool = False,
    ready_domains: str = "1",
    signal_during_partial_start: bool = False,
    start_mode: str = "sync",
    gate_deadline_offset_sec: float | None = None,
    use_production_timeouts: bool = False,
) -> tuple[subprocess.CompletedProcess[str], list[str]]:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_timeout = fake_bin / "timeout"
    fake_timeout.write_text(
        """#!/usr/bin/env bash
set -eu
export PATH="${FAKE_BIN}:${PATH}"
exec /usr/bin/timeout "$@"
""",
        encoding="utf-8",
    )
    fake_timeout.chmod(0o755)
    state_path = tmp_path / "admin-state"
    state_path.write_text(initial_admin_state, encoding="utf-8")
    start_publish_count_path = tmp_path / "start-publish-count"
    start_publish_count_path.write_text("0", encoding="utf-8")
    admin_state_read_count_path = tmp_path / "admin-state-read-count"
    admin_state_read_count_path.write_text("0", encoding="utf-8")
    observer_read_count_path = tmp_path / "observer-read-count"
    observer_read_count_path.write_text("0", encoding="utf-8")
    observer_pid_path = tmp_path / "observer.pid"
    race_observer_pid_dir = tmp_path / "race-observer-pids"
    race_observer_pid_dir.mkdir()
    vehicle_observer_pid_dir = tmp_path / "vehicle-observer-pids"
    vehicle_observer_pid_dir.mkdir()
    observer_dirs_path = tmp_path / "observer-dirs"
    observer_dirs_path.write_text("", encoding="utf-8")
    race_arm_state_path = tmp_path / "race-arm-state"
    race_arm_state_path.write_text(initial_race_arm_state, encoding="utf-8")
    race_arm_event_generation_path = tmp_path / "race-arm-event-generation"
    race_arm_event_generation_path.write_text("0", encoding="utf-8")
    calls_path = tmp_path / "calls.log"
    publisher_pid_path = tmp_path / "publisher.pid"
    publisher_terminated_path = tmp_path / "publisher.terminated"
    service_pid_path = tmp_path / "service.pid"
    service_child_pid_path = tmp_path / "service-child.pid"
    service_terminated_path = tmp_path / "service.terminated"
    ready_during_preflight_path = tmp_path / "ready-during-preflight"
    reset_occurred_path = tmp_path / "reset-occurred"
    post_arm_initialization_false_path = tmp_path / "post-arm-initialization-false"
    fake_ros2 = fake_bin / "ros2"
    fake_ros2.write_text(
        """#!/usr/bin/env bash
set -u
publish_race_arm_event() {
    printf '%s' "$1" >"${FAKE_RACE_ARM_STATE_PATH}"
    generation=$(( $(cat "${FAKE_RACE_ARM_EVENT_GENERATION_PATH}") + 1 ))
    printf '%s' "${generation}" >"${FAKE_RACE_ARM_EVENT_GENERATION_PATH}"
}
printf '%s|%s\\n' "${ROS_DOMAIN_ID:-0}" "$*" >>"${FAKE_ROS_CALLS}"
case "$*" in
*"topic pub"*"/admin/awsim/reset"*)
    printf 'grounded' >"${FAKE_ADMIN_STATE}"
    : >"${FAKE_RESET_OCCURRED}"
    exit 0
    ;;
*"topic echo"*"/admin/awsim/state"*)
    if { [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_then_start" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_then_start_regress" ]; } && \
        [ "$(cat "${FAKE_START_PUBLISH_COUNT}")" -ge 1 ]; then
        admin_read_count=$(( $(cat "${FAKE_ADMIN_STATE_READ_COUNT}") + 1 ))
        printf '%s' "${admin_read_count}" >"${FAKE_ADMIN_STATE_READ_COUNT}"
        if [ "${admin_read_count}" -ge "${FAKE_ADMIN_WAITSTART_READS_BEFORE_START}" ]; then
            printf 'start' >"${FAKE_ADMIN_STATE}"
        fi
    fi
    if [ "${FAKE_START_BEHAVIOR}" = "persistent_post_watermark_start" ] && \
        [ "$(cat "${FAKE_START_PUBLISH_COUNT}")" -ge 1 ]; then
        printf 'data: waitstart\\n'
    else
        printf 'data: %s\\n' "$(cat "${FAKE_ADMIN_STATE}")"
    fi
    ;;
*"topic echo"*"/awsim/state"*)
    if [ -f "${FAKE_READY_DURING_PREFLIGHT}" ] && \
        [ "$(cat "${FAKE_ADMIN_STATE}")" != "start" ]; then
        printf 'data: Ready\n'
    elif { [ "${FAKE_START_BEHAVIOR}" = "two_stage" ] && \
        [ "$(cat "${FAKE_START_PUBLISH_COUNT}")" -eq 1 ]; } || \
        { [ "${FAKE_START_BEHAVIOR}" = "two_stage_lapcomplete_pre_final" ] && \
        [ "$(cat "${FAKE_START_PUBLISH_COUNT}")" -eq 1 ]; } || \
        { [ "${FAKE_START_BEHAVIOR}" = "two_stage_no_final_start" ] && \
        [ "$(cat "${FAKE_START_PUBLISH_COUNT}")" -ge 1 ]; }; then
        printf 'data: Ready\n'
    elif { [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_playstart" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_then_start" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_then_start_regress" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_pub_fail" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "persistent_post_watermark_start" ]; } && \
        [ "$(cat "${FAKE_START_PUBLISH_COUNT}")" -ge 1 ] && \
        [ -n "${FAKE_VEHICLE_STATE_AFTER_START}" ]; then
        if [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_then_start_regress" ] && \
            [ "$(cat "${FAKE_ADMIN_STATE}")" = "start" ] && \
            [ "${ROS_DOMAIN_ID:-0}" = "2" ]; then
            printf 'data: Grounded\\n'
        elif [ "${ROS_DOMAIN_ID:-0}" = "2" ] && \
            [ -n "${FAKE_DOMAIN_2_VEHICLE_STATE_AFTER_START}" ]; then
            printf 'data: %s\\n' "${FAKE_DOMAIN_2_VEHICLE_STATE_AFTER_START}"
        else
            printf 'data: %s\\n' "${FAKE_VEHICLE_STATE_AFTER_START}"
        fi
    elif [ -n "${FAKE_VEHICLE_STATE_AFTER_START}" ] && \
        [ "$(cat "${FAKE_ADMIN_STATE}")" = "start" ]; then
        printf 'data: %s\\n' "${FAKE_VEHICLE_STATE_AFTER_START}"
    else
        printf 'data: %s\\n' "${FAKE_VEHICLE_STATE}"
    fi
    ;;
*"topic echo"*"/autostart/initialization_ready"*)
    if [ "${FAKE_START_BEHAVIOR}" = "two_stage" ] && \
        [ "$(cat "${FAKE_START_PUBLISH_COUNT}")" -eq 1 ]; then
        printf 'data: true\\n'
    elif { { [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_playstart" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_then_start" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_then_start_regress" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_pub_fail" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "persistent_post_watermark_start" ]; } && \
        [ "$(cat "${FAKE_START_PUBLISH_COUNT}")" -ge 1 ]; } || \
        [ "$(cat "${FAKE_ADMIN_STATE}")" = "start" ]; then
        printf 'data: %s\\n' "${FAKE_INITIALIZATION_READY_AFTER_START}"
    else
        printf 'data: true\\n'
    fi
    ;;
*"topic echo"*"/overtake/race_armed"*)
    printf 'data: %s\\n' "$(cat "${FAKE_RACE_ARM_STATE_PATH}")"
    ;;
*"service type"*)
    if [ "${FAKE_SERVICE_BEHAVIOR}" != "missing" ] && ! {
        [ "${FAKE_SERVICE_MISSING_AFTER_START}" = "true" ] && \
            [ "$(cat "${FAKE_ADMIN_STATE}")" = "start" ]
    }; then
        printf 'std_srvs/srv/SetBool\\n'
    fi
    ;;
*"service call"*)
    if [[ "$*" == *"{data: true}"* ]] && \
        [ "${FAKE_SERVICE_BEHAVIOR}" = "block_partial" ] && \
        [ "${ROS_DOMAIN_ID:-0}" = "2" ]; then
        printf '%s\\n' "${BASHPID}" >"${FAKE_SERVICE_PID}"
        trap 'printf terminated >"${FAKE_SERVICE_TERMINATED}"; exit 143' TERM INT
        sleep 30
    fi
    if [[ "$*" == *"{data: true}"* ]] && \
        [ "${FAKE_SERVICE_BEHAVIOR}" = "ignore_term_partial" ] && \
        [ "${ROS_DOMAIN_ID:-0}" = "2" ]; then
        printf '%s\\n' "${BASHPID}" >"${FAKE_SERVICE_PID}"
        trap '' TERM INT
        (
            trap '' TERM INT
            printf '%s\\n' "${BASHPID}" >"${FAKE_SERVICE_CHILD_PID}"
            sleep 30
        ) &
        wait
    fi
    if [[ "$*" == *"{data: true}"* ]] && \
        [ "${FAKE_SERVICE_BEHAVIOR}" = "delayed_true_old_false" ]; then
        nohup bash -c 'sleep 0.2; printf true >"$1"; generation=$(( $(cat "$2") + 1 )); printf "%s" "${generation}" >"$2"' \
            _ "${FAKE_RACE_ARM_STATE_PATH}" "${FAKE_RACE_ARM_EVENT_GENERATION_PATH}" \
            >/dev/null 2>&1 &
        printf 'response: std_srvs.srv.SetBool_Response(success=False, message="delayed reject")\\n'
    elif [[ "$*" == *"{data: true}"* ]] && \
        [ "${FAKE_SERVICE_BEHAVIOR}" = "reject" ]; then
        printf 'response: std_srvs.srv.SetBool_Response(success=False, message="reject")\\n'
    else
        if [[ "$*" == *"{data: true}"* ]]; then
            publish_race_arm_event "${FAKE_RACE_ARM_AFTER_START}"
            if [ "${FAKE_SERVICE_BEHAVIOR}" = "post_arm_init_false" ]; then
                : >"${FAKE_POST_ARM_INITIALIZATION_FALSE}"
            fi
        elif [[ "$*" == *"{data: false}"* ]]; then
            if [ "${FAKE_SERVICE_BEHAVIOR}" != "delayed_true_old_false" ]; then
                publish_race_arm_event false
            fi
        fi
        printf 'response: std_srvs.srv.SetBool_Response(success=True, message="ok")\\n'
    fi
    ;;
*"topic pub"*"/admin/awsim/start"*)
    publish_count=$(( $(cat "${FAKE_START_PUBLISH_COUNT}") + 1 ))
    printf '%s' "${publish_count}" >"${FAKE_START_PUBLISH_COUNT}"
    if [[ " $* " == *" -1 "* ]]; then
        if [ "${FAKE_START_BEHAVIOR}" = "two_stage_lapcomplete_pre_final" ] && \
            [ "${publish_count}" -eq 1 ]; then
            printf 'lapcomplete' >"${FAKE_ADMIN_STATE}"
            exit 0
        fi
        if [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart" ] || \
            [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_then_start" ] || \
            [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_then_start_regress" ] || \
            [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_pub_fail" ]; then
            printf 'waitstart' >"${FAKE_ADMIN_STATE}"
            if [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_pub_fail" ]; then
                exit 1
            fi
        fi
        if [ "${FAKE_START_BEHAVIOR}" = "transition" ] || \
            [ "${FAKE_START_BEHAVIOR}" = "start_then_ready_after_publish" ] || \
            [ "${FAKE_START_BEHAVIOR}" = "start_then_grounded_after_publish" ] || \
            [ "${FAKE_START_BEHAVIOR}" = "start_then_finish_after_publish" ] || \
            [ "${FAKE_START_BEHAVIOR}" = "two_stage" ] || \
            [ "${FAKE_START_BEHAVIOR}" = "two_stage_lapcomplete_pre_final" ] || \
            [ "${FAKE_START_BEHAVIOR}" = "two_stage_no_final_start" ] || \
            [ "${FAKE_START_BEHAVIOR}" = "transition_then_unknown" ]; then
            printf 'start' >"${FAKE_ADMIN_STATE}"
        elif [ "${FAKE_START_BEHAVIOR}" = "delayed_transition" ]; then
            sleep 0.2
            printf 'start' >"${FAKE_ADMIN_STATE}"
        fi
        exit 0
    fi
    printf '%s\\n' "$$" >"${FAKE_PUBLISHER_PID}"
    trap 'printf terminated >"${FAKE_PUBLISHER_TERMINATED}"; if [ "${FAKE_START_BEHAVIOR}" = "transition_then_unknown" ]; then printf unknown >"${FAKE_ADMIN_STATE}"; nohup bash -c '"'"'sleep 0.2; printf start >"$1"'"'"' _ "${FAKE_ADMIN_STATE}" >/dev/null 2>&1 & elif [ "${FAKE_START_BEHAVIOR}" = "two_stage_lapcomplete_pre_final" ]; then printf lapcomplete >"${FAKE_ADMIN_STATE}"; fi; exit 143' TERM INT
    if [ "${FAKE_START_BEHAVIOR}" = "transition" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "two_stage" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "two_stage_lapcomplete_pre_final" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "two_stage_no_final_start" ] || \
        [ "${FAKE_START_BEHAVIOR}" = "transition_then_unknown" ]; then
        printf 'start' >"${FAKE_ADMIN_STATE}"
    elif [ "${FAKE_START_BEHAVIOR}" = "delayed_transition" ]; then
        sleep 0.2
        printf 'start' >"${FAKE_ADMIN_STATE}"
    fi
    sleep 30
    ;;
*)
    exit 2
    ;;
esac
""",
        encoding="utf-8",
    )
    fake_ros2.chmod(0o755)
    fake_python3 = fake_bin / "python3"
    fake_python3.write_text(
        """#!/usr/bin/env bash
set -u
if [[ "$1" == *"vehicle_readiness_observer.py" ]]; then
    shift
    exec /usr/bin/python3 "${FAKE_VEHICLE_OBSERVER_SCRIPT}" "$@"
fi
if [[ "$1" == *"race_arm_observer.py" ]]; then
    shift
    exec /usr/bin/python3 "${FAKE_RACE_OBSERVER_SCRIPT}" "$@"
fi
if [[ "$*" == *"admin_state_observer.py"* ]]; then
    token=""; jsonl=""; ready=""; expected_pid=""; snapshot=0
    while [ "$#" -gt 0 ]; do
        case "$1" in
        --token) token="$2"; shift 2 ;;
        --jsonl) jsonl="$2"; shift 2 ;;
        --ready) ready="$2"; shift 2 ;;
        --expected-pid) expected_pid="$2"; shift 2 ;;
        --snapshot) snapshot=1; shift ;;
        *) shift ;;
        esac
    done
    if [ "${snapshot}" = 0 ]; then
        printf '%s' "$$" >"${FAKE_OBSERVER_PID}"
        dirname "${ready}" >>"${FAKE_OBSERVER_DIRS}"
        if [ "${FAKE_START_BEHAVIOR}" = "observer_startup_exit" ]; then exit 1; fi
        started_monotonic_ns="$(/usr/bin/python3 -c 'import time; print(time.monotonic_ns())')"
        printf '{"pid":%s,"started_monotonic_ns":%s,"status":"ready","token":"%s"}\\n' "$$" "${started_monotonic_ns}" "${token}" >"${ready}"
        : >"${jsonl}"
        trap 'printf "{\\"pid\\":%s,\\"started_monotonic_ns\\":%s,\\"status\\":\\"stopped\\",\\"stopped_monotonic_ns\\":%s,\\"token\\":\\"%s\\"}\\n" "$$" "${started_monotonic_ns}" "$(/usr/bin/python3 -c "import time; print(time.monotonic_ns())")" "${token}" >"${ready}"; exit 0' TERM INT
        while true; do sleep 0.05; done
    fi
    if [ ! -f "${ready}" ] || [ "$(sed -n 's/.*\\"token\\":\\"\\([^\\"]*\\)\\".*/\\1/p' "${ready}")" != "${token}" ]; then exit 1; fi
    if [ -n "${expected_pid}" ] && ! grep -q "\\"pid\\":${expected_pid}" "${ready}"; then exit 1; fi
    state="$(cat "${FAKE_ADMIN_STATE}")"
    if [ "${FAKE_START_BEHAVIOR}" = "observer_exit_after_pulse" ] && [ "$(cat "${FAKE_START_PUBLISH_COUNT}")" -ge 1 ]; then
        kill -TERM "$(cat "${FAKE_OBSERVER_PID}")" 2>/dev/null || true
        exit 1
    fi
    # The persistent endpoint reports the same authoritative Ready/WaitStart
    # pre-state that the one-shot publisher is allowed to trigger from.
    if { [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_then_start" ] || [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_waitstart_then_start_regress" ] || [ "${FAKE_START_BEHAVIOR}" = "persistent_post_watermark_start" ]; } && [ "$(cat "${FAKE_START_PUBLISH_COUNT}")" -ge 1 ]; then
        observer_reads=$(( $(cat "${FAKE_OBSERVER_READ_COUNT}") + 1 ))
        printf '%s' "${observer_reads}" >"${FAKE_OBSERVER_READ_COUNT}"
        if [ "${observer_reads}" -ge "${FAKE_ADMIN_WAITSTART_READS_BEFORE_START}" ]; then
            state=start
            printf start >"${FAKE_ADMIN_STATE}"
        fi
    fi
    if [ "${FAKE_START_BEHAVIOR}" = "persistent_post_watermark_start" ] && [ "${state}" = "playstart" ]; then state=waitstart; fi
    if [ "${FAKE_START_BEHAVIOR}" = "observer_terminal_after_pulse" ] && [ "$(cat "${FAKE_START_PUBLISH_COUNT}")" -ge 1 ]; then state=finish; fi
    seq=$(( $(wc -l <"${jsonl}") + 1 ))
    printf '{"monotonic_ns":%s,"seq":%s,"state":"%s","token":"%s"}\\n' "${seq}" "${seq}" "${state}" "${token}" >>"${jsonl}"
    printf '%s %s %s\\n' "${seq}" "${state}" "${seq}"
    exit 0
fi
printf '%s|typed service wait %s\\n' "${ROS_DOMAIN_ID:-0}" "$*" >>"${FAKE_ROS_CALLS}"
if [ "${FAKE_SERVICE_BEHAVIOR}" = "ready_during_preflight" ]; then
    : >"${FAKE_READY_DURING_PREFLIGHT}"
fi
if [ "${FAKE_SERVICE_BEHAVIOR}" = "delayed_ready" ]; then
    sleep 0.2
fi
if [ "${FAKE_SERVICE_BEHAVIOR}" = "missing" ] || {
    [ "${FAKE_SERVICE_MISSING_AFTER_START}" = "true" ] && \
        { [ "$(cat "${FAKE_ADMIN_STATE}")" = "start" ] || \
            { [ "${FAKE_START_BEHAVIOR}" = "domain_start_admin_playstart" ] && \
                [ "$(cat "${FAKE_START_PUBLISH_COUNT}")" -ge 1 ]; }; }
}; then
    exit 1
fi
exit 0
""",
        encoding="utf-8",
    )
    fake_python3.chmod(0o755)
    environment = dict(os.environ)
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "FAKE_BIN": str(fake_bin),
            "FAKE_ADMIN_STATE": str(state_path),
            "FAKE_ROS_CALLS": str(calls_path),
            "FAKE_START_BEHAVIOR": start_behavior,
            "FAKE_START_PUBLISH_COUNT": str(start_publish_count_path),
            "FAKE_ADMIN_STATE_READ_COUNT": str(admin_state_read_count_path),
            "FAKE_OBSERVER_READ_COUNT": str(observer_read_count_path),
            "FAKE_OBSERVER_PID": str(observer_pid_path),
            "FAKE_RACE_OBSERVER_SCRIPT": str(
                Path(__file__).with_name("race_arm_observer_fixture.py")
            ),
            "FAKE_VEHICLE_OBSERVER_SCRIPT": str(
                Path(__file__).with_name("vehicle_readiness_observer_fixture.py")
            ),
            "FAKE_RACE_OBSERVER_PID_DIR": str(race_observer_pid_dir),
            "FAKE_VEHICLE_OBSERVER_PID_DIR": str(vehicle_observer_pid_dir),
            "FAKE_OBSERVER_DIRS": str(observer_dirs_path),
            "FAKE_ADMIN_WAITSTART_READS_BEFORE_START": str(
                admin_waitstart_reads_before_start
            ),
            "FAKE_SERVICE_BEHAVIOR": service_behavior,
            "FAKE_RACE_ARM_STATE_PATH": str(race_arm_state_path),
            "FAKE_RACE_ARM_EVENT_GENERATION_PATH": str(
                race_arm_event_generation_path
            ),
            "FAKE_RACE_ARM_AFTER_START": race_arm_state,
            "FAKE_RACE_ARM_STARTUP_BEHAVIOR": startup_race_arm_behavior,
            "FAKE_RACE_ARM_DOMAIN_2_INITIAL_STATE": domain_2_initial_race_arm_state,
            "FAKE_VEHICLE_STATE": vehicle_state,
            "FAKE_VEHICLE_STATE_AFTER_START": vehicle_state_after_start,
            "FAKE_DOMAIN_2_VEHICLE_STATE_AFTER_START": domain_2_vehicle_state_after_start,
            "FAKE_INITIALIZATION_READY_AFTER_START": initialization_ready_after_start,
            "FAKE_SERVICE_MISSING_AFTER_START": str(service_missing_after_start).lower(),
            "FAKE_PUBLISHER_PID": str(publisher_pid_path),
            "FAKE_PUBLISHER_TERMINATED": str(publisher_terminated_path),
            "FAKE_SERVICE_PID": str(service_pid_path),
            "FAKE_SERVICE_CHILD_PID": str(service_child_pid_path),
            "FAKE_SERVICE_TERMINATED": str(service_terminated_path),
            "FAKE_READY_DURING_PREFLIGHT": str(ready_during_preflight_path),
            "FAKE_RESET_OCCURRED": str(reset_occurred_path),
            "FAKE_POST_ARM_INITIALIZATION_FALSE": str(
                post_arm_initialization_false_path
            ),
            "AWSIM_READY_DOMAINS": ready_domains,
            # The Helper must retain its configured rollback/reap reserve at
            # commit. Phase-local failure bounds remain 1-2 seconds below.
            "AWSIM_START_TIMEOUT_SEC": "30",
            "AWSIM_START_PUBLISH_TIMEOUT_SEC": "3",
            "AWSIM_RACE_ARM_TIMEOUT_SEC": "1",
            "AWSIM_OFFICIAL_START_CALL_TIMEOUT_SEC": "2",
            "AWSIM_CHILD_REAP_TIMEOUT_SEC": "5",
            "AWSIM_START_POLL_INTERVAL_SEC": "0.01",
            "AWSIM_START_MODE": start_mode,
        }
    )
    if use_production_timeouts:
        for key in (
            "AWSIM_START_TIMEOUT_SEC",
            "AWSIM_START_PUBLISH_TIMEOUT_SEC",
            "AWSIM_RACE_ARM_TIMEOUT_SEC",
            "AWSIM_OFFICIAL_START_CALL_TIMEOUT_SEC",
            "AWSIM_CHILD_REAP_TIMEOUT_SEC",
            "AWSIM_START_POLL_INTERVAL_SEC",
        ):
            environment.pop(key, None)
    if gate_deadline_offset_sec is not None:
        environment["AIC_GATE_DEADLINE_MONOTONIC_NS"] = str(
            time.monotonic_ns() + int(gate_deadline_offset_sec * 1_000_000_000)
        )
    command = ["bash", str(Path(__file__).with_name("request_awsim_start.bash"))]
    if signal_during_partial_start:
        process = subprocess.Popen(
            command,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            calls_text = (
                calls_path.read_text(encoding="utf-8")
                if calls_path.is_file()
                else ""
            )
            if "2|service call" in calls_text and "{data: true}" in calls_text:
                break
            time.sleep(0.02)
        else:
            process.kill()
            process.wait(timeout=5)
            pytest.fail("partial official Start service call was not observed")
        process.terminate()
        stdout, stderr = process.communicate(timeout=15)
        result = subprocess.CompletedProcess(
            command, process.returncode, stdout=stdout, stderr=stderr
        )
    else:
        result = subprocess.run(
            command,
            env=environment,
            check=False,
            capture_output=True,
            text=True,
            timeout=15,
        )
    if publisher_pid_path.is_file():
        publisher_pid = int(publisher_pid_path.read_text(encoding="utf-8"))
        assert publisher_terminated_path.read_text(encoding="utf-8") == "terminated"
        with pytest.raises(ProcessLookupError):
            os.kill(publisher_pid, 0)
    if service_pid_path.is_file():
        service_pid = int(service_pid_path.read_text(encoding="utf-8"))
        if service_behavior == "block_partial":
            assert service_terminated_path.read_text(encoding="utf-8") == "terminated"
        with pytest.raises(ProcessLookupError):
            os.kill(service_pid, 0)
    if service_child_pid_path.is_file():
        service_child_pid = int(service_child_pid_path.read_text(encoding="utf-8"))
        with pytest.raises(ProcessLookupError):
            os.kill(service_child_pid, 0)
    if observer_pid_path.is_file():
        observer_pid = int(observer_pid_path.read_text(encoding="utf-8"))
        with pytest.raises(ProcessLookupError):
            os.kill(observer_pid, 0)
    for race_observer_pid_path in race_observer_pid_dir.glob("*.pid"):
        race_observer_pid = int(race_observer_pid_path.read_text(encoding="utf-8"))
        with pytest.raises(ProcessLookupError):
            os.kill(race_observer_pid, 0)
    for vehicle_observer_pid_path in vehicle_observer_pid_dir.glob("*.pid"):
        vehicle_observer_pid = int(vehicle_observer_pid_path.read_text(encoding="utf-8"))
        with pytest.raises(ProcessLookupError):
            os.kill(vehicle_observer_pid, 0)
    for observer_dir in observer_dirs_path.read_text(encoding="utf-8").splitlines():
        assert not Path(observer_dir).exists()
    calls = calls_path.read_text(encoding="utf-8").splitlines() if calls_path.exists() else []
    assert not any(
        "topic echo" in call and "/overtake/race_armed" in call for call in calls
    )
    assert not any("topic echo" in call and " /awsim/state" in call for call in calls)
    assert not any(
        "topic echo" in call and "/autostart/initialization_ready" in call
        for call in calls
    )
    return result, calls


def test_start_helper_rejects_duplicate_ready_domains_before_ros(tmp_path: Path) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path, "playstart", ready_domains="1,1"
    )
    assert result.returncode == 2
    assert "duplicate vehicle domain in AWSIM_READY_DOMAINS: 1" in result.stderr
    assert calls == []


def test_start_helper_never_sends_official_true_after_reserve_exhaustion(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path, "playstart", gate_deadline_offset_sec=5
    )
    assert result.returncode != 0
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )


def test_start_helper_refuses_to_spawn_any_observer_without_cleanup_reserve(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path, "playstart", gate_deadline_offset_sec=4
    )

    assert result.returncode != 0
    assert "refusing observer startup without cleanup deadline reserve=5s" in result.stderr
    assert not any("race arm observer snapshot=0" in call for call in calls)
    assert not any("topic pub" in call for call in calls)
    assert not any("service call" in call for call in calls)


def test_production_defaults_allow_commit_with_runtime25_ready_at_28_seconds(
    tmp_path: Path,
) -> None:
    # Runtime25 reached the Ready/commit path about 28 seconds into the fixed
    # 60-second Gate.  Do not shorten any Helper timeout in this fixture: the
    # remaining 32 seconds must cover nominal true/post-true work while the
    # production rollback and global-reap reserve remains intact.
    started = time.monotonic()
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "ready",
        gate_deadline_offset_sec=32,
        use_production_timeouts=True,
    )
    elapsed = time.monotonic() - started
    assert result.returncode == 0
    assert elapsed < 32
    assert any("service call" in call and "{data: true}" in call for call in calls)
    assert "AWSIM race start confirmed" in result.stdout


def test_production_defaults_refuse_start_pulse_without_reset_reap_reserve(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        gate_deadline_offset_sec=5,
        use_production_timeouts=True,
    )
    assert result.returncode != 0
    assert not any(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    )
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )


def test_start_helper_accepts_only_post_publish_transition(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(tmp_path, "ready")

    assert result.returncode == 0
    assert "one-shot Start pulse publisher completed" in result.stdout
    assert any("service call" in call and "{data: true}" in call for call in calls)


def test_start_helper_polls_until_delayed_post_publish_transition(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path, "waitstart", start_behavior="delayed_transition"
    )

    assert result.returncode == 0
    assert "one-shot Start pulse publisher completed" in result.stdout
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert any("service call" in call and "{data: true}" in call for call in calls)


def test_start_helper_accepts_waitstart_one_shot_transition(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="domain_start_admin_waitstart_then_start",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
    )

    assert result.returncode == 0
    assert "one-shot Start pulse publisher completed" in result.stdout
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert any("service call" in call and "{data: true}" in call for call in calls)


def test_start_helper_accepts_exact_start_when_transient_ready_is_not_sampled(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="transition",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
    )

    assert result.returncode == 0
    assert "one-shot Start pulse publisher completed" in result.stdout
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert sum(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    ) == 0
    assert any(
        "service call" in call and "{data: true}" in call for call in calls
    )
    assert "AWSIM race start confirmed" in result.stdout


@pytest.mark.parametrize(
    ("initial_admin_state", "vehicle_state"),
    [("ready", "Ready"), ("waitstart", "Grounded")],
)
def test_start_helper_ready_or_waitstart_uses_exactly_one_official_pulse(
    tmp_path: Path,
    initial_admin_state: str,
    vehicle_state: str,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        initial_admin_state,
        start_behavior="transition",
        vehicle_state=vehicle_state,
        vehicle_state_after_start="Start",
    )

    assert result.returncode == 0, result.stderr
    start_publishes = [
        call
        for call in calls
        if "topic pub" in call and "/admin/awsim/start" in call
    ]
    assert len(start_publishes) == 1
    assert " -1 " in f" {start_publishes[0]} "
    assert "--rate" not in start_publishes[0]
    assert not any("/admin/awsim/reset" in call for call in calls)
    assert sum(
        "service call" in call and "{data: true}" in call for call in calls
    ) == 1


def test_start_helper_accepts_start_then_ready_history(tmp_path: Path) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="start_then_ready_after_publish",
        vehicle_state="Grounded",
        vehicle_state_after_start="Ready",
    )

    assert result.returncode == 0, result.stderr
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert not any("/admin/awsim/reset" in call for call in calls)
    assert "vehicle domain 1 state: ready" in result.stdout
    assert "AWSIM race start confirmed" in result.stdout


def test_start_helper_rejects_post_pulse_race_arm_true_false_burst(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="transition",
        startup_race_arm_behavior="post_pulse_burst_true_false",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
    )

    assert result.returncode != 0
    assert sum("/admin/awsim/start" in call for call in calls) == 1
    assert sum("/admin/awsim/reset" in call for call in calls) == 1
    assert not any("service call" in call and "{data: true}" in call for call in calls)
    assert "did not remain continuously false" in result.stderr


@pytest.mark.parametrize("invalid_state", ["grounded", "finish"])
def test_start_helper_rolls_back_start_then_forbidden_state(
    tmp_path: Path,
    invalid_state: str,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior=f"start_then_{invalid_state}_after_publish",
        vehicle_state="Grounded",
        vehicle_state_after_start=invalid_state.title(),
    )

    assert result.returncode != 0
    assert "forbidden vehicle state after the one-shot Start watermark" in result.stderr
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert sum("/admin/awsim/reset" in call for call in calls) == 1
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )


def test_start_helper_preserves_ready_start_burst_in_persistent_vehicle_observer(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="transition",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
    )

    assert result.returncode == 0, result.stderr
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert any("vehicle readiness observer snapshot=1" in call for call in calls)
    assert not any("topic echo" in call and " /awsim/state" in call for call in calls)


def test_start_helper_keeps_pre_pulse_initialization_true_without_post_pulse_republish(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="transition",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
        initialization_ready_after_start="none",
    )

    assert result.returncode == 0, result.stderr
    assert any("service call" in call and "{data: true}" in call for call in calls)
    assert not any(
        "topic echo" in call and "/autostart/initialization_ready" in call
        for call in calls
    )


def test_start_helper_rejects_initialization_false_after_pre_pulse_true(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="transition",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
        initialization_ready_after_start="false",
    )

    assert result.returncode != 0
    assert "initialization readiness became false after the pre-pulse true watermark" in result.stderr
    assert not any("service call" in call and "{data: true}" in call for call in calls)
    assert any("topic pub" in call and "/admin/awsim/reset" in call for call in calls)


def test_start_helper_rejects_vehicle_observer_death_after_pulse_and_cleans_up(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="vehicle_observer_exit_after_pulse",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
    )

    assert result.returncode != 0
    assert "persistent vehicle readiness observer died" in result.stderr
    assert not any("service call" in call and "{data: true}" in call for call in calls)


def test_start_helper_rolls_back_if_initialization_falls_after_fresh_race_arm_true(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="transition",
        service_behavior="post_arm_init_false",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
    )

    assert result.returncode != 0
    assert sum("service call" in call and "{data: true}" in call for call in calls) == 1
    assert sum("service call" in call and "{data: false}" in call for call in calls) == 1
    assert sum("/admin/awsim/reset" in call for call in calls) == 1
    assert "initialization readiness became false" in result.stderr
    assert "AWSIM race start confirmed" not in result.stdout


def test_start_helper_rejects_waitstart_when_admin_never_reaches_start_after_one_pulse(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="domain_start_admin_waitstart",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
        ready_domains="1,2",
    )

    assert result.returncode != 0
    start_publishes = [
        call
        for call in calls
        if "topic pub" in call and "/admin/awsim/start" in call
    ]
    assert len(start_publishes) == 1
    assert " -1 " in f" {start_publishes[0]} "
    assert any(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    )
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )
    assert not any(
        "service call" in call and "{data: false}" in call for call in calls
    )
    assert sum(
        "race arm observer snapshot=1" in call for call in calls
    ) >= 4
    assert "AWSIM race start confirmed" not in result.stdout


def test_start_helper_accepts_real_sync_waitstart_exact_start_after_one_pulse(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="domain_start_admin_waitstart_then_start",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
        ready_domains="1,2",
    )

    assert result.returncode == 0
    assert "one-shot Start pulse publisher completed" in result.stdout
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1


def test_start_helper_accepts_persistent_post_watermark_start_when_one_shot_stays_waitstart(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="persistent_post_watermark_start",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
    )
    assert result.returncode == 0
    admin_one_shots = [call for call in calls if "/admin/awsim/state" in call]
    assert admin_one_shots
    assert sum("topic pub" in call and "/admin/awsim/start" in call for call in calls) == 1
    assert sum(
        "service call" in call and "{data: true}" in call for call in calls
    ) == 1
    assert not any(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    )
    assert "AWSIM race start confirmed" in result.stdout


def test_start_helper_rejects_observer_start_before_pulse_without_waitstart(tmp_path: Path) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path, "start", start_behavior="observer_start_only"
    )
    assert result.returncode != 0
    assert sum("topic pub" in call and "/admin/awsim/start" in call for call in calls) == 0
    assert sum("topic pub" in call and "/admin/awsim/reset" in call for call in calls) == 0
    assert not any("service call" in call and "{data: true}" in call for call in calls)


def test_start_helper_rejects_terminal_observer_event_after_pulse(tmp_path: Path) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path, "playstart", start_behavior="observer_terminal_after_pulse", vehicle_state="Grounded"
    )
    assert result.returncode != 0
    assert sum("topic pub" in call and "/admin/awsim/start" in call for call in calls) == 1
    assert sum("topic pub" in call and "/admin/awsim/reset" in call for call in calls) == 1
    assert not any("service call" in call and "{data: true}" in call for call in calls)


def test_start_helper_rejects_observer_startup_exit_and_reaps_it(tmp_path: Path) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path, "playstart", start_behavior="observer_startup_exit"
    )
    assert result.returncode != 0
    assert not any("/admin/awsim/start" in call for call in calls)
    assert not any("/admin/awsim/reset" in call for call in calls)
    assert not any("service call" in call and "{data: true}" in call for call in calls)


def test_start_helper_rejects_observer_exit_after_pulse_and_reaps_it(tmp_path: Path) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path, "playstart", start_behavior="observer_exit_after_pulse", vehicle_state="Grounded"
    )
    assert result.returncode != 0
    assert sum("/admin/awsim/start" in call for call in calls) == 1
    assert sum("/admin/awsim/reset" in call for call in calls) == 1
    assert not any("service call" in call and "{data: true}" in call for call in calls)


def test_start_helper_polls_repeated_waitstart_until_start_after_one_pulse(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="domain_start_admin_waitstart_then_start",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
        admin_waitstart_reads_before_start=4,
        ready_domains="1,2",
    )

    assert result.returncode == 0
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert not any(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    )
    assert sum(
        "service call" in call and "{data: true}" in call for call in calls
    ) == 2
    for domain in ("1", "2"):
        assert any(
            call.startswith(f"{domain}|race arm observer snapshot=0")
            for call in calls
        )
        service_index = max(
            index
            for index, call in enumerate(calls)
            if call.startswith(f"{domain}|service call") and "{data: true}" in call
        )
        assert any(
            index > service_index
            and call.startswith(f"{domain}|race arm observer snapshot=1")
            for index, call in enumerate(calls)
        )
    assert "AWSIM race start confirmed" in result.stdout


def test_start_helper_resets_when_direct_start_waitstart_times_out(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "playstart",
        start_behavior="domain_start_admin_waitstart",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
        ready_domains="1,2",
    )

    assert result.returncode != 0
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert sum(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    ) == 1
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )
    assert "AWSIM race start confirmed" not in result.stdout


def test_start_helper_resets_if_domain_regresses_after_direct_start_detection(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="domain_start_admin_waitstart_then_start_regress",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
        ready_domains="1,2",
    )

    assert result.returncode != 0
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert sum(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    ) == 1
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )
    assert "forbidden vehicle state" in result.stderr
    assert "AWSIM race start confirmed" not in result.stdout


def test_start_helper_ambiguous_waitstart_publish_failure_resets(
    tmp_path: Path,
) -> None:
    started = time.monotonic()
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="domain_start_admin_waitstart_pub_fail",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
    )
    elapsed = time.monotonic() - started

    assert result.returncode != 0
    assert elapsed < 5
    assert "delivery is ambiguous" in result.stderr
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert sum(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    ) == 1
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )


def test_start_helper_rejects_retained_race_arm_before_pulse(tmp_path: Path) -> None:
    started = time.monotonic()
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "playstart",
        start_behavior="domain_start_admin_waitstart",
        vehicle_state="Grounded",
        initial_race_arm_state="true",
    )
    elapsed = time.monotonic() - started

    assert result.returncode != 0
    assert elapsed < 3
    assert not any(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    )
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )
    assert not any("/admin/awsim/reset" in call for call in calls)
    assert "startup race-arm history observed forbidden true" in result.stderr


def test_start_helper_waits_for_valid_empty_then_delayed_false_before_actions(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "playstart",
        initial_race_arm_state="empty",
        startup_race_arm_behavior="delayed_false",
    )

    assert result.returncode == 0
    first_start_action = next(
        index
        for index, call in enumerate(calls)
        if ("topic pub" in call and "/admin/awsim/start" in call)
        or ("service call" in call)
    )
    assert sum("race arm observer snapshot=1" in call for call in calls[:first_start_action]) >= 2
    assert "AWSIM race start confirmed" in result.stdout


def test_start_helper_empty_startup_race_arm_times_out_without_actions_or_residue(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "playstart",
        initial_race_arm_state="empty",
    )

    assert result.returncode != 0
    assert "startup race-arm evidence did not become exact false" in result.stderr
    assert not any("topic pub" in call for call in calls)
    assert not any("service call" in call for call in calls)


def test_start_helper_empty_then_true_is_immediately_terminal_without_actions(
    tmp_path: Path,
) -> None:
    started = time.monotonic()
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "playstart",
        initial_race_arm_state="empty",
        startup_race_arm_behavior="delayed_true",
    )

    assert result.returncode != 0
    assert time.monotonic() - started < 3
    assert "startup race-arm history observed forbidden true" in result.stderr
    assert not any("topic pub" in call for call in calls)
    assert not any("service call" in call for call in calls)


def test_start_helper_rejects_true_then_false_burst_between_startup_polls(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "playstart",
        initial_race_arm_state="empty",
        startup_race_arm_behavior="burst_true_false",
    )

    assert result.returncode != 0
    assert "startup race-arm history observed forbidden true" in result.stderr
    assert not any("topic pub" in call for call in calls)
    assert not any("service call" in call for call in calls)


@pytest.mark.parametrize("startup_behavior", ["observer_exit", "malformed", "missing"])
def test_start_helper_startup_observer_failure_is_terminal_without_actions(
    tmp_path: Path,
    startup_behavior: str,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "playstart",
        initial_race_arm_state="empty",
        startup_race_arm_behavior=startup_behavior,
    )

    assert result.returncode != 0
    assert "observer exited or emitted invalid/missing evidence" in result.stderr
    assert not any("topic pub" in call for call in calls)
    assert not any("service call" in call for call in calls)


@pytest.mark.parametrize(
    ("domain_2_state", "expected_error"),
    [
        ("true", "startup race-arm history observed forbidden true"),
        ("empty", "startup race-arm evidence did not become exact false"),
    ],
)
def test_start_helper_multi_domain_true_or_empty_blocks_all_actions(
    tmp_path: Path,
    domain_2_state: str,
    expected_error: str,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "playstart",
        ready_domains="1,2",
        initial_race_arm_state="false",
        domain_2_initial_race_arm_state=domain_2_state,
    )

    assert result.returncode != 0
    assert expected_error in result.stderr
    assert not any("topic pub" in call for call in calls)
    assert not any("service call" in call for call in calls)


@pytest.mark.parametrize(
    ("vehicle_state_after_start", "initialization_ready_after_start"),
    [("Start", "false"), ("Grounded", "true")],
)
def test_start_helper_one_shot_barrier_failure_resets(
    tmp_path: Path,
    vehicle_state_after_start: str,
    initialization_ready_after_start: str,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="transition",
        vehicle_state="Grounded",
        vehicle_state_after_start=vehicle_state_after_start,
        initialization_ready_after_start=initialization_ready_after_start,
    )

    assert result.returncode != 0
    start_publishes = [
        call
        for call in calls
        if "topic pub" in call and "/admin/awsim/start" in call
    ]
    assert len(start_publishes) == 1
    assert " -1 " in f" {start_publishes[0]} "
    assert sum(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    ) == 1
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )
    assert not any(
        "service call" in call and "{data: false}" in call for call in calls
    )
    assert any(
        "race arm observer snapshot=1" in call for call in calls
    )
    assert "AWSIM race start confirmed" not in result.stdout


def test_start_helper_one_shot_partial_domain_start_resets(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="domain_start_admin_waitstart_then_start",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
        domain_2_vehicle_state_after_start="Grounded",
        ready_domains="1,2",
    )

    assert result.returncode != 0
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert sum(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    ) == 1
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )
    assert not any(
        "service call" in call and "{data: false}" in call for call in calls
    )
    assert any(
        "race arm observer snapshot=1" in call for call in calls
    )
    assert "AWSIM race start confirmed" not in result.stdout


@pytest.mark.parametrize(
    (
        "service_behavior",
        "service_missing_after_start",
        "race_arm_state",
        "expected_true_calls",
        "minimum_race_arm_reads",
    ),
    [
        ("accept", True, "true", 0, 1),
        ("reject", False, "true", 1, 1),
        ("accept", False, "false", 1, 2),
    ],
)
def test_start_helper_one_shot_commit_failure_rolls_back(
    tmp_path: Path,
    service_behavior: str,
    service_missing_after_start: bool,
    race_arm_state: str,
    expected_true_calls: int,
    minimum_race_arm_reads: int,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="domain_start_admin_waitstart_then_start",
        service_behavior=service_behavior,
        service_missing_after_start=service_missing_after_start,
        race_arm_state=race_arm_state,
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
    )

    assert result.returncode != 0
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert sum(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    ) == 1
    assert sum(
        "service call" in call and "{data: true}" in call for call in calls
    ) == expected_true_calls
    assert sum(
        "service call" in call and "{data: false}" in call for call in calls
    ) == 1
    assert sum(
        "race arm observer snapshot=1" in call for call in calls
    ) >= minimum_race_arm_reads
    assert "AWSIM race start confirmed" not in result.stdout


def test_start_helper_uses_one_shot_when_vehicle_becomes_ready_during_preflight(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="transition",
        service_behavior="ready_during_preflight",
        vehicle_state="Grounded",
        vehicle_state_after_start="Start",
    )

    assert result.returncode == 0
    assert "vehicle domain 1 state: ready" in result.stdout
    assert "one-shot Start pulse publisher completed" in result.stdout
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert not any(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    )
    assert any("service call" in call and "{data: true}" in call for call in calls)


def test_start_helper_forbidden_post_pulse_state_resets_immediately(
    tmp_path: Path,
) -> None:
    started = time.monotonic()
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="start_then_grounded_after_publish",
        vehicle_state="Grounded",
        vehicle_state_after_start="Grounded",
    )
    elapsed = time.monotonic() - started

    assert result.returncode != 0
    assert elapsed < 3
    assert "entered a forbidden vehicle state" in result.stderr
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert sum(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    ) == 1
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )


def test_start_helper_resets_when_one_shot_does_not_reach_vehicle_start(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="transition",
        vehicle_state="Grounded",
        vehicle_state_after_start="Ready",
    )

    assert result.returncode != 0
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert any(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    )
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )
    assert "AWSIM race start confirmed" not in result.stdout


@pytest.mark.parametrize("terminal_vehicle_state", ["LapComplete", "Finish"])
def test_start_helper_rejects_non_start_state_after_one_shot_publish(
    tmp_path: Path,
    terminal_vehicle_state: str,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="transition",
        vehicle_state="Grounded",
        vehicle_state_after_start=terminal_vehicle_state,
    )

    assert result.returncode != 0
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert any(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    )
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )


@pytest.mark.parametrize(
    (
        "vehicle_state_after_start",
        "initialization_ready_after_start",
        "service_missing_after_start",
    ),
    [
        ("Grounded", "true", False),
        ("Start", "false", False),
        ("Start", "true", True),
    ],
)
def test_start_helper_rechecks_authority_inputs_after_proved_transition(
    tmp_path: Path,
    vehicle_state_after_start: str,
    initialization_ready_after_start: str,
    service_missing_after_start: bool,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="transition",
        vehicle_state="Grounded",
        vehicle_state_after_start=vehicle_state_after_start,
        initialization_ready_after_start=initialization_ready_after_start,
        service_missing_after_start=service_missing_after_start,
    )

    assert result.returncode != 0
    assert not any("service call" in call and "{data: true}" in call for call in calls)
    assert "AWSIM race start confirmed" not in result.stdout


@pytest.mark.parametrize("initial_admin_state", ["", "unknown", "finish"])
def test_start_helper_rejects_non_allowlisted_pre_publish_admin_state(
    tmp_path: Path,
    initial_admin_state: str,
) -> None:
    result, calls = run_start_helper_with_fake_ros(tmp_path, initial_admin_state)

    assert result.returncode != 0
    assert not any("topic pub" in call and "/admin/awsim/start" in call for call in calls)
    assert not any("service call" in call and "{data: true}" in call for call in calls)


def test_start_helper_rejects_publisher_without_admin_transition(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path, "waitstart", start_behavior="no_transition"
    )

    assert result.returncode != 0
    assert "one-shot Start did not produce post-watermark vehicle Start history and admin Start" in result.stderr
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert sum(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    ) == 1
    assert not any("service call" in call for call in calls)


def test_start_helper_rejects_preexisting_retained_start(
    tmp_path: Path,
) -> None:
    started = time.monotonic()
    result, calls = run_start_helper_with_fake_ros(tmp_path, "start")
    elapsed = time.monotonic() - started

    assert result.returncode != 0
    assert elapsed < 3
    assert (
        "refusing retained admin Start without a helper-owned "
        "Ready/WaitStart one-shot transition"
    ) in result.stderr
    assert not any(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    )
    assert not any("/admin/awsim/reset" in call for call in calls)
    assert not any("service call" in call for call in calls)


def test_start_helper_rejects_lapcomplete_as_final_start_prestate(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(tmp_path, "lapcomplete")

    assert result.returncode != 0
    assert not any(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    )
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )


def test_start_helper_rejects_lapcomplete_between_count_and_final_start(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        start_behavior="two_stage_lapcomplete_pre_final",
        vehicle_state="Grounded",
    )

    assert result.returncode != 0
    assert sum(
        "topic pub" in call and "/admin/awsim/start" in call for call in calls
    ) == 1
    assert any(
        "topic pub" in call and "/admin/awsim/reset" in call for call in calls
    )
    assert not any(
        "service call" in call and "{data: true}" in call for call in calls
    )


def test_start_helper_rejects_retained_start_across_multiple_domains(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "start",
        vehicle_state="Start",
        ready_domains="1,2",
    )

    assert result.returncode != 0
    assert not any("service call" in call for call in calls)


def test_start_helper_rejects_missing_official_start_service(
    tmp_path: Path,
) -> None:
    started_at = time.monotonic()
    result, calls = run_start_helper_with_fake_ros(
        tmp_path, "waitstart", service_behavior="missing"
    )
    elapsed = time.monotonic() - started_at

    assert result.returncode != 0
    assert elapsed < 15
    assert "official Start service preflight failed before any Start pulse; terminal" in result.stderr
    assert not any("topic pub" in call for call in calls)
    assert not any("service call" in call for call in calls)


def test_start_helper_accepts_bounded_delayed_official_start_service(
    tmp_path: Path,
) -> None:
    started_at = time.monotonic()
    result, calls = run_start_helper_with_fake_ros(
        tmp_path, "waitstart", service_behavior="delayed_ready"
    )
    elapsed = time.monotonic() - started_at

    assert result.returncode == 0, result.stderr
    assert elapsed < 15
    assert any("typed service wait" in call for call in calls)
    assert any("service call" in call and "{data: true}" in call for call in calls)


def test_start_helper_rolls_back_rejected_official_start(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path, "waitstart", service_behavior="reject"
    )

    assert result.returncode != 0
    assert any("service call" in call and "{data: true}" in call for call in calls)
    assert any("service call" in call and "{data: false}" in call for call in calls)
    assert any("topic pub" in call and "/admin/awsim/reset" in call for call in calls)
    assert "AWSIM race start confirmed" not in result.stdout


def test_start_helper_rejects_old_false_when_delayed_true_arrives_after_rollback_dispatch(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        service_behavior="delayed_true_old_false",
    )

    assert result.returncode != 0
    true_call = next(
        index
        for index, call in enumerate(calls)
        if "service call" in call and "{data: true}" in call
    )
    false_call = next(
        index
        for index, call in enumerate(calls)
        if "service call" in call and "{data: false}" in call
    )
    post_false_snapshots = [
        index
        for index, call in enumerate(calls)
        if index > false_call and "race arm observer snapshot=1" in call
    ]
    assert true_call < false_call
    assert post_false_snapshots
    assert any("topic pub" in call and "/admin/awsim/reset" in call for call in calls)
    assert "did not report post-dispatch false" in result.stderr
    assert "AWSIM race start confirmed" not in result.stdout


def test_start_helper_rolls_back_when_race_arm_does_not_confirm(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path, "waitstart", race_arm_state="false"
    )

    assert result.returncode != 0
    assert any("service call" in call and "{data: true}" in call for call in calls)
    assert any("service call" in call and "{data: false}" in call for call in calls)
    assert any("topic pub" in call and "/admin/awsim/reset" in call for call in calls)
    assert "did not become true" in result.stderr
    assert "AWSIM race start confirmed" not in result.stdout


def test_start_helper_signal_reaps_partial_start_and_rolls_back(
    tmp_path: Path,
) -> None:
    result, calls = run_start_helper_with_fake_ros(
        tmp_path,
        "waitstart",
        service_behavior="ignore_term_partial",
        race_arm_state="false",
        ready_domains="1,2",
        signal_during_partial_start=True,
    )

    assert result.returncode == 143
    assert sum(
        "service call" in call and "{data: false}" in call for call in calls
    ) == 2
    assert any("topic pub" in call and "/admin/awsim/reset" in call for call in calls)
    assert "AWSIM race start confirmed" not in result.stdout


def test_gate2_ready_is_not_a_motion_arm_authority() -> None:
    makefile = Path(__file__).parents[1] / "Makefile"
    source = makefile.read_text(encoding="utf-8")

    gate_recipe = source[source.index("gate1 gate2 gate3:") : source.index(
        "# Kept for backward compatibility"
    )]
    assert 'RACE_ARM_ON_VEHICLE_STATE="Start"' in gate_recipe
    assert 'RACE_ARM_ON_VEHICLE_STATE="Ready,Start"' not in gate_recipe
    assert "AWSIM_READY_DOMAINS=1" in gate_recipe
    assert 'AWSIM_START_MODE="$$start_mode"' in gate_recipe

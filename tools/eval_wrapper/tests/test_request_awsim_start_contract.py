from pathlib import Path
import re


REPO_ROOT = Path(__file__).resolve().parents[3]
START_HELPER = (
    REPO_ROOT
    / "tools/scripts/headless_overrides/aichallenge/request_awsim_start.bash"
)
INSTALLED_SOURCE_HELPER = REPO_ROOT / "aichallenge/request_awsim_start.bash"
RACE_ARM_OBSERVER = REPO_ROOT / "aichallenge/race_arm_observer.py"
HEADLESS_RACE_ARM_OBSERVER = (
    REPO_ROOT / "tools/scripts/headless_overrides/aichallenge/race_arm_observer.py"
)
VEHICLE_READINESS_OBSERVER = REPO_ROOT / "aichallenge/vehicle_readiness_observer.py"
HEADLESS_VEHICLE_READINESS_OBSERVER = (
    REPO_ROOT
    / "tools/scripts/headless_overrides/aichallenge/vehicle_readiness_observer.py"
)
AUTOSTART_ORCHESTRATOR = (
    REPO_ROOT
    / "aichallenge/workspace/src/aichallenge_system/autostart_orchestrator_py/"
    "autostart_orchestrator_py/autostart_orchestrator_node.py"
)
HEADLESS_AUTOSTART_ORCHESTRATOR = (
    REPO_ROOT
    / "tools/scripts/headless_overrides/aichallenge/workspace/src/"
    "aichallenge_system/autostart_orchestrator_py/autostart_orchestrator_py/"
    "autostart_orchestrator_node.py"
)
SUPERVISOR = REPO_ROOT / "aichallenge/run_awsim_with_d1_watchdog.bash"
HEADLESS_SUPERVISOR = (
    REPO_ROOT
    / "tools/scripts/headless_overrides/aichallenge/run_awsim_with_d1_watchdog.bash"
)
HELPER_REAPER = REPO_ROOT / "aichallenge/helper_process_reaper.py"
HEADLESS_HELPER_REAPER = (
    REPO_ROOT / "tools/scripts/headless_overrides/aichallenge/helper_process_reaper.py"
)


def _function_body(source: str, name: str) -> str:
    match = re.search(
        rf"^{re.escape(name)}\(\) \{{\n(?P<body>.*?)^\}}$",
        source,
        flags=re.MULTILINE | re.DOTALL,
    )
    assert match is not None, f"missing shell function: {name}"
    return match.group("body")


def test_retained_state_readers_request_only_latest_sample() -> None:
    for helper in (START_HELPER, INSTALLED_SOURCE_HELPER):
        source = helper.read_text(encoding="utf-8")
        body = _function_body(source, "read_admin_state")
        assert "--qos-durability transient_local" in body
        assert "--qos-history keep_last" in body
        assert "--qos-depth 1" in body
        assert "read_vehicle_state()" not in source
        assert "read_initialization_ready()" not in source


def test_ready_is_wait_only_and_start_is_required_for_started_state() -> None:
    source = INSTALLED_SOURCE_HELPER.read_text(encoding="utf-8")
    observer = _function_body(source, "observe_vehicle_states")
    assert 'if [ "${vehicle_state}" != "ready" ]; then' in observer
    assert "all_vehicle_ready=false" in observer
    assert re.search(
        r'case "\$\{vehicle_state\}" in\s+start \| lapcomplete \| finish\)',
        observer,
    )
    assert "all_vehicle_started=false" in observer


def test_official_start_publish_is_exactly_one_bounded_pulse() -> None:
    for helper in (START_HELPER, INSTALLED_SOURCE_HELPER):
        source = helper.read_text(encoding="utf-8")
        publish_start = _function_body(source, "publish_start")
        assert "ready | waitstart)" in publish_start
        assert "selectmode | playstart | ready | waitstart)" not in publish_start
        assert "-1 \\" in {line.strip() for line in publish_start.splitlines()}
        assert "--wait-matching-subscriptions 1" in publish_start
        assert "--qos-reliability reliable" in publish_start
        assert "--qos-durability transient_local" in publish_start
        assert "--rate" not in publish_start
        assert "start_pulse_may_have_been_delivered=true" in publish_start
        assert 'wait "${publisher_pid}"' in publish_start
        assert "delivery is ambiguous" in publish_start
        assert "rollback_start_pulse" in publish_start


def test_one_shot_start_uses_persistent_history_and_single_commit_path() -> None:
    for helper in (START_HELPER, INSTALLED_SOURCE_HELPER):
        source = helper.read_text(encoding="utf-8")
        assert "publish_countdown_start_and_wait_ready" not in source
        assert "publish_final_start_and_wait_vehicle_start" not in source
        assert "publish_count_start_when_initialized" not in source
        assert "all_vehicle_exactly_start" not in source

        start = _function_body(source, "publish_official_one_shot_start")
        assert "check_all_official_start_services" in start
        assert start.count("wait_race_arm_all false false") == 2
        assert start.count("publish_start") == 1
        assert "wait_for_one_shot_start_barrier" in start
        assert "rollback_start_pulse" in start
        assert "arm_after_authoritative_start" in start

        observe = _function_body(source, "observe_vehicle_states")
        assert "vehicle_readiness_observer_saw_invalid_vehicle" in observe
        assert "all_vehicle_start_history_valid" in observe
        assert "vehicle_readiness_observer_seen_start" in observe

        commit = _function_body(source, "arm_after_authoritative_start")
        assert commit.count("all_vehicle_start_history_valid") >= 2
        assert "save_race_arm_false_watermarks" in commit
        assert "call_official_start_all true" in commit
        assert "wait_race_arm_all true true" in commit


def test_one_shot_barrier_accepts_ready_after_start_history() -> None:
    for helper in (START_HELPER, INSTALLED_SOURCE_HELPER):
        source = helper.read_text(encoding="utf-8")
        barrier = _function_body(source, "wait_for_one_shot_start_barrier")
        assert "publisher_timeout_sec" in barrier
        assert "admin_observer_snapshot" in barrier
        assert "observe_vehicle_states" in barrier
        assert "all_vehicle_start_history_valid" in barrier
        assert "all_vehicle_exactly_start" not in barrier
        assert "verify_pre_pulse_race_arm_stayed_false" in barrier
        assert 'ready | waitstart)' in barrier
        assert "start)" in barrier
        assert "current_run_start_transition_proved=true" in barrier
        assert "topic pub" not in barrier


def test_persistent_admin_observer_is_read_only_and_pre_pulse_watermarked() -> None:
    observer = REPO_ROOT / "aichallenge/admin_state_observer.py"
    source = observer.read_text(encoding="utf-8")
    helper = INSTALLED_SOURCE_HELPER.read_text(encoding="utf-8")
    assert "create_subscription(String, topic, callback, qos)" in source
    assert "ReliabilityPolicy.RELIABLE" in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "HistoryPolicy.KEEP_LAST" in source
    assert "depth=1" in source
    assert "create_publisher" not in source
    assert "time.monotonic_ns()" in source
    assert "os.O_APPEND" in source
    assert "os.write(self._jsonl_fd, encoded)" in source
    assert "save_start_observer_watermark" in helper
    assert "start_admin_observer" in helper
    assert "stop_all_observers" in helper


def test_persistent_race_arm_observers_are_per_domain_read_only_and_watermarked() -> None:
    source = RACE_ARM_OBSERVER.read_text(encoding="utf-8")
    for helper_path in (START_HELPER, INSTALLED_SOURCE_HELPER):
        helper = helper_path.read_text(encoding="utf-8")
        assert "read_race_arm()" not in helper
        assert "start_race_arm_observers" in helper
        assert 'setsid env ROS_DOMAIN_ID="${domain}" python3 "${race_arm_state_observer}"' in helper
        assert '--domain-id "${domain}"' in helper
        assert '--expected-domain "${domain}"' in helper
        assert "save_race_arm_false_watermarks" in helper
        assert "wait_race_arm_all true true" in helper
        wait_domain = _function_body(helper, "wait_race_arm_domain")
        assert '"${arm_seq}" -gt "${watermark_seq}"' in wait_domain
        assert '"${arm_monotonic_ns}" -gt "${service_begin_monotonic_ns}"' in wait_domain
        call_all = _function_body(helper, "call_official_start_all")
        assert "official_start_call_begin_monotonic_ns" in call_all
    assert "create_subscription(Bool, topic, callback, qos)" in source
    assert "ReliabilityPolicy.RELIABLE" in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "HistoryPolicy.KEEP_LAST" in source
    assert "depth=1" in source
    assert "create_publisher" not in source
    assert "time.monotonic_ns()" in source
    assert "os.O_APPEND" in source
    assert "os.write(self._jsonl_fd, encoded)" in source
    assert '"domain_id": self._domain_id' in source


def test_vehicle_and_initialization_use_same_persistent_post_watermark_history() -> None:
    observer = VEHICLE_READINESS_OBSERVER.read_text(encoding="utf-8")
    for helper_path in (START_HELPER, INSTALLED_SOURCE_HELPER):
        helper = helper_path.read_text(encoding="utf-8")
        assert "read_vehicle_state()" not in helper
        assert "read_initialization_ready()" not in helper
        assert "ros2 topic echo --no-daemon --once" not in _function_body(
            helper, "observe_vehicle_states"
        )
        assert "start_vehicle_readiness_observers" in helper
        assert 'setsid env ROS_DOMAIN_ID="${domain}" python3 "${vehicle_readiness_observer}"' in helper
        snapshot = _function_body(helper, "vehicle_readiness_observer_snapshot")
        assert "--history-watermark-seq" in snapshot
        assert "--initialization-watermark-seq" in snapshot
        assert 'if [ "${seq}" -lt "${previous_seq}" ]' in snapshot
        watermark = _function_body(helper, "save_vehicle_readiness_watermarks")
        assert "grounded | ready)" in watermark
        assert 'if [ "${initialization_ready}" != true ]' in watermark
        observe = _function_body(helper, "observe_vehicle_states")
        assert "vehicle_readiness_observer_seen_start" in observe
        assert "vehicle_readiness_observer_saw_invalid_vehicle" in observe
        assert "vehicle_readiness_observer_saw_initialization_false" in observe
        assert "initialization readiness became false after the pre-pulse true watermark" in observe
        assert _function_body(helper, "publish_start").index(
            "save_vehicle_readiness_watermarks"
        ) < _function_body(helper, "publish_start").index("topic pub")
    assert "create_subscription(String, vehicle_topic, vehicle_callback, qos)" in observer
    assert "create_subscription(Bool, initialization_topic, initialization_callback, qos)" in observer
    assert "ReliabilityPolicy.RELIABLE" in observer
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in observer
    assert "HistoryPolicy.KEEP_LAST" in observer
    assert "depth=10" in observer
    assert "create_publisher" not in observer
    assert "os.O_APPEND" in observer
    assert "record[\"seq\"] != previous_seq + 1" in observer
    assert "saw_invalid_vehicle_after_watermark" in observer


def test_authoritative_commit_rechecks_start_history_admin_service_and_disarm() -> None:
    for helper in (START_HELPER, INSTALLED_SOURCE_HELPER):
        source = helper.read_text(encoding="utf-8")
        commit = _function_body(source, "arm_after_authoritative_start")
        assert commit.count("admin_observer_snapshot") >= 3
        assert 'observed_admin_state="${admin_observer_state}"' in commit
        assert '[ "${observed_admin_state}" != "start" ]' in commit
        assert '[ "${all_vehicle_start_history_valid}" != true ]' in commit
        assert '[ "${all_vehicle_initialization_ready}" != true ]' in commit
        assert commit.count("check_all_official_start_services") >= 2
        assert "verify_pre_pulse_race_arm_stayed_false" in commit
        assert "save_race_arm_false_watermarks" in commit
        assert "wait_race_arm_all true true" in commit
        assert "final post-arm vehicle/admin authority recheck failed" in commit
        assert commit.rindex("check_all_official_start_services") < commit.index(
            "call_official_start_all true"
        )
        assert commit.rindex("save_race_arm_false_watermarks") < commit.index(
            "call_official_start_all true"
        )
        assert 'deadline_has_budget "${rollback_reserve_sec}"' in commit
        assert commit.index('deadline_has_budget "${rollback_reserve_sec}"') < commit.index(
            "call_official_start_all true"
        )


def test_shared_monotonic_deadline_bounds_helper_phases_and_rejects_duplicate_domains() -> None:
    for helper in (START_HELPER, INSTALLED_SOURCE_HELPER):
        source = helper.read_text(encoding="utf-8")
        assert 'gate_deadline_monotonic_ns="${AIC_GATE_DEADLINE_MONOTONIC_NS:-}"' in source
        assert "time.monotonic_ns()" in source
        assert "bounded_operation_timeout()" in source
        assert "deadline_has_budget()" in source
        assert "rollback_reserve_sec=" in source
        assert "pulse_rollback_reserve_sec=" in source
        assert "kill_after_grace_sec=2" in source
        assert "duplicate vehicle domain in AWSIM_READY_DOMAINS" in source
        assert 'while deadline_has_budget 0; do' in source
        first_observer_start = source.index("if ! start_admin_observer; then")
        reserve_assignment = source.index(
            'required_failure_reserve_sec="${observer_reap_reserve_sec}"'
        )
        reserve_check = source.index(
            'if ! deadline_has_budget "${observer_reap_reserve_sec}"; then'
        )
        assert reserve_assignment < reserve_check < first_observer_start
        retained_start = source[
            source.index("    start)\n", source.index("while deadline_has_budget 0; do")) :
            source.index("    lapcomplete)\n", source.index("while deadline_has_budget 0; do"))
        ]
        assert "without a helper-owned Ready/WaitStart one-shot transition" in retained_start
        assert "exit 1" in retained_start
        assert "causal disarm precondition is impossible" in source


def test_pre_pulse_service_preflight_failure_is_terminal_and_bounded() -> None:
    for helper in (START_HELPER, INSTALLED_SOURCE_HELPER):
        source = helper.read_text(encoding="utf-8")
        service_check = _function_body(source, "check_official_start_service")
        assert "bounded_operation_timeout" in service_check
        assert "official_start_call_timeout_sec" in service_check
        assert "reserve_sec" in service_check
        assert "kill_after_grace_sec" in service_check

        publisher = _function_body(source, "publish_official_one_shot_start")
        assert "check_all_official_start_services" in publisher
        assert "startup_service_preflight_failed=true" in publisher
        assert "terminal" in publisher

        main_loop = source.split("while deadline_has_budget 0; do", 1)[1]
        assert '[ "${startup_service_preflight_failed}" = true ]' in main_loop


def test_production_reserve_is_parallel_global_and_start_pulses_preserve_it() -> None:
    for helper in (START_HELPER, INSTALLED_SOURCE_HELPER):
        source = helper.read_text(encoding="utf-8")
        assert "service_false_bound_sec=" in source
        assert 'official_start_rollback_timeout_sec="${AWSIM_OFFICIAL_START_ROLLBACK_TIMEOUT_SEC:-5}"' in source
        assert "service_false_bound_sec=$((official_start_rollback_timeout_sec + kill_after_grace_sec))" in source
        assert "rollback_domain_bound_sec=$((service_false_bound_sec + race_arm_timeout_sec))" in source
        assert "rollback_parallel_bound_sec=" in source
        assert 'global_reap_timeout_sec="${kill_after_grace_sec}"' in source
        assert "subreaper_drain_reserve_sec=$((kill_after_grace_sec + 1))" in source
        assert "observer_reap_reserve_sec=$((global_reap_timeout_sec + subreaper_drain_reserve_sec))" in source
        assert "rollback_reserve_sec=$((rollback_parallel_bound_sec + observer_reap_reserve_sec + 1))" in source
        assert "pulse_rollback_reserve_sec=$((pulse_parallel_bound_sec + observer_reap_reserve_sec + 1))" in source
        bounded = _function_body(source, "bounded_operation_timeout")
        assert 'local kill_grace_sec="${3:-0}"' in bounded
        assert "- kill_grace" in bounded
        for function_name in (
            "publish_start",
            "wait_for_one_shot_start_barrier",
            "publish_official_one_shot_start",
        ):
            body = _function_body(source, function_name)
            assert "pulse_rollback_reserve_sec" in body
        rollback = _function_body(source, "rollback_official_start")
        assert rollback.index("rollback_official_start_domain") < rollback.index(
            'for pid in "${pids[@]}"'
        )
        assert rollback.index("publish_reset") < rollback.index(
            'for pid in "${pids[@]}"'
        )
        assert "wait_race_arm_all false" not in rollback
        domain_rollback = _function_body(source, "rollback_official_start_domain")
        assert domain_rollback.index("call_official_start_domain") < domain_rollback.index(
            "wait_race_arm_domain_after_monotonic"
        )
        causal_wait = _function_body(source, "wait_race_arm_domain_after_monotonic")
        assert '"${arm_monotonic_ns}" -gt "${dispatch_monotonic_ns}"' in causal_wait


def test_startup_race_arm_uses_dedicated_three_state_bounded_wait() -> None:
    for helper in (START_HELPER, INSTALLED_SOURCE_HELPER):
        source = helper.read_text(encoding="utf-8")
        startup_wait = _function_body(source, "wait_for_initial_race_arm_false")
        assert "false)" in startup_wait
        assert "true)" in startup_wait
        assert "empty)" in startup_wait
        assert 'local initial_deadline=$((SECONDS + race_arm_timeout_sec))' in startup_wait
        assert 'deadline_has_budget "${observer_reap_reserve_sec}"' in startup_wait
        assert 'race_arm_observer_snapshot "${domain}"' in startup_wait
        assert "race_arm_observer_startup_watermark_seqs" in startup_wait
        assert "race_arm_observer_seen_true" in startup_wait
        assert "race_arm_observer_saw_forbidden_state" in startup_wait
        assert "startup race-arm history observed forbidden true" in startup_wait
        assert "startup race-arm evidence is missing" in startup_wait
        assert "observer exited or emitted invalid/missing evidence" in startup_wait
        assert "wait_race_arm_all" not in startup_wait
        call_index = source.index("if ! wait_for_initial_race_arm_false; then")
        main_loop_index = source.index("while deadline_has_budget 0; do")
        assert call_index < main_loop_index
        snapshot = _function_body(source, "race_arm_observer_snapshot")
        assert "--history-watermark-seq" in snapshot
        assert "--forbid-state-after-watermark" in snapshot
        assert "seen_true saw_forbidden_state" in snapshot
    observer = RACE_ARM_OBSERVER.read_text(encoding="utf-8")
    assert 'parser.add_argument("--history-watermark-seq", type=int)' in observer
    assert 'parser.add_argument("--forbid-state-after-watermark"' in observer
    assert "seen_true_after_history_watermark" in observer
    assert "saw_forbidden_state" in observer


def test_helper_owned_descendants_are_reaped_by_global_process_group_bound() -> None:
    source = INSTALLED_SOURCE_HELPER.read_text(encoding="utf-8")
    launch = _function_body(source, "launch_process_group")
    terminate = _function_body(source, "terminate_and_reap_process_groups")
    stop_observers = _function_body(source, "stop_all_observers")
    assert "set -m" in launch and '"$@" &' in launch and "set +m" in launch
    assert 'kill -TERM -- "-${pid}"' in terminate
    assert 'kill -KILL -- "-${pid}"' in terminate
    assert terminate.count('wait "${pid}"') == 1
    assert stop_observers.count("terminate_and_reap_process_groups") == 1
    assert 'AWSIM_HELPER_REAPER_ACTIVE:-' in source
    assert 'exec /usr/bin/python3 "${script_dir}/helper_process_reaper.py"' in source
    assert '--deadline-monotonic-ns "${AIC_GATE_DEADLINE_MONOTONIC_NS:-0}"' in source
    reaper = HELPER_REAPER.read_text(encoding="utf-8")
    assert "PR_SET_CHILD_SUBREAPER = 36" in reaper
    assert "PR_SET_PDEATHSIG = 1" in reaper
    assert "os.waitpid(-1, os.WNOHANG)" in reaper
    assert 'parser.add_argument("--deadline-monotonic-ns", type=int, default=0)' in reaper
    assert "deadline_monotonic_ns" in reaper
    assert "shutdown_start -= grace_sec + 1.0" in reaper
    assert "deadline_shutdown_started" in reaper


def test_race_arm_observers_remain_alive_until_rollback_finishes() -> None:
    for helper_path in (START_HELPER, INSTALLED_SOURCE_HELPER):
        source = helper_path.read_text(encoding="utf-8")
        for handler_name in ("handle_start_helper_signal", "handle_start_helper_exit"):
            handler = _function_body(source, handler_name)
            rollback_index = min(
                index
                for marker in ("rollback_official_start", "rollback_start_pulse")
                if (index := handler.find(marker)) >= 0
            )
            assert rollback_index < handler.index("stop_all_observers")
        cleanup = _function_body(source, "cleanup_observer_dir")
        assert '"/tmp/${expected_prefix}."??????' in cleanup
        assert "rm -rf" not in cleanup
        assert 'rm -f -- "${entry}"' in cleanup
        assert 'rmdir -- "${observer_dir}"' in cleanup
        assert source.index("terminate_and_reap_process_groups 0") < source.index(
            'cleanup_observer_dir "${admin_observer_dir}"'
        )


def test_canonical_and_headless_start_helpers_match() -> None:
    assert START_HELPER.read_bytes() == INSTALLED_SOURCE_HELPER.read_bytes()
    assert RACE_ARM_OBSERVER.read_bytes() == HEADLESS_RACE_ARM_OBSERVER.read_bytes()
    assert (
        VEHICLE_READINESS_OBSERVER.read_bytes()
        == HEADLESS_VEHICLE_READINESS_OBSERVER.read_bytes()
    )
    assert (
        AUTOSTART_ORCHESTRATOR.read_bytes()
        == HEADLESS_AUTOSTART_ORCHESTRATOR.read_bytes()
    )
    assert SUPERVISOR.read_bytes() == HEADLESS_SUPERVISOR.read_bytes()
    assert HELPER_REAPER.read_bytes() == HEADLESS_HELPER_REAPER.read_bytes()
    assert (
        REPO_ROOT / "aichallenge/admin_state_observer.py"
    ).read_bytes() == (
        REPO_ROOT / "tools/scripts/headless_overrides/aichallenge/admin_state_observer.py"
    ).read_bytes()


def test_race_arm_observer_is_wired_into_image_and_headless_manifests() -> None:
    dockerfile = (REPO_ROOT / "Dockerfile").read_text(encoding="utf-8")
    assert (
        "COPY aichallenge/race_arm_observer.py /aichallenge/race_arm_observer.py"
        in dockerfile
    )
    assert "chmod +x /aichallenge/race_arm_observer.py" in dockerfile
    assert (
        "COPY aichallenge/vehicle_readiness_observer.py "
        "/aichallenge/vehicle_readiness_observer.py"
        in dockerfile
    )
    assert "chmod +x /aichallenge/vehicle_readiness_observer.py" in dockerfile
    assert "COPY aichallenge/helper_process_reaper.py /aichallenge/helper_process_reaper.py" in dockerfile
    assert "chmod +x /aichallenge/helper_process_reaper.py" in dockerfile
    fingerprint = (REPO_ROOT / "aichallenge/capture_run_fingerprint.py").read_text(
        encoding="utf-8"
    )
    headless_fingerprint = (
        REPO_ROOT
        / "tools/scripts/headless_overrides/aichallenge/capture_run_fingerprint.py"
    ).read_text(encoding="utf-8")
    assert fingerprint == headless_fingerprint
    assert (
        '"admin_state_observer": "aichallenge/admin_state_observer.py"'
        in fingerprint
    )
    assert '"race_arm_observer": "aichallenge/race_arm_observer.py"' in fingerprint
    assert (
        '"vehicle_readiness_observer": "aichallenge/vehicle_readiness_observer.py"'
        in fingerprint
    )
    assert '"helper_process_reaper": "aichallenge/helper_process_reaper.py"' in fingerprint
    for manifest_name in ("manifest.txt", "manifest.2026-full.txt"):
        manifest = (
            REPO_ROOT / "tools/scripts/headless_overrides" / manifest_name
        ).read_text(encoding="utf-8")
        assert "aichallenge/race_arm_observer.py" in manifest.splitlines()
        assert "aichallenge/vehicle_readiness_observer.py" in manifest.splitlines()
        assert "aichallenge/helper_process_reaper.py" in manifest.splitlines()

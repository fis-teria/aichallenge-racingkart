from __future__ import annotations

import ast
import json
import math
import os
import re
import shutil
import stat
import subprocess
from pathlib import Path

import pytest

from aichallenge import capture_run_fingerprint as fingerprint


def _raw_compose_service_for_normalization() -> dict[str, object]:
    return {
        "image": "aichallenge-2025-eval@sha256:" + "a" * 64,
        "command": ["bash", "-lc", "exec sleep infinity"],
        "entrypoint": None,
        "environment": {"RUN_KIND": "planner-pp-control-smoke"},
        "volumes": [{"type": "bind", "source": "/tmp/input", "target": "/input"}],
        "privileged": False,
        "network_mode": "host",
        "security_opt": [],
        "cap_add": [],
        "read_only": False,
        "devices": [],
        "working_dir": None,
        "stop_signal": None,
        "stop_grace_period": None,
        "pull_policy": "never",
    }


def test_gate2_raw_compose_volumes_normalize_to_mounts_and_raw_mounts_fail() -> None:
    raw = _raw_compose_service_for_normalization()
    normalized = fingerprint._normalize_compose_service(raw)
    assert normalized["mounts"] == [
        '{"source":"/tmp/input","target":"/input","type":"bind"}'
    ]

    raw_with_normalized_key = dict(raw)
    raw_with_normalized_key["mounts"] = []
    with pytest.raises(fingerprint.FingerprintError, match="unknown keys"):
        fingerprint._normalize_compose_service(raw_with_normalized_key)


@pytest.mark.skipif(shutil.which("docker") is None, reason="docker is unavailable")
def test_gate2_repository_compose_renders_two_target_services() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    launch_spec = fingerprint._rendered_launch_spec(repo_root)
    assert launch_spec["service_set"] == [
        "autoware-eval-command", "autoware-eval-runtime",
    ]
    assert set(launch_spec["services"]) == set(launch_spec["service_set"])
    for service in launch_spec["services"].values():
        assert set(service) == fingerprint.GATE2_NORMALIZED_SERVICE_KEYS
        assert service["mounts"]


def _fake_snapshot(value: str) -> tuple[dict, list[dict]]:
    return (
        {
            "schema_version": 2,
            "captured_at": "2026-07-20T00:00:00+09:00",
            "launch_context": {"plain": {"RUN_KIND": "gate2"}, "hashed": {}},
            "fingerprints": {
                "autonomy_artifact_sha256": value,
                "launch_config_sha256": value,
                "harness_sha256": value,
                "simulator_sha256": value,
                "submission_sha256": value,
                "experiment_sha256": value,
            },
        },
        [
            {
                "path": "aichallenge/workspace/src/pkg/node.py",
                "sha256": value,
            }
        ],
    )


def test_source_tree_includes_untracked_inputs_and_excludes_generated_logs(tmp_path: Path) -> None:
    source = tmp_path / "aichallenge/workspace/src"
    tracked = source / "pkg/src/node.cpp"
    untracked = source / "pkg/src/new_builder.cpp"
    generated = source / "pkg/log/runtime.log"
    tracked.parent.mkdir(parents=True)
    generated.parent.mkdir(parents=True)
    tracked.write_text("tracked\n", encoding="utf-8")
    untracked.write_text("untracked\n", encoding="utf-8")
    generated.write_text("generated\n", encoding="utf-8")

    records = fingerprint.collect_source_records(tmp_path)
    paths = {record["path"] for record in records}

    assert "aichallenge/workspace/src/pkg/src/node.cpp" in paths
    assert "aichallenge/workspace/src/pkg/src/new_builder.cpp" in paths
    assert "aichallenge/workspace/src/pkg/log/runtime.log" not in paths


def test_source_tree_hash_changes_for_content_and_symlink_target(tmp_path: Path) -> None:
    source = tmp_path / "aichallenge/workspace/src/pkg"
    source.mkdir(parents=True)
    node = source / "node.py"
    link = source / "active.py"
    node.write_text("first\n", encoding="utf-8")
    link.symlink_to("node.py")

    initial = fingerprint._canonical_sha256(fingerprint.collect_source_records(tmp_path))
    node.write_text("second\n", encoding="utf-8")
    content_changed = fingerprint._canonical_sha256(
        fingerprint.collect_source_records(tmp_path)
    )
    link.unlink()
    link.symlink_to("other.py")
    link_changed = fingerprint._canonical_sha256(fingerprint.collect_source_records(tmp_path))

    assert initial != content_changed
    assert content_changed != link_changed


def test_launch_context_uses_allowlist_and_hashes_free_form_arguments() -> None:
    context = fingerprint.collect_launch_context(
        {
            "RUN_KIND": "gate2",
            "AWSIM_VEHICLES": "1",
            "AUTOWARE_RUN_MODE": "awsim-no-viz",
            "AUTOSTART_DEBUG_VISUALIZATION": "false",
            "AWSIM_EXTRA_ARGS": "--scenario SafetyGate/scenario2.yaml",
            "NTRIP_PASSWORD": "must-not-be-recorded",
        }
    )

    encoded = json.dumps(context)
    assert context["plain"]["RUN_KIND"] == "gate2"
    assert context["plain"]["AWSIM_VEHICLES"] == "1"
    assert context["plain"]["AUTOWARE_RUN_MODE"] == "awsim-no-viz"
    assert context["plain"]["AUTOSTART_DEBUG_VISUALIZATION"] == "false"
    assert context["hashed"]["AWSIM_EXTRA_ARGS"]["present"] is True
    assert "SafetyGate" not in encoded
    assert "must-not-be-recorded" not in encoded
    assert "NTRIP_PASSWORD" not in encoded


def test_prelaunch_capture_is_atomic_and_refuses_run_reuse(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        fingerprint,
        "build_stable_snapshot",
        lambda _repo, _context: _fake_snapshot("a" * 64),
    )
    monkeypatch.setattr(fingerprint, "assert_runtime_stopped", lambda _repo: None)
    output_root = tmp_path / "output"
    run_dir = output_root / "run"
    environment = {
        "RUN_KIND": "gate2",
        "OUTPUT_HOST_ROOT": str(output_root),
        "OUTPUT_ROOT": "/output",
    }

    manifest = fingerprint.capture_prelaunch(
        tmp_path, output_root, run_dir, environment
    )

    assert manifest == run_dir / "provenance/prelaunch-manifest.json"
    assert manifest.is_file()
    assert (run_dir / "provenance/source-tree.sha256").is_file()
    assert (run_dir / "provenance/artifact-fingerprint.sha256").is_file()
    assert not list(run_dir.glob(".provenance-*"))
    with pytest.raises(fingerprint.FingerprintError, match="non-empty run directory"):
        fingerprint.capture_prelaunch(
            tmp_path, output_root, run_dir, environment
        )


def test_postrun_verification_writes_result_and_detects_change(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    snapshots = iter((_fake_snapshot("a" * 64), _fake_snapshot("b" * 64)))
    monkeypatch.setattr(
        fingerprint, "build_stable_snapshot", lambda _repo, _context: next(snapshots)
    )
    monkeypatch.setattr(fingerprint, "assert_runtime_stopped", lambda _repo: None)
    output_root = tmp_path / "output"
    run_dir = output_root / "run"
    environment = {
        "RUN_KIND": "gate2",
        "OUTPUT_HOST_ROOT": str(output_root),
        "OUTPUT_ROOT": "/output",
    }
    fingerprint.capture_prelaunch(
        tmp_path, output_root, run_dir, environment
    )

    verification_path, verified = fingerprint.verify_postrun(
        tmp_path, output_root, run_dir
    )

    verification = json.loads(verification_path.read_text(encoding="utf-8"))
    assert verified is False
    assert verification["verified"] is False
    assert any(not item["match"] for item in verification["comparisons"].values())
    assert (run_dir / "provenance/postrun-manifest.json").is_file()


def test_missing_required_artifact_fails_closed(tmp_path: Path) -> None:
    with pytest.raises(fingerprint.FingerprintError, match="required artifact is missing"):
        fingerprint.collect_named_paths(tmp_path, {"planner": "missing/planner_node"})


def test_runtime_tree_hash_changes_and_excludes_python_cache(tmp_path: Path) -> None:
    tree = tmp_path / "runtime"
    cache = tree / "pkg/__pycache__/module.cpython-310.pyc"
    native = tree / "pkg/controller.so"
    cache.parent.mkdir(parents=True)
    native.write_bytes(b"first")
    cache.write_bytes(b"generated")

    first = fingerprint.collect_tree_records(tmp_path, "runtime")
    native.write_bytes(b"second")
    second = fingerprint.collect_tree_records(tmp_path, "runtime")

    assert first["sha256"] != second["sha256"]
    assert all("__pycache__" not in item["path"] for item in second["records"])


def test_run_directory_must_be_safe_direct_output_child(tmp_path: Path) -> None:
    output_root = tmp_path / "output"
    expected_root, expected_run = fingerprint.validate_run_location(
        output_root, output_root / "run-001"
    )
    assert expected_root == output_root.resolve()
    assert expected_run == (output_root / "run-001").resolve()

    with pytest.raises(fingerprint.FingerprintError, match="direct child"):
        fingerprint.validate_run_location(output_root, output_root / "nested/run")
    with pytest.raises(fingerprint.FingerprintError, match="reserved run id"):
        fingerprint.validate_run_location(output_root, output_root / "latest")
    output_root.mkdir()
    real_run = output_root / "real-run"
    real_run.mkdir()
    linked_run = output_root / "linked-run"
    linked_run.symlink_to(real_run, target_is_directory=True)
    with pytest.raises(fingerprint.FingerprintError, match="must not be a symlink"):
        fingerprint.validate_run_location(output_root, linked_run)


def test_prepare_domain_artifact_dirs_creates_host_owned_writable_domains(
    tmp_path: Path,
) -> None:
    run_dir = tmp_path / "output/run"
    run_dir.mkdir(parents=True)

    created = fingerprint.prepare_domain_artifact_dirs(run_dir, "1,2,3")

    assert created == [run_dir / "d1", run_dir / "d2", run_dir / "d3"]
    for domain_dir in created:
        domain_stat = domain_dir.stat()
        assert domain_stat.st_uid == fingerprint.os.geteuid()
        assert domain_stat.st_gid == fingerprint.os.getegid()
        assert stat.S_IMODE(domain_stat.st_mode) == 0o775


@pytest.mark.parametrize("domains", ["", "0", "1,,2", "1,1", "1,5", "one"])
def test_prepare_domain_artifact_dirs_rejects_invalid_domains(
    tmp_path: Path, domains: str
) -> None:
    run_dir = tmp_path / "output/run"
    run_dir.mkdir(parents=True)

    with pytest.raises(fingerprint.FingerprintError):
        fingerprint.prepare_domain_artifact_dirs(run_dir, domains)
    assert list(run_dir.iterdir()) == []


@pytest.mark.parametrize("existing_kind", ["directory", "file", "symlink"])
def test_prepare_domain_artifact_dirs_refuses_existing_path(
    tmp_path: Path, existing_kind: str
) -> None:
    run_dir = tmp_path / "output/run"
    run_dir.mkdir(parents=True)
    domain_dir = run_dir / "d1"
    if existing_kind == "directory":
        domain_dir.mkdir()
    elif existing_kind == "file":
        domain_dir.write_text("occupied\n", encoding="utf-8")
    else:
        domain_dir.symlink_to(run_dir / "missing-target", target_is_directory=True)

    with pytest.raises(fingerprint.FingerprintError, match="existing domain"):
        fingerprint.prepare_domain_artifact_dirs(run_dir, "1")


def test_prelaunch_domain_creation_failure_is_fail_closed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setattr(
        fingerprint,
        "build_stable_snapshot",
        lambda _repo, _context: _fake_snapshot("a" * 64),
    )
    monkeypatch.setattr(fingerprint, "assert_runtime_stopped", lambda _repo: None)
    output_root = tmp_path / "output"
    run_dir = output_root / "run"
    monkeypatch.setenv("OUTPUT_HOST_ROOT", str(output_root))
    monkeypatch.setenv("OUTPUT_ROOT", "/output")
    original_mkdir = Path.mkdir

    def fail_d2_mkdir(
        path: Path,
        mode: int = 0o777,
        parents: bool = False,
        exist_ok: bool = False,
    ) -> None:
        if path == run_dir / "d2":
            raise OSError("injected d2 creation failure")
        original_mkdir(path, mode=mode, parents=parents, exist_ok=exist_ok)

    monkeypatch.setattr(Path, "mkdir", fail_d2_mkdir)
    argv = [
        "--repo-root",
        str(tmp_path),
        "--output-root",
        str(output_root),
        "--run-dir",
        str(run_dir),
        "--prepare-domains",
        "1,2,3",
    ]

    assert fingerprint.main(argv) == 1
    first_error = capsys.readouterr().err
    assert "failed to create domain artifact directory" in first_error
    assert (run_dir / "provenance/prelaunch-manifest.json").is_file()
    assert (run_dir / "d1").is_dir()
    assert not (run_dir / "d2").exists()

    assert fingerprint.main(argv) == 1
    retry_error = capsys.readouterr().err
    assert "refusing to reuse non-empty run directory" in retry_error


def test_output_mapping_requires_matching_host_and_scoped_container_root(
    tmp_path: Path,
) -> None:
    output_root = tmp_path / "output"
    assert fingerprint.validate_output_mapping(
        output_root,
        {"OUTPUT_HOST_ROOT": str(output_root), "OUTPUT_ROOT": "/output"},
    ) == "/output"

    with pytest.raises(fingerprint.FingerprintError, match="does not match"):
        fingerprint.validate_output_mapping(
            output_root,
            {"OUTPUT_HOST_ROOT": str(tmp_path / "other"), "OUTPUT_ROOT": "/output"},
        )
    with pytest.raises(fingerprint.FingerprintError, match="absolute"):
        fingerprint.validate_output_mapping(
            output_root,
            {"OUTPUT_HOST_ROOT": str(output_root), "OUTPUT_ROOT": "relative"},
        )


def test_runtime_guard_rejects_existing_compose_container(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def fake_run(command: list[str], _cwd: Path) -> bytes:
        return b"container-id\n" if command == ["docker", "compose", "ps", "-aq"] else b""

    monkeypatch.setattr(fingerprint, "_run_command", fake_run)
    with pytest.raises(fingerprint.FingerprintError, match="requires stopped"):
        fingerprint.assert_runtime_stopped(tmp_path)


def test_rosbag_run_requires_finalized_metadata(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output_root = tmp_path / "output"
    run_dir = output_root / "run"
    provenance = run_dir / "provenance"
    provenance.mkdir(parents=True)
    prelaunch, _ = _fake_snapshot("a" * 64)
    prelaunch["launch_context"]["plain"]["ROSBAG"] = "true"
    (provenance / "prelaunch-manifest.json").write_text(
        json.dumps(prelaunch), encoding="utf-8"
    )
    monkeypatch.setattr(fingerprint, "assert_runtime_stopped", lambda _repo: None)

    with pytest.raises(fingerprint.FingerprintError, match="rosbag metadata"):
        fingerprint.verify_postrun(tmp_path, output_root, run_dir)


def test_rosbag_metadata_requires_nonempty_referenced_storage(tmp_path: Path) -> None:
    run_dir = tmp_path / "run"
    bag_dir = run_dir / "d1/rosbag2_autoware"
    bag_dir.mkdir(parents=True)
    (bag_dir / "metadata.yaml").write_text(
        """rosbag2_bagfile_information:
  message_count: 12
  relative_file_paths:
    - rosbag2_autoware_0.db3
""",
        encoding="utf-8",
    )
    storage = bag_dir / "rosbag2_autoware_0.db3"
    storage.write_bytes(b"sqlite")

    assert fingerprint.validate_finalized_rosbags(run_dir) == [
        "d1/rosbag2_autoware/metadata.yaml"
    ]
    storage.write_bytes(b"")
    with pytest.raises(fingerprint.FingerprintError, match="missing or empty"):
        fingerprint.validate_finalized_rosbags(run_dir)


def test_stable_snapshot_rejects_full_input_change(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    snapshots = iter((_fake_snapshot("a" * 64), _fake_snapshot("b" * 64)))
    monkeypatch.setattr(fingerprint, "build_snapshot", lambda _repo, _context: next(snapshots))
    with pytest.raises(fingerprint.FingerprintError, match="runtime inputs changed"):
        fingerprint.build_stable_snapshot(tmp_path, {})


def test_postrun_verification_accepts_identical_snapshot(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        fingerprint,
        "build_stable_snapshot",
        lambda _repo, _context: _fake_snapshot("a" * 64),
    )
    monkeypatch.setattr(fingerprint, "assert_runtime_stopped", lambda _repo: None)
    output_root = tmp_path / "output"
    run_dir = output_root / "run"
    environment = {
        "RUN_KIND": "gate2",
        "OUTPUT_HOST_ROOT": str(output_root),
        "OUTPUT_ROOT": "/output",
    }
    fingerprint.capture_prelaunch(tmp_path, output_root, run_dir, environment)

    _, verified = fingerprint.verify_postrun(tmp_path, output_root, run_dir)

    assert verified is True


def test_make_and_headless_override_wiring_remain_identical() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    makefile = (repo_root / "Makefile").read_text(encoding="utf-8")
    override_makefile = (
        repo_root / "tools/scripts/headless_overrides/Makefile"
    ).read_text(encoding="utf-8")
    docker_compose = (repo_root / "docker-compose.yml").read_bytes()
    override_docker_compose = (
        repo_root / "tools/scripts/headless_overrides/docker-compose.yml"
    ).read_bytes()
    helper = (repo_root / "aichallenge/capture_run_fingerprint.py").read_bytes()
    override_helper = (
        repo_root
        / "tools/scripts/headless_overrides/aichallenge/capture_run_fingerprint.py"
    ).read_bytes()
    watchdog = (repo_root / "aichallenge/d1_start_progress_watchdog.py").read_bytes()
    override_watchdog = (
        repo_root
        / "tools/scripts/headless_overrides/aichallenge/d1_start_progress_watchdog.py"
    ).read_bytes()
    supervisor = (
        repo_root / "aichallenge/run_awsim_with_d1_watchdog.bash"
    ).read_bytes()
    override_supervisor = (
        repo_root
        / "tools/scripts/headless_overrides/aichallenge/run_awsim_with_d1_watchdog.bash"
    ).read_bytes()
    start_helper = (repo_root / "aichallenge/request_awsim_start.bash").read_bytes()
    override_start_helper = (
        repo_root
        / "tools/scripts/headless_overrides/aichallenge/request_awsim_start.bash"
    ).read_bytes()
    admin_observer = (repo_root / "aichallenge/admin_state_observer.py").read_bytes()
    override_admin_observer = (
        repo_root
        / "tools/scripts/headless_overrides/aichallenge/admin_state_observer.py"
    ).read_bytes()
    race_arm_observer = (repo_root / "aichallenge/race_arm_observer.py").read_bytes()
    override_race_arm_observer = (
        repo_root
        / "tools/scripts/headless_overrides/aichallenge/race_arm_observer.py"
    ).read_bytes()
    vehicle_readiness_observer = (
        repo_root / "aichallenge/vehicle_readiness_observer.py"
    ).read_bytes()
    override_vehicle_readiness_observer = (
        repo_root
        / "tools/scripts/headless_overrides/aichallenge/vehicle_readiness_observer.py"
    ).read_bytes()
    autostart_orchestrator = (
        repo_root
        / "aichallenge/workspace/src/aichallenge_system/autostart_orchestrator_py/"
        "autostart_orchestrator_py/autostart_orchestrator_node.py"
    ).read_bytes()
    override_autostart_orchestrator = (
        repo_root
        / "tools/scripts/headless_overrides/aichallenge/workspace/src/"
        "aichallenge_system/autostart_orchestrator_py/autostart_orchestrator_py/"
        "autostart_orchestrator_node.py"
    ).read_bytes()
    service_waiter = (
        repo_root / "aichallenge/wait_for_typed_service.py"
    ).read_bytes()
    override_service_waiter = (
        repo_root
        / "tools/scripts/headless_overrides/aichallenge/wait_for_typed_service.py"
    ).read_bytes()

    assert makefile == override_makefile
    assert docker_compose == override_docker_compose
    assert helper == override_helper
    assert watchdog == override_watchdog
    assert supervisor == override_supervisor
    assert start_helper == override_start_helper
    assert admin_observer == override_admin_observer
    assert race_arm_observer == override_race_arm_observer
    assert vehicle_readiness_observer == override_vehicle_readiness_observer
    assert autostart_orchestrator == override_autostart_orchestrator
    assert service_waiter == override_service_waiter
    for relative_path in (
        "hybrid_control_mux/hybrid_control_mux/core.py",
        "hybrid_control_mux/hybrid_control_mux/hybrid_control_mux_node.py",
        "hybrid_control_mux/config/hybrid_control_mux.param.yaml",
        "hybrid_control_mux/config/pure_pursuit_mpc_horizon.param.yaml",
        "hybrid_control_mux/launch/hybrid_control_mux.launch.xml",
        "multi_purpose_mpc_ros_msgs/CMakeLists.txt",
        "multi_purpose_mpc_ros_msgs/package.xml",
        "multi_purpose_mpc_ros_msgs/msg/ControllerCommandEnvelope.msg",
        "multi_purpose_mpc_ros_msgs/msg/MotionAuthorityGrant.msg",
        "multi_purpose_mpc_ros_msgs/msg/PlanSampleKey.msg",
        "overtake_planner/src/overtake_planner_node.cpp",
    ):
        source = (
            repo_root
            / "aichallenge/workspace/src/aichallenge_submit"
            / relative_path
        ).read_bytes()
        override = (
            repo_root
            / "tools/scripts/headless_overrides/aichallenge/workspace/src/"
            "aichallenge_submit"
            / relative_path
        ).read_bytes()
        assert source == override
    assert "autoware-simulator: capture-run-fingerprint" in makefile
    assert "simulator: capture-run-fingerprint" in makefile
    assert 'RUN_ID="$(RUN_ID)" RUN_KIND="$@"' in makefile
    assert makefile.count("$(MAKE) $(AWSIM_START_TARGET) \\") == 2
    assert makefile.count('RUN_ID="$(RUN_ID)" RUN_KIND=') >= 3
    assert "AWSIM_Data/StreamingAssets" in fingerprint.SIMULATOR_TREE_INPUTS["streaming_assets"]
    assert "fingerprint_recorder" in fingerprint.HARNESS_INPUTS
    assert "d1_start_progress_watchdog" in fingerprint.HARNESS_INPUTS
    assert "wait_for_typed_service" in fingerprint.HARNESS_INPUTS
    assert "admin_state_observer" in fingerprint.HARNESS_INPUTS
    assert "race_arm_observer" in fingerprint.HARNESS_INPUTS
    assert "vehicle_readiness_observer" in fingerprint.HARNESS_INPUTS
    assert "D1_STALL_TIMEOUT_SEC" in fingerprint.PLAINTEXT_ENV_KEYS
    assert "mpc_python_environment" in fingerprint.RUNTIME_TREE_INPUTS
    for manifest_name in ("manifest.txt", "manifest.2026-full.txt"):
        manifest = (
            repo_root / "tools/scripts/headless_overrides" / manifest_name
        ).read_text(encoding="utf-8")
        assert "aichallenge/capture_run_fingerprint.py" in manifest.splitlines()
        assert "aichallenge/d1_start_progress_watchdog.py" in manifest.splitlines()
        assert "aichallenge/run_awsim_with_d1_watchdog.bash" in manifest.splitlines()
        assert "aichallenge/request_awsim_start.bash" in manifest.splitlines()
        assert "aichallenge/admin_state_observer.py" in manifest.splitlines()
        assert "aichallenge/race_arm_observer.py" in manifest.splitlines()
        assert "aichallenge/vehicle_readiness_observer.py" in manifest.splitlines()
        assert "aichallenge/wait_for_typed_service.py" in manifest.splitlines()
        for relative_path in (
            "config/hybrid_control_mux.param.yaml",
            "config/pure_pursuit_mpc_horizon.param.yaml",
            "hybrid_control_mux/core.py",
            "hybrid_control_mux/hybrid_control_mux_node.py",
            "launch/hybrid_control_mux.launch.xml",
        ):
            assert (
                "aichallenge/workspace/src/aichallenge_submit/"
                f"hybrid_control_mux/{relative_path}"
                in manifest.splitlines()
            )
        for relative_path in (
            "CMakeLists.txt",
            "package.xml",
            "msg/ControllerCommandEnvelope.msg",
            "msg/MotionAuthorityGrant.msg",
            "msg/PlanSampleKey.msg",
        ):
            assert (
                "aichallenge/workspace/src/aichallenge_submit/"
                f"multi_purpose_mpc_ros_msgs/{relative_path}"
                in manifest.splitlines()
            )
        assert (
            "aichallenge/workspace/src/aichallenge_submit/"
            "overtake_planner/src/overtake_planner_node.cpp"
            in manifest.splitlines()
        )


@pytest.mark.parametrize("target", ["dev3", "gate2"])
def test_make_dry_run_has_no_output_side_effect(tmp_path: Path, target: str) -> None:
    repo_root = Path(__file__).resolve().parents[3]
    output_root = tmp_path / "output"
    result = subprocess.run(
        [
            "make",
            "-n",
            target,
            "RUN_ID=dry-run",
            f"OUTPUT_HOST_ROOT={output_root}",
        ],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    assert not output_root.exists()
    assert result.stdout.count("python3 aichallenge/capture_run_fingerprint.py") == 1
    assert result.stdout.count("--prepare-domains") == 1
    assert result.stdout.index("--prepare-domains") < result.stdout.index("Start AWSIM")
    assert "awsim-request-start-and-watch-d1" in result.stdout
    assert result.stdout.count("run_awsim_with_d1_watchdog.bash") == 1
    assert 'D1_STALL_TIMEOUT_SEC="15"' in result.stdout
    assert 'D1_STALL_ENTER_SPEED_MPS="0.05"' in result.stdout
    assert 'D1_STALL_EXIT_SPEED_MPS="0.10"' in result.stdout
    recursive_run_ids = set(re.findall(r'RUN_ID="([^"]+)"', result.stdout))
    assert recursive_run_ids == {"dry-run"}
    assert f'RUN_HOST_DIR="{output_root}/dry-run"' in result.stdout
    if target == "gate2":
        assert "DEV_AUTO_START=false" in result.stdout
        assert "request_awsim_start.bash" not in result.stdout
        for expected in (
            'RUN_KIND="gate2"',
            'AWSIM_START_MODE="sync"',
            'AWSIM_VEHICLES="4"',
            'AWSIM_LAPS="unlimited"',
            'RUN_GATE_SCENARIO="SafetyGate/scenario2.yaml"',
            '--scenario /aichallenge/simulator/AWSIM/AWSIM_Data/StreamingAssets/SafetyGate/scenario2.yaml',
        ):
            assert expected in result.stdout
        assert "--safety-gate" not in result.stdout
        assert 'AWSIM_READY_DOMAINS="1"' in result.stdout
        assert '--prepare-domains "1"' in result.stdout
        assert 'AWSIM_READY_DOMAINS="1,2,3,4"' not in result.stdout
        assert "--prepare-domains \"1,2,3,4\"" not in result.stdout
    else:
        for expected in (
            'RUN_KIND="dev3"',
            'SIM_MODE="3p"',
            'AWSIM_START_MODE="sync"',
            'AWSIM_VEHICLES="3"',
            'AWSIM_LAPS="6"',
            'AWSIM_TIMEOUT="600"',
            'AWSIM_READY_DOMAINS="1,2,3"',
            '--prepare-domains "1,2,3"',
        ):
            assert expected in result.stdout


def test_gate2_scenario_preserves_static_opponents() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    scenario = (
        repo_root
        / "aichallenge/simulator/AWSIM/AWSIM_Data/StreamingAssets/"
        "SafetyGate/scenario2.yaml"
    ).read_text(encoding="utf-8")
    vehicle_section = scenario.split("vehicles:\n", 1)[1].split("objects:", 1)[0]
    vehicle_blocks = dict(
        re.findall(r'(?m)^  "([1-4])":\n((?:    .*\n)+)', vehicle_section)
    )

    assert set(vehicle_blocks) == {"1", "2", "3", "4"}
    assert "    at:" in vehicle_blocks["1"]
    assert "    static: true" not in vehicle_blocks["1"]
    for vehicle_id in ("1", "2", "3", "4"):
        coordinates = re.search(r"at: \[([^\]]+)\]", vehicle_blocks[vehicle_id])
        assert coordinates is not None
        position = [float(value.strip()) for value in coordinates.group(1).split(",")]
        assert len(position) == 2
        assert all(math.isfinite(value) for value in position)
    for opponent_id in ("2", "3", "4"):
        assert "    at:" in vehicle_blocks[opponent_id]
        assert "    static: true" in vehicle_blocks[opponent_id]


def test_planner_pp_control_smoke_is_the_only_gate2_live_spatial_opt_in(
    tmp_path: Path,
) -> None:
    repo_root = Path(__file__).resolve().parents[3]

    def render(target: str, *, hostile_opt_in: bool = False) -> str:
        environment = os.environ.copy()
        if hostile_opt_in:
            environment["PLANNER_PP_CONTROL_SMOKE_LIVE_SPATIAL"] = "true"
        result = subprocess.run(
            [
                "make",
                "-n",
                target,
                "RUN_ID=dry-run",
                f"OUTPUT_HOST_ROOT={tmp_path / target}",
            ],
            cwd=repo_root,
            env=environment,
            check=False,
            capture_output=True,
            text=True,
            timeout=30,
        )
        assert result.returncode == 0, result.stderr
        return result.stdout

    ordinary_gate1 = render("gate1")
    ordinary_gate2 = render("gate2")
    ordinary_gate3 = render("gate3")
    hostile_gate2 = render("gate2", hostile_opt_in=True)
    live_smoke = render("planner-pp-control-smoke")

    assert "STATE_LATTICE_EXACT_SPATIAL_FOLLOW_SHADOW_ENABLED=false" not in ordinary_gate2
    assert (
        'STATE_LATTICE_EXACT_SPATIAL_FOLLOW_SHADOW_ENABLED="true"'
        in ordinary_gate2
    )
    assert 'STATE_LATTICE_EXACT_SPATIAL_FOLLOW_SHADOW_ENABLED="true"' in hostile_gate2
    assert 'STATE_LATTICE_EXACT_SPATIAL_FOLLOW_SHADOW_ENABLED="false"' in live_smoke
    assert 'CONTROL_METHOD="state_lattice_pure_pursuit"' in live_smoke
    assert 'RUN_KIND="planner-pp-control-smoke"' in live_smoke
    assert 'PLANNER_PP_CONTROL_SMOKE_LIVE_SPATIAL="true"' in live_smoke
    assert "SafetyGate/scenario2.yaml" in live_smoke
    assert 'RUN_MODE="awsim-no-viz"' in ordinary_gate2
    assert 'AUTOSTART_DEBUG_VISUALIZATION="false"' in ordinary_gate2
    assert 'RUN_MODE="awsim-no-viz"' in live_smoke
    assert 'RUN_MODE="awsim"' in ordinary_gate1
    assert 'RUN_MODE="awsim"' in ordinary_gate3


@pytest.mark.parametrize(
    ("target", "scenario_name"),
    [
        ("gate1", "scenario1.yaml"),
        ("gate2", "scenario2.yaml"),
        ("gate3", "scenario3.yaml"),
    ],
)
def test_gate_scenario_uses_readable_absolute_container_path(
    tmp_path: Path, target: str, scenario_name: str
) -> None:
    repo_root = Path(__file__).resolve().parents[3]
    scenario = (
        repo_root
        / "aichallenge/simulator/AWSIM/AWSIM_Data/StreamingAssets/SafetyGate"
        / scenario_name
    )
    assert scenario.is_file()
    assert scenario.stat().st_size > 0

    result = subprocess.run(
        [
            "make",
            "-n",
            target,
            "RUN_ID=dry-run",
            f"OUTPUT_HOST_ROOT={tmp_path / 'output'}",
        ],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    absolute_arg = (
        "--scenario /aichallenge/simulator/AWSIM/AWSIM_Data/StreamingAssets/"
        f"SafetyGate/{scenario_name}"
    )
    assert absolute_arg in result.stdout
    assert f"--scenario SafetyGate/{scenario_name}" not in result.stdout


def test_tuning_gui_reports_configured_gate_vehicle_counts() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    module = ast.parse(
        (repo_root / "tools/tuning_gui/app.py").read_text(encoding="utf-8")
    )
    safety_gates = next(
        ast.literal_eval(node.value)
        for node in module.body
        if isinstance(node, ast.AnnAssign)
        and isinstance(node.target, ast.Name)
        and node.target.id == "SAFETY_GATES"
    )

    assert safety_gates["gate1"]["vehicles"] == 4
    assert safety_gates["gate2"]["vehicles"] == 4
    assert safety_gates["gate3"]["vehicles"] == 1


def test_make_dev_auto_starts_unless_explicitly_suppressed(
    tmp_path: Path,
) -> None:
    repo_root = Path(__file__).resolve().parents[3]
    output_root = tmp_path / "output"

    automatic = subprocess.run(
        [
            "make",
            "-n",
            "dev",
            "RUN_ID=dry-run",
            f"OUTPUT_HOST_ROOT={output_root}",
        ],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert automatic.returncode == 0, automatic.stderr
    assert "request_awsim_start.bash" in automatic.stdout
    assert "AWSIM_READY_DOMAINS=1" in automatic.stdout
    assert 'DEV_AUTO_START="true"' in automatic.stdout
    assert 'DEV_AUTO_START_EFFECTIVE_MODE="sync"' in automatic.stdout
    assert 'AWSIM_START_MODE="sync"' in automatic.stdout
    assert 'AWSIM_VEHICLES="1"' in automatic.stdout
    assert 'AWSIM_LAPS="600"' in automatic.stdout
    simulator_compose_index = automatic.stdout.index("docker compose up -d simulator")
    simulator_env = automatic.stdout[max(0, simulator_compose_index - 500) : simulator_compose_index]
    for expected in (
        'AWSIM_START_MODE="sync"',
        'AWSIM_VEHICLES="1"',
        'AWSIM_LAPS="600"',
        'AWSIM_TIMEOUT="60000000"',
    ):
        assert expected in simulator_env
    forced_count = subprocess.run(
        [
            "make",
            "-n",
            "dev",
            "DEV_AUTO_START_EFFECTIVE_MODE=count",
        ],
        cwd=repo_root,
        text=True,
        capture_output=True,
        check=True,
    )
    assert 'AWSIM_START_MODE="sync"' in forced_count.stdout
    assert "docker compose down --remove-orphans" in automatic.stdout

    suppressed = subprocess.run(
        [
            "make",
            "-n",
            "dev",
            "DEV_AUTO_START=false",
            "RUN_ID=dry-run",
            f"OUTPUT_HOST_ROOT={output_root}",
        ],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert suppressed.returncode == 0, suppressed.stderr
    assert "request_awsim_start.bash" not in suppressed.stdout
    assert 'DEV_AUTO_START="false"' in suppressed.stdout
    assert 'AWSIM_START_MODE="sync"' in suppressed.stdout


@pytest.mark.parametrize(
    ("target", "artifact_domains", "active_vehicles"),
    [
        ("dev2", "1,2", "2"),
        ("dev4", "1,2,3,4", "4"),
        ("gate1", "1", "4"),
        ("gate3", "1", "1"),
    ],
)
def test_non_overtake_runs_keep_start_only_path(
    tmp_path: Path, target: str, artifact_domains: str, active_vehicles: str
) -> None:
    repo_root = Path(__file__).resolve().parents[3]
    result = subprocess.run(
        [
            "make",
            "-n",
            target,
            "RUN_ID=dry-run",
            f"OUTPUT_HOST_ROOT={tmp_path / 'output'}",
        ],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    assert "awsim-request-start-and-watch-d1" not in result.stdout
    assert "awsim-request-start" in result.stdout
    assert result.stdout.count("request_awsim_start.bash") == 1
    assert f'--prepare-domains "{artifact_domains}"' in result.stdout
    assert f'AWSIM_VEHICLES="{active_vehicles}"' in result.stdout
    if target in {"gate1", "gate3"}:
        assert "DEV_AUTO_START=false" in result.stdout

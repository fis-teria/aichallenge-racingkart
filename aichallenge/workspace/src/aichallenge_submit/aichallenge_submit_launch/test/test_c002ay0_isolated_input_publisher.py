#!/usr/bin/env python3

import inspect
import os
import socket
import subprocess
import time
import unittest
from unittest.mock import Mock, mock_open, patch

from c002ay0_isolated_input_publisher import (
    ASSOCIATED_CHILD_PROVENANCE,
    ASSOCIATED_CHILD_ROLE,
    ACK_PHASE,
    ACK_FREEZE,
    COMMAND,
    COMMAND_PHASE,
    CHILD_MODE_ARGUMENT,
    CHILD_MODULE_PATH,
    DirectChildSample,
    IsolatedInputPublisher,
    MARKERS,
    PYTHON_EXECUTABLE,
    PopenProcessAdapter,
    _child_main,
    build_spawn_associated_child_ledger,
    child_argv,
    child_socket_from_fd,
    parse_child_argv,
    reconcile_spawn_associated_child_ledger,
    require_explicit_empty_cyclonedds_uri,
    sample_direct_child_identities,
    send_fixed_packet,
    stop_process_bounded,
    target_lifecycle_record,
    validate_exact_spawn_target,
    validate_ack,
    validate_command,
    validate_release,
)


class IsolatedInputPublisherUnitTest(unittest.TestCase):
    @staticmethod
    def child_identity(
        pid: int,
        *,
        owner_pid: int = 7000,
        starttime_ticks: int = 100,
        state: str = "S",
    ) -> dict[str, object]:
        return {
            "pid": pid,
            "readable": True,
            "classification": "zombie" if state == "Z" else "live",
            "state": state,
            "ppid": owner_pid,
            "session_id": owner_pid,
            "starttime_ticks": starttime_ticks,
            "cmdline": "python3 -c synthetic",
            "comm": "python3",
            "cgroup": ["0::/synthetic"],
        }

    def test_command_gap_duplicate_stale_and_unknown_fail_closed(self) -> None:
        packets = (
            COMMAND.pack(COMMAND_PHASE, 1, 1, 0),
            COMMAND.pack(COMMAND_PHASE, 3, 1, 0),
            COMMAND.pack(COMMAND_PHASE, 2, 0, 0),
            COMMAND.pack(99, 2, 2, 0),
        )
        expected = (
            None,
            "sequence_invalid",
            "generation_invalid",
            "command_unknown",
        )
        for packet, error in zip(packets, expected):
            with self.subTest(error=error):
                if error is None:
                    self.assertEqual(
                        validate_command(
                            packet,
                            expected_sequence=1,
                            expected_generation=1,
                        )[1:3],
                        (1, 1),
                    )
                else:
                    with self.assertRaisesRegex(AssertionError, error):
                        validate_command(
                            packet,
                            expected_sequence=2,
                            expected_generation=2,
                        )

    def test_ipc_full_and_partial_are_hard_invalid(self) -> None:
        endpoint = Mock()
        endpoint.send.side_effect = BlockingIOError()
        with self.assertRaisesRegex(AssertionError, "ipc_full"):
            send_fixed_packet(endpoint, b"x", "command")
        endpoint.send.side_effect = None
        endpoint.send.return_value = 0
        with self.assertRaisesRegex(AssertionError, "ipc_partial"):
            send_fixed_packet(endpoint, b"x", "command")

    def test_markers_exclude_m4_credit(self) -> None:
        self.assertTrue(MARKERS["diagnostic_only"])
        self.assertTrue(MARKERS["not_acceptance"])
        self.assertFalse(MARKERS["m4_wcet_eligible"])
        self.assertFalse(MARKERS["m4_acceptance_credit"])
        self.assertEqual(MARKERS["parent_lease_timeout_sec"], 30.0)
        self.assertEqual(
            MARKERS["launcher_diagnostic_baseline"],
            "direct_popen_exec_v1",
        )
        self.assertFalse(MARKERS["launcher_timing_parity"])

    def test_child_runtime_contains_no_descendant_spawn_path(self) -> None:
        child_source = inspect.getsource(_child_main)
        self.assertNotIn("Popen", child_source)
        self.assertNotIn("multiprocessing", child_source)
        self.assertNotIn("subprocess.", child_source)

    def test_child_argv_is_strict_and_validates_seqpacket_fd(self) -> None:
        argv = child_argv(
            command_fd=55,
            parent_pid=7000,
            curved=True,
            real_state_lattice=False,
            parent_lease_timeout_sec=4.5,
        )
        with patch(
            "c002ay0_isolated_input_publisher.os.getpid",
            return_value=7001,
        ):
            parsed = parse_child_argv(argv[2:])
        self.assertEqual(parsed.command_fd, 55)
        self.assertEqual(parsed.parent_pid, 7000)
        self.assertEqual(parsed.curved, 1)
        self.assertEqual(parsed.real_state_lattice, 0)
        self.assertEqual(parsed.parent_lease_timeout_sec, 4.5)
        for invalid_argv in (
            argv[3:],
            [*argv[2:], "--unknown"],
            [
                *argv[2:4],
                "-1",
                *argv[5:],
            ],
        ):
            with self.subTest(argv=invalid_argv):
                with (
                    patch(
                        "c002ay0_isolated_input_publisher.os.getpid",
                        return_value=7001,
                    ),
                    self.assertRaises(SystemExit),
                ):
                    parse_child_argv(invalid_argv)
        parent_pid_index = argv.index("--parent-pid") + 1
        lease_index = argv.index("--parent-lease-timeout-sec") + 1
        for index, invalid_value in (
            (parent_pid_index, "7001"),
            (lease_index, "nan"),
            (lease_index, "0"),
        ):
            invalid_argv = list(argv[2:])
            invalid_argv[index - 2] = invalid_value
            with self.subTest(invalid_value=invalid_value):
                with (
                    patch(
                        "c002ay0_isolated_input_publisher.os.getpid",
                        return_value=7001,
                    ),
                    self.assertRaises(SystemExit),
                ):
                    parse_child_argv(invalid_argv)

        endpoint = Mock()
        endpoint.getsockopt.return_value = socket.SOCK_SEQPACKET
        with patch(
            "c002ay0_isolated_input_publisher.socket.socket",
            return_value=endpoint,
        ) as socket_factory:
            self.assertIs(child_socket_from_fd(55), endpoint)
        socket_factory.assert_called_once_with(fileno=55)
        endpoint.getsockopt.assert_called_once_with(
            socket.SOL_SOCKET,
            socket.SO_TYPE,
        )
        endpoint.getsockopt.return_value = socket.SOCK_STREAM
        with (
            patch(
                "c002ay0_isolated_input_publisher.socket.socket",
                return_value=endpoint,
            ),
            self.assertRaisesRegex(AssertionError, "socket_type_invalid"),
        ):
            child_socket_from_fd(55)
        endpoint.close.assert_called_once_with()

    def test_exact_spawn_target_rejects_missing_extra_unreadable_mismatch(
        self,
    ) -> None:
        owner_pid = 7000
        target_pid = 7001
        valid = self.child_identity(target_pid)
        cases = {
            "missing": ({}, "target_missing"),
            "extra": (
                {
                    target_pid: valid,
                    7002: self.child_identity(7002),
                },
                "direct_child_delta_not_exact",
            ),
            "unreadable": (
                DirectChildSample(
                    {
                        target_pid: {
                            "pid": target_pid,
                            "readable": False,
                            "classification": "unreadable",
                            "error": "PermissionError: denied",
                        }
                    }
                ),
                "target_identity_unreadable",
            ),
            "ppid_mismatch": (
                {
                    target_pid: self.child_identity(
                        target_pid,
                        owner_pid=8000,
                    )
                },
                "target_ppid_mismatch",
            ),
            "sid_mismatch": (
                {
                    target_pid: {
                        **valid,
                        "session_id": 8000,
                    }
                },
                "target_sid_mismatch",
            ),
        }
        for name, (after, expected) in cases.items():
            with self.subTest(name=name):
                target, failures = validate_exact_spawn_target(
                    before=DirectChildSample(),
                    after=DirectChildSample(after),
                    target_pid=target_pid,
                    owner_pid=owner_pid,
                    owner_sid=owner_pid,
                )
                if name == "missing":
                    self.assertIsNone(target)
                self.assertIn(expected, " ".join(failures))

    def test_process_status_exposes_exact_child_lifecycle(self) -> None:
        publisher = IsolatedInputPublisher.__new__(
            IsolatedInputPublisher
        )
        publisher._process = Mock()
        publisher._process.pid = 4321
        publisher._process.is_alive.return_value = False
        publisher._process.exitcode = 0
        self.assertEqual(
            publisher.process_status,
            {
                "pid": 4321,
                "role": "isolated_input_publisher",
                "alive": False,
                "exitcode": 0,
                "target_process_lifecycle": None,
                "target_cleanup_failure": "",
                "associated_child_lifecycle": [],
                "associated_child_cleanup_failure": "",
                "associated_child_sample_failures": [],
            },
        )

    def test_constructor_execs_exact_module_with_seqpacket_fd(
        self,
    ) -> None:
        owner_pid = 7000
        target_pid = 7001
        events: list[str] = []
        snapshots = iter(
            (
                {},
                {target_pid: self.child_identity(target_pid)},
            )
        )

        def sampler(observed_owner_pid: int) -> dict:
            self.assertEqual(observed_owner_pid, owner_pid)
            events.append("sample")
            return next(snapshots)

        popen = Mock()
        popen.pid = target_pid
        popen.poll.return_value = None
        popen_factory = Mock(
            side_effect=lambda *args, **kwargs: (
                events.append("popen") or popen
            )
        )
        parent_socket = Mock()
        child_socket = Mock()
        child_socket.fileno.return_value = 55
        ready = {
            "kind": 1,
            "sequence": 0,
            "generation": 0,
            "tick_index": 1,
            "publish_complete_watermark": 1,
            "stamp_ns": 1,
            "topic_counts": (1, 1, 1, 1, 1, 1),
        }
        with (
            patch(
                "c002ay0_isolated_input_publisher.os.getpid",
                return_value=owner_pid,
            ),
            patch(
                "c002ay0_isolated_input_publisher.os.getsid",
                return_value=owner_pid,
            ),
            patch(
                "c002ay0_isolated_input_publisher.socket.socketpair",
                return_value=(parent_socket, child_socket),
            ),
            patch.object(
                IsolatedInputPublisher,
                "_receive_ack",
                return_value=ready,
            ),
        ):
            publisher = IsolatedInputPublisher(
                curved=False,
                real_state_lattice=False,
                direct_child_sampler=sampler,
                monotonic_ns=Mock(side_effect=(10, 20)),
                waitpid=Mock(),
                popen_factory=popen_factory,
            )
        self.assertEqual(events, ["sample", "popen", "sample"])
        argv = popen_factory.call_args.args[0]
        self.assertEqual(argv[0], PYTHON_EXECUTABLE)
        self.assertEqual(argv[1], CHILD_MODULE_PATH)
        self.assertEqual(argv[2], CHILD_MODE_ARGUMENT)
        self.assertIn("--command-fd", argv)
        popen_kwargs = popen_factory.call_args.kwargs
        self.assertIn("CYCLONEDDS_URI", popen_kwargs["env"])
        self.assertEqual(popen_kwargs["env"]["CYCLONEDDS_URI"], "")
        self.assertEqual(
            {
                key: value
                for key, value in popen_kwargs.items()
                if key != "env"
            },
            {
                "stdin": subprocess.DEVNULL,
                "pass_fds": (55,),
                "close_fds": True,
                "shell": False,
                "preexec_fn": None,
            },
        )
        child_socket.close.assert_called_once_with()
        lifecycle = publisher.process_status["target_process_lifecycle"]
        self.assertEqual(lifecycle["pid"], target_pid)
        self.assertEqual(lifecycle["starttime_ticks"], 100)
        self.assertEqual(lifecycle["sid"], owner_pid)
        self.assertEqual(lifecycle["ppid"], owner_pid)
        self.assertEqual(
            publisher.process_status["associated_child_lifecycle"],
            [],
        )

    def test_cyclonedds_uri_contract_requires_present_empty_value(
        self,
    ) -> None:
        require_explicit_empty_cyclonedds_uri({"CYCLONEDDS_URI": ""})
        for environment in ({}, {"CYCLONEDDS_URI": "file:///bad.xml"}):
            with self.assertRaisesRegex(
                AssertionError,
                "cyclonedds_uri_not_explicit_empty",
            ):
                require_explicit_empty_cyclonedds_uri(environment)

    def test_child_rejects_nonempty_cyclonedds_before_rclpy_init(
        self,
    ) -> None:
        with (
            patch.dict(
                os.environ,
                {"CYCLONEDDS_URI": "file:///bad.xml"},
                clear=False,
            ),
            patch(
                "c002ay0_isolated_input_publisher.os.getpid",
                return_value=9001,
            ),
            patch(
                "c002ay0_isolated_input_publisher.rclpy.init"
            ) as init,
        ):
            with self.assertRaisesRegex(
                AssertionError,
                "cyclonedds_uri_not_explicit_empty",
            ):
                _child_main(
                    Mock(),
                    9000,
                    False,
                    False,
                    30.0,
                )
        init.assert_not_called()

    def constructor_sample_failure(
        self,
        samples: tuple[object, ...],
        popen: Mock,
    ) -> tuple[AssertionError, Mock, Mock, Mock, Mock]:
        parent_socket = Mock()
        child_socket = Mock()
        child_socket.fileno.return_value = 55
        receive_ack = Mock()
        popen_factory = Mock(return_value=popen)
        sampled = iter(samples)
        with (
            patch(
                "c002ay0_isolated_input_publisher.os.getpid",
                return_value=7000,
            ),
            patch(
                "c002ay0_isolated_input_publisher.os.getsid",
                return_value=7000,
            ),
            patch(
                "c002ay0_isolated_input_publisher.socket.socketpair",
                return_value=(parent_socket, child_socket),
            ),
            patch.object(
                IsolatedInputPublisher,
                "_receive_ack",
                new=receive_ack,
            ),
            self.assertRaises(AssertionError) as raised,
        ):
            IsolatedInputPublisher(
                curved=False,
                real_state_lattice=False,
                direct_child_sampler=lambda _: next(sampled),
                monotonic_ns=Mock(side_effect=(10, 20)),
                waitpid=Mock(),
                popen_factory=popen_factory,
            )
        return (
            raised.exception,
            parent_socket,
            child_socket,
            receive_ack,
            popen_factory,
        )

    def test_before_sample_failure_does_not_start_target(self) -> None:
        popen = Mock()
        popen.pid = 7001
        failure = DirectChildSample(
            errors=[
                {
                    "pid": 7000,
                    "kind": "owner_task_children_read_failed",
                    "detail": "PermissionError: before denied",
                }
            ]
        )
        error, parent_socket, child_socket, receive_ack, popen_factory = (
            self.constructor_sample_failure((failure,), popen)
        )
        self.assertIn("before denied", str(error))
        popen_factory.assert_not_called()
        parent_socket.close.assert_called_once_with()
        child_socket.close.assert_called_once_with()
        receive_ack.assert_not_called()

    def test_after_sample_failure_cleans_only_known_target(self) -> None:
        popen = Mock()
        popen.pid = 7001
        popen.wait.return_value = 0
        popen.poll.return_value = 0
        failure = DirectChildSample(
            errors=[
                {
                    "pid": 7000,
                    "kind": "owner_task_children_read_failed",
                    "detail": "PermissionError: after denied",
                }
            ]
        )
        error, parent_socket, child_socket, receive_ack, popen_factory = (
            self.constructor_sample_failure(({}, failure), popen)
        )
        self.assertIn("after denied", str(error))
        self.assertIn(
            "after_owner_task_children_read_failed: "
            "PermissionError: after denied",
            getattr(error, "_associated_child_sample_failures"),
        )
        popen_factory.assert_called_once()
        popen.wait.assert_called_once_with(timeout=1.0)
        popen.terminate.assert_not_called()
        popen.kill.assert_not_called()
        parent_socket.close.assert_called_once_with()
        child_socket.close.assert_called_once_with()
        receive_ack.assert_not_called()

    def test_after_sample_failure_preserves_target_cleanup_failure(
        self,
    ) -> None:
        popen = Mock()
        popen.pid = 7001
        popen.wait.side_effect = RuntimeError("wait exploded")
        failure = DirectChildSample(
            errors=[
                {
                    "pid": 7000,
                    "kind": "owner_task_children_read_failed",
                    "detail": "PermissionError: sample denied",
                }
            ]
        )
        error, parent_socket, child_socket, receive_ack, popen_factory = (
            self.constructor_sample_failure(({}, failure), popen)
        )
        self.assertIn("sample denied", str(error))
        self.assertIn("wait exploded", str(error))
        self.assertIn(
            "wait exploded",
            getattr(error, "_target_cleanup_failure"),
        )
        popen_factory.assert_called_once()
        parent_socket.close.assert_called_once_with()
        child_socket.close.assert_called_once_with()
        receive_ack.assert_not_called()

    def test_constructor_rejects_non_exact_target_and_cleans_popen(
        self,
    ) -> None:
        target_pid = 7001
        valid = self.child_identity(target_pid)
        cases = {
            "missing": ({}, "target_missing"),
            "extra": (
                {
                    target_pid: valid,
                    7002: self.child_identity(7002),
                },
                "direct_child_delta_not_exact",
            ),
            "unreadable": (
                {
                    target_pid: {
                        "pid": target_pid,
                        "readable": False,
                        "classification": "unreadable",
                        "error": "PermissionError: denied",
                    }
                },
                "target_identity_unreadable",
            ),
            "mismatch": (
                {
                    target_pid: {
                        **valid,
                        "session_id": 8000,
                    }
                },
                "target_sid_mismatch",
            ),
        }
        for name, (after, expected) in cases.items():
            with self.subTest(name=name):
                popen = Mock()
                popen.pid = target_pid
                popen.wait.return_value = 0
                popen.poll.return_value = 0
                (
                    error,
                    parent_socket,
                    child_socket,
                    receive_ack,
                    _,
                ) = self.constructor_sample_failure(({}, after), popen)
                self.assertIn(expected, str(error))
                popen.wait.assert_called_once_with(timeout=1.0)
                popen.terminate.assert_not_called()
                popen.kill.assert_not_called()
                parent_socket.close.assert_called_once_with()
                child_socket.close.assert_called_once_with()
                receive_ack.assert_not_called()

    def test_popen_failure_closes_both_sockets_without_sampling_after(
        self,
    ) -> None:
        parent_socket = Mock()
        child_socket = Mock()
        child_socket.fileno.return_value = 55
        popen_error = OSError("exec failed")
        sampler = Mock(return_value={})
        with (
            patch(
                "c002ay0_isolated_input_publisher.os.getpid",
                return_value=7000,
            ),
            patch(
                "c002ay0_isolated_input_publisher.os.getsid",
                return_value=7000,
            ),
            patch(
                "c002ay0_isolated_input_publisher.socket.socketpair",
                return_value=(parent_socket, child_socket),
            ),
            self.assertRaises(OSError) as raised,
        ):
            IsolatedInputPublisher(
                curved=False,
                real_state_lattice=False,
                direct_child_sampler=sampler,
                popen_factory=Mock(side_effect=popen_error),
            )
        self.assertIs(raised.exception, popen_error)
        sampler.assert_called_once_with(7000)
        parent_socket.close.assert_called_once_with()
        child_socket.close.assert_called_once_with()

    def test_popen_adapter_cleanup_wait_term_wait_kill_wait(self) -> None:
        popen = Mock()
        popen.pid = 7001
        popen.wait.side_effect = (
            subprocess.TimeoutExpired(["child"], 1.0),
            subprocess.TimeoutExpired(["child"], 1.0),
            0,
        )
        popen.poll.side_effect = (None, None, 0, 0)
        process = PopenProcessAdapter(popen)

        stop_process_bounded(process)

        self.assertEqual(
            popen.wait.call_args_list,
            [
                unittest.mock.call(timeout=1.0),
                unittest.mock.call(timeout=1.0),
                unittest.mock.call(timeout=1.0),
            ],
        )
        popen.terminate.assert_called_once_with()
        popen.kill.assert_called_once_with()
        self.assertFalse(process.is_alive())

    def test_close_records_exact_target_absence_and_is_idempotent(
        self,
    ) -> None:
        owner_pid = 7000
        target_pid = 7001
        target_identity = self.child_identity(target_pid)
        target_absent = {
            "pid": target_pid,
            "readable": False,
            "classification": "absent",
            "error": "FileNotFoundError",
        }
        publisher = IsolatedInputPublisher.__new__(
            IsolatedInputPublisher
        )
        publisher._owner_pid = owner_pid
        publisher._process = Mock()
        publisher._process.pid = target_pid
        publisher._process.join.return_value = None
        publisher._process.is_alive.return_value = False
        publisher._process.exitcode = 0
        publisher._parent_socket = Mock()
        publisher._closed = False
        publisher._direct_child_sampler = Mock(return_value={})
        publisher._process_identity_sampler = Mock(
            side_effect=(target_identity, target_absent)
        )
        publisher._target_process_lifecycle = target_lifecycle_record(
            target_identity,
            owner_pid=owner_pid,
            spawn_before_monotonic_ns=10,
            spawn_after_monotonic_ns=20,
        )
        publisher._target_cleanup_complete = False
        publisher._target_cleanup_failure = ""

        publisher.close()
        publisher.close()

        publisher._process.join.assert_called_once_with(1.0)
        publisher._process.terminate.assert_not_called()
        publisher._process.kill.assert_not_called()
        publisher._parent_socket.close.assert_called_once_with()
        lifecycle = publisher._target_process_lifecycle
        self.assertEqual(
            lifecycle["identity_poststop"]["classification"],
            "absent",
        )
        self.assertEqual(lifecycle["cleanup_stage"], "absent_no_residue")
        self.assertFalse(lifecycle["unresolved"])
        self.assertEqual(publisher._target_cleanup_failure, "")

    def test_cleanup_rejects_descendants_without_signaling_them(
        self,
    ) -> None:
        owner_pid = 7000
        target_pid = 7001
        descendant_pid = 7002
        target_identity = self.child_identity(target_pid)
        descendant_identity = self.child_identity(
            descendant_pid,
            owner_pid=target_pid,
        )
        target_absent = {
            "pid": target_pid,
            "readable": False,
            "classification": "absent",
            "error": "FileNotFoundError",
        }
        descendant_absent = {
            "pid": descendant_pid,
            "readable": False,
            "classification": "absent",
            "error": "FileNotFoundError",
        }
        publisher = IsolatedInputPublisher.__new__(
            IsolatedInputPublisher
        )
        publisher._owner_pid = owner_pid
        publisher._process = Mock()
        publisher._process.pid = target_pid
        publisher._process.join.return_value = None
        publisher._process.is_alive.return_value = False
        publisher._process.exitcode = 0
        publisher._direct_child_sampler = Mock(
            return_value={descendant_pid: descendant_identity}
        )
        identities = iter(
            (target_identity, descendant_absent, target_absent)
        )
        publisher._process_identity_sampler = Mock(
            side_effect=lambda _: next(identities)
        )
        publisher._target_process_lifecycle = target_lifecycle_record(
            target_identity,
            owner_pid=owner_pid,
            spawn_before_monotonic_ns=10,
            spawn_after_monotonic_ns=20,
        )
        publisher._target_cleanup_complete = False
        publisher._target_cleanup_failure = ""

        with self.assertRaisesRegex(
            AssertionError,
            "target_descendants_created",
        ):
            publisher._cleanup_known_target()

        publisher._process.join.assert_called_once_with(1.0)
        publisher._process.terminate.assert_not_called()
        publisher._process.kill.assert_not_called()
        lifecycle = publisher._target_process_lifecycle
        self.assertEqual(
            lifecycle["descendants_before_stop"][0]["pid"],
            descendant_pid,
        )
        self.assertEqual(
            lifecycle["descendants_poststop"][0]["classification"],
            "absent",
        )
        self.assertEqual(
            lifecycle["identity_poststop"]["classification"],
            "absent",
        )
        with self.assertRaisesRegex(
            AssertionError,
            "target_descendants_created",
        ):
            publisher._cleanup_known_target()
        publisher._process.join.assert_called_once_with(1.0)

    def test_spawn_associated_child_ledger_reaps_exact_zombie(
        self,
    ) -> None:
        owner_pid = 7000
        target_pid = 7001
        helper_pid = 7002
        unchanged_pid = 6999
        unchanged = self.child_identity(unchanged_pid)
        helper = self.child_identity(helper_pid)
        records = build_spawn_associated_child_ledger(
            before={unchanged_pid: unchanged},
            after={
                unchanged_pid: unchanged,
                target_pid: self.child_identity(target_pid),
                helper_pid: helper,
            },
            target_pid=target_pid,
            owner_pid=owner_pid,
            spawn_before_monotonic_ns=10,
            spawn_after_monotonic_ns=20,
        )
        self.assertEqual(len(records), 1)
        record = records[0]
        self.assertEqual(record["pid"], helper_pid)
        self.assertEqual(record["role"], ASSOCIATED_CHILD_ROLE)
        self.assertEqual(
            record["provenance"],
            ASSOCIATED_CHILD_PROVENANCE,
        )
        self.assertEqual(record["owner_pid"], owner_pid)
        self.assertEqual(record["spawn_before_monotonic_ns"], 10)
        self.assertEqual(record["spawn_after_monotonic_ns"], 20)
        for field in (
            "starttime_ticks",
            "session_id",
            "ppid",
            "state",
            "cmdline",
            "comm",
            "cgroup",
        ):
            self.assertIn(field, record["identity_at_spawn"])
            self.assertEqual(
                record[field],
                record["identity_at_spawn"][field],
            )
        self.assertEqual(
            record["sid"],
            record["identity_at_spawn"]["session_id"],
        )

        zombie = self.child_identity(helper_pid, state="Z")
        waitpid = Mock(return_value=(helper_pid, 0))
        reconcile_spawn_associated_child_ledger(
            records,
            owner_pid=owner_pid,
            direct_child_sampler=lambda _: {helper_pid: zombie},
            waitpid=waitpid,
        )
        waitpid.assert_called_once_with(helper_pid, os.WNOHANG)
        self.assertEqual(record["cleanup_stage"], "reaped_no_residue")
        self.assertFalse(record["alive"])
        self.assertFalse(record["unresolved"])
        self.assertEqual(record["exitcode"], 0)
        self.assertEqual(record["cleanup_failure"], "")

    def test_direct_child_sampler_unions_all_owner_threads(self) -> None:
        first_task = Mock()
        first_task.name = "7000"
        second_task = Mock()
        second_task.name = "7004"
        identities = {
            7002: self.child_identity(7002),
            7003: self.child_identity(7003),
        }

        def open_children(path: str, *args: object, **kwargs: object) -> object:
            del args, kwargs
            return mock_open(
                read_data=(
                    "7002"
                    if path.endswith("/7000/children")
                    else "7002 7003"
                )
            )()

        with (
            patch(
                "c002ay0_isolated_input_publisher.os.scandir",
                return_value=[first_task, second_task],
            ) as scandir,
            patch(
                "builtins.open",
                side_effect=open_children,
            ),
            patch(
                "c002ay0_isolated_input_publisher."
                "process_identity_snapshot",
                side_effect=lambda pid: identities[pid],
            ),
        ):
            sampled = sample_direct_child_identities(7000)
        scandir.assert_called_once_with("/proc/7000/task")
        self.assertEqual(sampled, identities)
        self.assertEqual(sampled.errors, [])

    def test_unreadable_direct_child_is_invalid_and_not_reaped(
        self,
    ) -> None:
        owner_pid = 7000
        helper_pid = 7002
        task = Mock()
        task.name = str(owner_pid)
        unreadable = {
            "pid": helper_pid,
            "readable": False,
            "classification": "unreadable",
            "error": "PermissionError: denied",
        }
        with (
            patch(
                "c002ay0_isolated_input_publisher.os.scandir",
                return_value=[task],
            ),
            patch("builtins.open", mock_open(read_data=str(helper_pid))),
            patch(
                "c002ay0_isolated_input_publisher."
                "process_identity_snapshot",
                return_value=unreadable,
            ),
        ):
            sampled = sample_direct_child_identities(owner_pid)
        self.assertEqual(sampled[helper_pid], unreadable)
        self.assertEqual(
            sampled.errors[0]["kind"],
            "direct_child_identity_unreadable",
        )
        records = build_spawn_associated_child_ledger(
            before=DirectChildSample(),
            after=sampled,
            target_pid=7001,
            owner_pid=owner_pid,
            spawn_before_monotonic_ns=10,
            spawn_after_monotonic_ns=20,
        )
        waitpid = Mock()
        reconcile_spawn_associated_child_ledger(
            records,
            owner_pid=owner_pid,
            direct_child_sampler=lambda _: {},
            waitpid=waitpid,
        )
        waitpid.assert_not_called()
        self.assertTrue(records[0]["unresolved"])
        self.assertEqual(
            records[0]["cleanup_stage"],
            "cleanup_sample_invalid",
        )
        self.assertIn(
            "direct_child_identity_unreadable",
            records[0]["cleanup_failure"],
        )

    def test_direct_child_ppid_mismatch_is_invalid_and_not_reaped(
        self,
    ) -> None:
        owner_pid = 7000
        helper_pid = 7002
        mismatch = self.child_identity(
            helper_pid,
            owner_pid=8000,
        )
        sampled = DirectChildSample(
            {helper_pid: mismatch},
            errors=[
                {
                    "pid": helper_pid,
                    "kind": "direct_child_ppid_mismatch",
                    "detail": "expected=7000 actual=8000",
                }
            ],
        )
        records = build_spawn_associated_child_ledger(
            before=DirectChildSample(),
            after=sampled,
            target_pid=7001,
            owner_pid=owner_pid,
            spawn_before_monotonic_ns=10,
            spawn_after_monotonic_ns=20,
        )
        waitpid = Mock()
        reconcile_spawn_associated_child_ledger(
            records,
            owner_pid=owner_pid,
            direct_child_sampler=lambda _: {},
            waitpid=waitpid,
        )
        waitpid.assert_not_called()
        self.assertTrue(records[0]["unresolved"])
        self.assertIn(
            "direct_child_ppid_mismatch",
            records[0]["cleanup_failure"],
        )

    def test_owner_task_children_read_failure_is_invalid(self) -> None:
        owner_pid = 7000
        task = Mock()
        task.name = str(owner_pid)
        with (
            patch(
                "c002ay0_isolated_input_publisher.os.scandir",
                return_value=[task],
            ),
            patch(
                "builtins.open",
                side_effect=PermissionError("denied"),
            ),
        ):
            sampled = sample_direct_child_identities(owner_pid)
        self.assertEqual(dict(sampled), {})
        self.assertEqual(
            sampled.errors[0]["kind"],
            "owner_task_children_read_failed",
        )
        records = build_spawn_associated_child_ledger(
            before=DirectChildSample(),
            after=sampled,
            target_pid=7001,
            owner_pid=owner_pid,
            spawn_before_monotonic_ns=10,
            spawn_after_monotonic_ns=20,
        )
        self.assertEqual(records, [])
        failure = (
            "owner_task_children_read_failed: "
            "PermissionError: denied"
        )
        publisher = IsolatedInputPublisher.__new__(
            IsolatedInputPublisher
        )
        publisher._process = Mock()
        publisher._process.pid = 7001
        publisher._process.is_alive.return_value = False
        publisher._process.exitcode = 0
        publisher._owner_pid = owner_pid
        publisher._direct_child_sampler = lambda _: {}
        publisher._waitpid = Mock()
        publisher._associated_child_lifecycle = records
        publisher._associated_child_sample_failures = [failure]
        publisher._associated_child_cleanup_complete = False
        publisher._associated_child_cleanup_failure = failure
        waitpid = Mock()
        publisher._waitpid = waitpid
        with self.assertRaisesRegex(
            AssertionError,
            "owner_task_children_read_failed",
        ):
            publisher._reconcile_associated_children()
        waitpid.assert_not_called()
        status = publisher.process_status
        self.assertEqual(status["associated_child_lifecycle"], [])
        self.assertEqual(
            status["associated_child_sample_failures"],
            [failure],
        )
        self.assertIn(
            "owner_task_children_read_failed",
            status["associated_child_cleanup_failure"],
        )

    def test_spawn_associated_zombie_remaining_is_invalid(self) -> None:
        owner_pid = 7000
        helper_pid = 7002
        helper = self.child_identity(helper_pid)
        records = build_spawn_associated_child_ledger(
            before={},
            after={helper_pid: helper},
            target_pid=7001,
            owner_pid=owner_pid,
            spawn_before_monotonic_ns=10,
            spawn_after_monotonic_ns=20,
        )
        reconcile_spawn_associated_child_ledger(
            records,
            owner_pid=owner_pid,
            direct_child_sampler=lambda _: {
                helper_pid: self.child_identity(
                    helper_pid,
                    state="Z",
                )
            },
            waitpid=Mock(return_value=(0, 0)),
        )
        record = records[0]
        self.assertEqual(record["cleanup_stage"], "zombie_not_reaped")
        self.assertTrue(record["alive"])
        self.assertTrue(record["unresolved"])
        self.assertTrue(record["cleanup_failure"])

    def test_spawn_associated_pid_reuse_is_not_reaped(self) -> None:
        owner_pid = 7000
        helper_pid = 7002
        records = build_spawn_associated_child_ledger(
            before={},
            after={helper_pid: self.child_identity(helper_pid)},
            target_pid=7001,
            owner_pid=owner_pid,
            spawn_before_monotonic_ns=10,
            spawn_after_monotonic_ns=20,
        )
        waitpid = Mock()
        reconcile_spawn_associated_child_ledger(
            records,
            owner_pid=owner_pid,
            direct_child_sampler=lambda _: {
                helper_pid: self.child_identity(
                    helper_pid,
                    starttime_ticks=101,
                    state="Z",
                )
            },
            waitpid=waitpid,
        )
        waitpid.assert_not_called()
        record = records[0]
        self.assertEqual(
            record["cleanup_stage"],
            "identity_mismatch_not_reaped",
        )
        self.assertTrue(record["unresolved"])
        self.assertTrue(record["cleanup_failure"])

    def test_spawn_associated_live_child_is_not_reaped(self) -> None:
        owner_pid = 7000
        helper_pid = 7002
        records = build_spawn_associated_child_ledger(
            before={},
            after={helper_pid: self.child_identity(helper_pid)},
            target_pid=7001,
            owner_pid=owner_pid,
            spawn_before_monotonic_ns=10,
            spawn_after_monotonic_ns=20,
        )
        waitpid = Mock()
        reconcile_spawn_associated_child_ledger(
            records,
            owner_pid=owner_pid,
            direct_child_sampler=lambda _: {
                helper_pid: self.child_identity(helper_pid)
            },
            waitpid=waitpid,
        )
        waitpid.assert_not_called()
        record = records[0]
        self.assertEqual(
            record["cleanup_stage"],
            "unknown_live_not_signaled",
        )
        self.assertTrue(record["unresolved"])
        self.assertTrue(record["cleanup_failure"])

    def test_missed_release_prevents_catch_up_burst(self) -> None:
        validate_release(9_999_999, 0)
        with self.assertRaisesRegex(AssertionError, "missed_release"):
            validate_release(10_000_000, 0)

    def test_ack_stall_and_crash_are_hard_invalid_and_cleaned(self) -> None:
        for readable, packet, error in (
            ((), b"", "ack_stall"),
            ((object(),), b"", "ack_size_invalid"),
        ):
            with self.subTest(error=error):
                publisher = IsolatedInputPublisher.__new__(
                    IsolatedInputPublisher
                )
                publisher._parent_socket = Mock()
                publisher._parent_socket.recv.return_value = packet
                publisher._process = Mock()
                publisher._process.is_alive.return_value = False
                publisher._closed = False
                with patch(
                    "c002ay0_isolated_input_publisher.select.select",
                    return_value=(readable, (), ()),
                ):
                    with self.assertRaisesRegex(AssertionError, error):
                        publisher._receive_ack(0.0)
                publisher._parent_socket.close.assert_called_once_with()

    def test_ack_kind_replay_and_nonmonotonic_are_hard_invalid(
        self,
    ) -> None:
        previous = {
            "kind": ACK_PHASE,
            "sequence": 1,
            "generation": 1,
            "tick_index": 5,
            "publish_complete_watermark": 5,
            "stamp_ns": 100,
            "topic_counts": (5, 5, 5, 1, 1, 5),
        }
        valid = {
            **previous,
            "sequence": 2,
            "generation": 2,
            "tick_index": 10,
            "publish_complete_watermark": 10,
            "stamp_ns": 200,
        }
        validate_ack(
            valid,
            previous_ack=previous,
            expected_kind=ACK_PHASE,
            expected_sequence=2,
            expected_generation=2,
        )
        for mutation, error in (
            ({"kind": ACK_FREEZE}, "kind_invalid"),
            ({"sequence": 1}, "identity_invalid"),
            ({"tick_index": 5, "publish_complete_watermark": 5},
             "nonmonotonic"),
            ({"stamp_ns": 100}, "nonmonotonic"),
        ):
            with self.subTest(error=error):
                with self.assertRaisesRegex(AssertionError, error):
                    validate_ack(
                        {**valid, **mutation},
                        previous_ack=previous,
                        expected_kind=ACK_PHASE,
                        expected_sequence=2,
                        expected_generation=2,
                    )


@unittest.skipUnless(
    os.environ.get("C002AY0_ISOLATED_PUBLISHER_INTEGRATION") == "1",
    "DDS integration is explicit",
)
class IsolatedInputPublisherIntegrationTest(unittest.TestCase):
    def assert_popen_target_closed_without_child_residue(
        self,
        publisher: IsolatedInputPublisher,
    ) -> None:
        status = publisher.process_status
        self.assertEqual(status["associated_child_lifecycle"], [])
        lifecycle = status["target_process_lifecycle"]
        self.assertIsInstance(lifecycle, dict)
        self.assertEqual(lifecycle["cleanup_stage"], "absent_no_residue")
        self.assertFalse(lifecycle["alive"])
        self.assertFalse(lifecycle["unresolved"])
        self.assertEqual(lifecycle["descendants_before_stop"], [])
        self.assertEqual(lifecycle["descendants_poststop"], [])
        direct_children = sample_direct_child_identities(os.getpid())
        self.assertEqual(direct_children.errors, [])
        self.assertEqual(dict(direct_children), {})

    def test_spawn_context_ack_watermark_cadence_freeze_shutdown(
        self,
    ) -> None:
        import rclpy
        from autoware_auto_planning_msgs.msg import Trajectory
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Bool
        from v2x_msgs.msg import V2XVehiclePositionArray
        from multi_purpose_mpc_ros_msgs.msg import OvertakePlan

        rclpy.init()
        node = rclpy.create_node("isolated_input_publisher_observer")
        received = [0] * 6
        armed_values: list[bool] = []
        topics = (
            (Clock, "/clock"),
            (Odometry, "/ay0/input/kinematics"),
            (V2XVehiclePositionArray, "/ay0/input/v2x"),
            (Trajectory, "/ay0/input/trajectory"),
            (OvertakePlan, "/ay0/input/overtake_plan"),
            (Bool, "/ay0/input/race_armed"),
        )
        subscriptions = [
            node.create_subscription(
                message_type,
                topic,
                lambda _message, index=index: received.__setitem__(
                    index,
                    received[index] + 1,
                ),
                QoSProfile(depth=10),
            )
            for index, (message_type, topic) in enumerate(topics)
        ]
        node.destroy_subscription(subscriptions[-1])
        subscriptions[-1] = node.create_subscription(
            Bool,
            "/ay0/input/race_armed",
            lambda message: (
                received.__setitem__(5, received[5] + 1),
                armed_values.append(bool(message.data)),
            ),
            QoSProfile(depth=10),
        )
        parent_pid = os.getpid()
        publisher = IsolatedInputPublisher(
            curved=True,
            real_state_lattice=False,
        )
        try:
            self.assertNotEqual(publisher.pid, parent_pid)
            first = publisher.phase(True)
            self.assertEqual(first["kind"], ACK_PHASE)
            self.assertEqual(
                first["publish_complete_watermark"],
                first["tick_index"],
            )
            counts = first["topic_counts"]
            self.assertGreaterEqual(counts[0], counts[3] * 5)
            self.assertEqual(counts[3], counts[4])
            deadline = time.monotonic() + 2.0
            while (
                (min(received) == 0 or True not in armed_values)
                and time.monotonic() < deadline
            ):
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertGreater(min(received), 0)
            self.assertIn(True, armed_values)
            for _, topic in topics:
                self.assertEqual(node.count_publishers(topic), 1)
            frozen = publisher.freeze()
            deadline = time.monotonic() + 1.0
            while (
                (not armed_values or armed_values[-1])
                and time.monotonic() < deadline
            ):
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertFalse(armed_values[-1])
            time.sleep(0.03)
            shutdown_receive_start = len(armed_values)
            shutdown_false_count_before = armed_values.count(False)
            shutdown = publisher.shutdown()
            deadline = time.monotonic() + 1.0
            while (
                (
                    len(armed_values) == shutdown_receive_start
                    or False
                    not in armed_values[shutdown_receive_start:]
                )
                and time.monotonic() < deadline
            ):
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertGreater(
                len(armed_values),
                shutdown_receive_start,
            )
            self.assertIn(
                False,
                armed_values[shutdown_receive_start:],
            )
            self.assertGreater(
                armed_values.count(False),
                shutdown_false_count_before,
            )
            self.assertGreater(
                shutdown["topic_counts"][5],
                frozen["topic_counts"][5],
            )
            self.assertFalse(armed_values[-1])
            self.assertFalse(publisher._process.is_alive())
            self.assertGreater(
                shutdown["publish_complete_watermark"],
                frozen["publish_complete_watermark"],
            )
            self.assertEqual(publisher.shutdown(), shutdown)
            publisher.close()
            publisher.close()
            self.assert_popen_target_closed_without_child_residue(
                publisher
            )
        finally:
            publisher.close()
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()

    def test_absolute_publisher_is_autonomous_without_tick_rpc(self) -> None:
        publisher = IsolatedInputPublisher(
            curved=False,
            real_state_lattice=True,
        )
        try:
            time.sleep(0.12)
            ack = publisher.phase(False)
            counts = ack["topic_counts"]
            self.assertGreaterEqual(counts[0], 10)
            self.assertGreaterEqual(counts[0], counts[3] * 10)
            self.assertEqual(counts[3], counts[4])
            publisher.shutdown()
        finally:
            publisher.close()
        self.assert_popen_target_closed_without_child_residue(publisher)

    def test_parent_lease_expiry_disarms_and_exits(self) -> None:
        publisher = IsolatedInputPublisher(
            curved=False,
            real_state_lattice=False,
            parent_lease_timeout_sec=0.05,
        )
        deadline = time.monotonic() + 1.0
        while publisher._process.is_alive() and time.monotonic() < deadline:
            time.sleep(0.01)
        with self.assertRaisesRegex(
            AssertionError,
            "process_not_alive",
        ):
            publisher.assert_alive()
        self.assertFalse(publisher._process.is_alive())
        publisher.close()
        publisher.close()
        self.assert_popen_target_closed_without_child_residue(publisher)


if __name__ == "__main__":
    unittest.main()

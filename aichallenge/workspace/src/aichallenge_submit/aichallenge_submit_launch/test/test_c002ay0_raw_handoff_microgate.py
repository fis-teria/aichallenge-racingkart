#!/usr/bin/env python3

import multiprocessing
import os
from pathlib import Path
import socket
import tempfile
import time
import unittest
from unittest.mock import Mock

from rclpy.serialization import serialize_message

from c002ay0_raw_handoff_microgate import (
    BARRIER_TOPIC_ID,
    HEADER,
    MIN_COMPLETED_SAMPLES,
    MAX_COMPLETED_SAMPLES,
    PAYLOAD_CAP_BYTES,
    TOPIC_ID,
    decode_data_packet,
    encode_header,
    representative_execution_envelope,
    reference_gate_criteria,
    run_microgate,
    send_owned_raw_packet,
    stop_worker_bounded,
    wait_worker_progress,
    wait_worker_ready,
    wait_worker_result,
)


def crash_worker() -> None:
    os._exit(7)


def hang_worker() -> None:
    time.sleep(10.0)


class RawHandoffUnitTest(unittest.TestCase):
    def test_representative_has_declared_shape(self) -> None:
        message = representative_execution_envelope()
        self.assertEqual(len(message.witness.base_trajectory.points), 100)
        self.assertEqual(len(message.witness.applied_trajectory.points), 100)
        self.assertEqual(len(message.witness.rollout_samples), 100)
        self.assertLessEqual(
            len(serialize_message(message)),
            PAYLOAD_CAP_BYTES,
        )

    def test_oversize_is_hard_invalid_before_send(self) -> None:
        sender = Mock()
        with self.assertRaisesRegex(AssertionError, "oversize"):
            send_owned_raw_packet(
                sender,
                bytes(PAYLOAD_CAP_BYTES + 1),
                topic_id=TOPIC_ID,
                receive_index=1,
            )
        sender.sendmsg.assert_not_called()

    def test_eagain_and_partial_send_are_hard_invalid(self) -> None:
        sender = Mock()
        sender.sendmsg.side_effect = BlockingIOError()
        with self.assertRaisesRegex(AssertionError, "seqpacket_full"):
            send_owned_raw_packet(
                sender,
                b"x",
                topic_id=TOPIC_ID,
                receive_index=1,
            )
        sender.sendmsg.side_effect = None
        sender.sendmsg.return_value = HEADER.size
        with self.assertRaisesRegex(AssertionError, "partial_send"):
            send_owned_raw_packet(
                sender,
                b"x",
                topic_id=TOPIC_ID,
                receive_index=1,
            )

    def test_real_seqpacket_full_is_hard_invalid(self) -> None:
        sender, receiver = socket.socketpair(
            socket.AF_UNIX,
            socket.SOCK_SEQPACKET,
        )
        sender.setblocking(False)
        try:
            payload = bytes(16_384)
            index = 1
            while True:
                try:
                    send_owned_raw_packet(
                        sender,
                        payload,
                        topic_id=TOPIC_ID,
                        receive_index=index,
                    )
                    index += 1
                except AssertionError as error:
                    self.assertRegex(str(error), "seqpacket_full")
                    break
            self.assertGreater(index, 1)
        finally:
            sender.close()
            receiver.close()

    def test_index_gap_and_duplicate_are_hard_invalid(self) -> None:
        payload = b"payload"
        for index in (1, 3):
            with self.subTest(index=index):
                packet = encode_header(TOPIC_ID, index, payload) + payload
                with self.assertRaisesRegex(
                    AssertionError,
                    "index_mismatch",
                ):
                    decode_data_packet(packet, expected_index=2)

    def test_length_crc_and_unknown_topic_are_hard_invalid(self) -> None:
        payload = b"payload"
        bad_length = HEADER.pack(TOPIC_ID, 1, len(payload) + 1, 0)
        with self.assertRaisesRegex(AssertionError, "length_mismatch"):
            decode_data_packet(bad_length + payload, expected_index=1)
        bad_crc = HEADER.pack(TOPIC_ID, 1, len(payload), 0)
        with self.assertRaisesRegex(AssertionError, "crc_mismatch"):
            decode_data_packet(bad_crc + payload, expected_index=1)
        unknown = encode_header(99, 1, payload) + payload
        with self.assertRaisesRegex(AssertionError, "unknown_topic"):
            decode_data_packet(unknown, expected_index=1)

    def test_worker_crash_is_hard_invalid(self) -> None:
        parent, child = multiprocessing.Pipe(duplex=False)
        process = multiprocessing.Process(target=crash_worker)
        process.start()
        child.close()
        with self.assertRaisesRegex(AssertionError, "worker_failed"):
            wait_worker_result(process, parent, timeout_sec=0.1)
        process.join(1.0)
        self.assertFalse(process.is_alive())
        parent.close()

    def test_worker_hang_is_bounded_and_leaves_no_residue(self) -> None:
        parent, child = multiprocessing.Pipe(duplex=False)
        process = multiprocessing.Process(target=hang_worker)
        process.start()
        child.close()
        started = time.monotonic()
        with self.assertRaisesRegex(AssertionError, "result_timeout"):
            wait_worker_result(process, parent, timeout_sec=0.05)
        self.assertLess(time.monotonic() - started, 1.0)
        self.assertFalse(process.is_alive())
        parent.close()

    def test_stubborn_worker_residue_is_hard_invalid(self) -> None:
        process = Mock()
        process.is_alive.side_effect = (True, True, True)
        with self.assertRaisesRegex(
            AssertionError,
            "raw_handoff_worker_residue",
        ):
            stop_worker_bounded(process)
        process.terminate.assert_called_once_with()
        process.kill.assert_called_once_with()
        self.assertEqual(process.join.call_count, 2)

    def test_ready_timeout_stops_worker_without_residue(self) -> None:
        parent, child = multiprocessing.Pipe(duplex=False)
        process = multiprocessing.Process(target=hang_worker)
        process.start()
        child.close()
        with self.assertRaisesRegex(AssertionError, "ready_timeout"):
            wait_worker_ready(
                process,
                parent,
                timeout_sec=0.05,
            )
        self.assertFalse(process.is_alive())
        parent.close()

    def test_progress_timeout_stops_worker_without_residue(self) -> None:
        parent, child = multiprocessing.Pipe(duplex=False)
        process = multiprocessing.Process(target=hang_worker)
        process.start()
        child.close()
        with self.assertRaisesRegex(AssertionError, "progress_timeout"):
            wait_worker_progress(
                process,
                parent,
                expected_index=1,
                timeout_sec=0.05,
            )
        self.assertFalse(process.is_alive())
        parent.close()

    def test_ready_failure_preserves_cleanup_failure_reason(self) -> None:
        process = Mock()
        process.is_alive.side_effect = (True, True, True)
        connection = Mock()
        connection.poll.return_value = False
        with self.assertRaisesRegex(
            AssertionError,
            "ready_timeout.*cleanup_failure=.*worker_residue",
        ):
            wait_worker_ready(
                process,
                connection,
                timeout_sec=0.0,
            )

    def test_barrier_header_is_unambiguous(self) -> None:
        barrier = HEADER.pack(BARRIER_TOPIC_ID, 3, 0, 0)
        self.assertEqual(
            HEADER.unpack(barrier),
            (BARRIER_TOPIC_ID, 3, 0, 0),
        )

    def test_sample_floor_and_artifact_overwrite_are_fail_closed(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            existing = Path(temporary)
            with self.assertRaisesRegex(ValueError, "sample_count"):
                run_microgate(
                    existing / "unused",
                    sample_count=MIN_COMPLETED_SAMPLES - 1,
                )
            with self.assertRaisesRegex(ValueError, "exactly"):
                run_microgate(
                    existing / "unused",
                    sample_count=MAX_COMPLETED_SAMPLES + 1,
                )
            with self.assertRaisesRegex(
                FileExistsError,
                "refusing to overwrite",
            ):
                run_microgate(
                    existing,
                    sample_count=MIN_COMPLETED_SAMPLES,
                )

    def test_reference_gate_requires_thread_cpu_and_integrity(
        self,
    ) -> None:
        passing = {
            "p99_ns": 1_000_000,
            "p999_ns": 2_000_000,
        }
        failing_thread = {
            "p99_ns": 1_000_001,
            "p999_ns": 2_000_000,
        }
        criteria = reference_gate_criteria(
            sample_count=MIN_COMPLETED_SAMPLES,
            worker_result={
                "accepted": MIN_COMPLETED_SAMPLES,
                "processed": MIN_COMPLETED_SAMPLES,
                "drop_count": 0,
                "parity_success": True,
            },
            callback_wall=passing,
            callback_thread=failing_thread,
            spin_wall=passing,
            spin_thread=passing,
            worker_residue=False,
        )
        self.assertFalse(criteria["raw_callback_thread_cpu_p99"])
        self.assertFalse(all(criteria.values()))


@unittest.skipUnless(
    os.environ.get("C002AY0_RAW_HANDOFF_INTEGRATION") == "1",
    "DDS integration is an explicit Docker test",
)
class RawHandoffIntegrationTest(unittest.TestCase):
    def test_raw_subscription_worker_parity_and_bounded_final_drain(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            artifact_root = Path(temporary) / "raw-handoff"
            result = run_microgate(
                artifact_root,
                sample_count=MIN_COMPLETED_SAMPLES,
            )
            self.assertEqual(result["sample_count"], MIN_COMPLETED_SAMPLES)
            self.assertEqual(
                result["worker"]["accepted"],
                MIN_COMPLETED_SAMPLES,
            )
            self.assertEqual(result["worker"]["drop_count"], 0)
            self.assertFalse(result["worker"]["residue"])
            self.assertEqual(
                result["scope"],
                "single_handoff_feasibility",
            )
            self.assertTrue(result["per_sample_worker_ack"])
            self.assertFalse(
                result["sustained_100hz_throughput_proven"]
            )
            self.assertFalse(
                result["worker_canonical_throughput_included"]
            )
            self.assertFalse(result["m4_acceptance_credit"])
            self.assertTrue(
                all(result["reference_gate"]["criteria"].values())
            )
            self.assertTrue((artifact_root / "result.json").is_file())


if __name__ == "__main__":
    unittest.main()

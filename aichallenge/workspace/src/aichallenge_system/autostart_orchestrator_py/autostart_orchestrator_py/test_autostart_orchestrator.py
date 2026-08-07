#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


@dataclass
class _Counts:
    initial_pose: int = 0
    capture: int = 0
    control_mode_msgs: int = 0
    race_arm_msgs: int = 0


class _Harness(Node):
    def __init__(
        self,
        vehicle_state_topic: str,
        race_arm_topic: str,
        official_start_service: str,
    ) -> None:
        super().__init__("autostart_orchestrator_test_harness")

        self.counts = _Counts()
        self._counts_lock = threading.Lock()
        self._vehicle_state_topic = vehicle_state_topic

        vehicle_state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_state = self.create_publisher(
            String, vehicle_state_topic, vehicle_state_qos
        )
        self._sub_control_mode = self.create_subscription(
            Bool, "/awsim/control_mode_request_topic", self._on_control_mode, 10
        )
        race_arm_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sub_race_arm = self.create_subscription(
            Bool, race_arm_topic, self._on_race_arm, race_arm_qos
        )
        self._official_start_client = self.create_client(
            SetBool, official_start_service
        )

        self._initial_pose_svc = self.create_service(Trigger, "/set_initial_pose", self._on_initial_pose)
        self._capture_svc = self.create_service(Trigger, "/debug/service/capture_screen", self._on_capture)

        self._evt_initial_pose_called = threading.Event()
        self._evt_capture_called_once = threading.Event()
        self._evt_capture_called_twice = threading.Event()
        self._evt_control_mode_received = threading.Event()
        self._evt_race_armed = threading.Event()
        self._evt_race_disarmed_after_arm = threading.Event()
        self._race_armed_seen = False
        self._latest_race_arm = False
        self._race_arm_history: list[bool] = []

    def _on_initial_pose(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        with self._counts_lock:
            self.counts.initial_pose += 1
        self._evt_initial_pose_called.set()
        resp.success = True
        resp.message = "ok"
        return resp

    def _on_capture(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        with self._counts_lock:
            self.counts.capture += 1
            capture_calls = self.counts.capture
        if capture_calls >= 1:
            self._evt_capture_called_once.set()
        if capture_calls >= 2:
            self._evt_capture_called_twice.set()
        resp.success = True
        resp.message = "ok"
        return resp

    def _on_control_mode(self, _msg: Bool) -> None:
        with self._counts_lock:
            self.counts.control_mode_msgs += 1
        self._evt_control_mode_received.set()

    def _on_race_arm(self, msg: Bool) -> None:
        with self._counts_lock:
            self.counts.race_arm_msgs += 1
            self._latest_race_arm = bool(msg.data)
            self._race_arm_history.append(self._latest_race_arm)
            if msg.data:
                self._race_armed_seen = True
                self._evt_race_armed.set()
            elif self._race_armed_seen:
                self._evt_race_disarmed_after_arm.set()

    def publish_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self._pub_state.publish(msg)

    def request_official_start(self, requested: bool) -> tuple[bool, str]:
        if not self._official_start_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("official Start service was not available")
        request = SetBool.Request()
        request.data = bool(requested)
        future = self._official_start_client.call_async(request)
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError("official Start service call timed out")
        response = future.result()
        if response is None:
            raise RuntimeError("official Start service returned no response")
        return bool(response.success), str(response.message)


def _require_ros2() -> None:
    if shutil.which("ros2") is None:
        raise RuntimeError("ros2 command not found. Run this inside the container or source ROS 2 environment.")


def _run_orchestrator(
    *,
    params_file: Path,
) -> subprocess.Popen:
    cmd = [
        "ros2",
        "run",
        "autostart_orchestrator_py",
        "autostart_orchestrator_node.py",
        "--ros-args",
        "--params-file",
        str(params_file),
    ]

    env = dict(os.environ)
    env.setdefault("PYTHONUNBUFFERED", "1")
    env["ROS_LOG_DIR"] = os.environ.get("ROS_LOG_DIR", "/tmp")

    return subprocess.Popen(
        cmd,
        cwd=str(params_file.parent),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke test for autostart_orchestrator_py.")
    parser.add_argument("--vehicle-state-topic", default="/test/awsim/state")
    parser.add_argument("--start-on", default="Grounded")
    parser.add_argument("--stop-on", default="Finish")
    parser.add_argument("--race-arm-topic", default="/test/overtake/race_armed")
    parser.add_argument(
        "--official-start-service", default="/test/autostart/official_start"
    )
    parser.add_argument("--race-arm-on", default="Start")
    parser.add_argument("--timeout-sec", type=int, default=20)
    parser.add_argument("--output-dir", default="")
    args = parser.parse_args()

    timeout_sec = max(5, int(args.timeout_sec))
    output_dir = (args.output_dir or "").strip()
    if output_dir:
        out_dir = Path(output_dir)
    else:
        out_dir = Path("/tmp") / f"autostart_orchestrator_smoketest_{os.getpid()}"

    _require_ros2()

    os.environ.setdefault("ROS_LOG_DIR", f"/tmp/ros_log_autostart_orchestrator_test_{os.getpid()}")
    Path(os.environ["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)

    rclpy.init()
    harness = _Harness(
        args.vehicle_state_topic,
        args.race_arm_topic,
        args.official_start_service,
    )
    executor = MultiThreadedExecutor()
    executor.add_node(harness)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    proc: subprocess.Popen | None = None
    output_buf: list[str] = []
    output_lock = threading.Lock()
    try:
        out_dir.mkdir(parents=True, exist_ok=True)

        params_file = out_dir / "autostart_orchestrator_test_params.yaml"
        params_file.write_text(
            "\n".join(
                [
                    "/**:",
                    "  ros__parameters:",
                    f"    vehicle_state_topic: \"{args.vehicle_state_topic}\"",
                    f"    start_on_vehicle_state: \"{args.start_on}\"",
                    f"    stop_on_vehicle_state: \"{args.stop_on}\"",
                    f"    race_arm_topic: \"{args.race_arm_topic}\"",
                    f"    official_start_service: \"{args.official_start_service}\"",
                    f"    race_arm_on_vehicle_state: \"{args.race_arm_on}\"",
                    "    race_arm_neutral_vehicle_states: \"Ready\"",
                    f"    race_disarm_on_vehicle_state: \"Spawned,Grounded,{args.stop_on}\"",
                    "    enable_capture: true",
                    "    enable_rosbag: true",
                    "    call_initial_pose: true",
                    "    request_control_mode: true",
                    "    initial_pose_service: \"/set_initial_pose\"",
                    "    control_mode_request_topic: \"/awsim/control_mode_request_topic\"",
                    "    capture_service: \"/debug/service/capture_screen\"",
                    f"    rosbag_topics: [\"{args.vehicle_state_topic}\", \"{args.race_arm_topic}\"]",
                    f"    rosbag_required_nonempty_topics: [\"{args.race_arm_topic}\"]",
                    "    rosbag_output: \"rosbag2_autoware\"",
                    "    rosbag_storage_id: \"sqlite3\"",
                    "    rosbag_compression_format: \"\"",
                    "    rosbag_compression_mode: \"\"",
                    "    exit_on_finish: true",
                    "",
                ]
            ),
            encoding="utf-8",
        )

        proc = _run_orchestrator(params_file=params_file)

        t0 = time.monotonic()

        def timed_out() -> bool:
            return time.monotonic() - t0 > timeout_sec

        def read_output() -> None:
            if proc is None or proc.stdout is None:
                return
            for line in proc.stdout:
                with output_lock:
                    output_buf.append(line)

        reader_thread = threading.Thread(target=read_output, daemon=True)
        reader_thread.start()

        while not harness._evt_initial_pose_called.is_set() and proc.poll() is None and not timed_out():
            harness.publish_state(args.start_on)
            time.sleep(0.2)

        if not harness._evt_initial_pose_called.wait(timeout=5.0):
            raise RuntimeError("initial pose service was not called")

        if not harness._evt_control_mode_received.wait(timeout=5.0):
            raise RuntimeError("control mode request topic was not observed")

        while not harness._evt_capture_called_once.is_set() and proc.poll() is None and not timed_out():
            if args.start_on:
                harness.publish_state(args.start_on)
            time.sleep(0.2)

        if not harness._evt_capture_called_once.is_set():
            raise RuntimeError("capture service was not called (start phase)")

        # AWSIM vehicle state emits Start for the count before Ready.  It must
        # never arm a motion-affecting planner or consume initialization.
        pre_start_deadline = time.monotonic() + 0.5
        while pre_start_deadline > time.monotonic() and proc.poll() is None:
            harness.publish_state("Start")
            time.sleep(0.1)
        if harness._evt_race_armed.is_set():
            raise RuntimeError("race arm became true before Ready")
        early_accepted, early_reason = harness.request_official_start(True)
        if early_accepted or early_reason != "official_start_before_neutral":
            raise RuntimeError(
                "official Start before Ready was not rejected without side effects: "
                f"accepted={early_accepted} reason={early_reason}"
            )

        ready_deadline = time.monotonic() + 0.5
        while ready_deadline > time.monotonic() and proc.poll() is None:
            harness.publish_state("Ready")
            time.sleep(0.1)

        arm_on_states = {
            state.strip().lower()
            for state in args.race_arm_on.split(",")
            if state.strip()
        }
        if "ready" not in arm_on_states:
            accepted, reason = harness.request_official_start(True)
            if not accepted or reason != "official_start":
                raise RuntimeError(
                    "official Start was rejected after Ready: "
                    f"accepted={accepted} reason={reason}"
                )

        duplicate_accepted, duplicate_reason = harness.request_official_start(True)
        if not duplicate_accepted or duplicate_reason != "already_armed":
            raise RuntimeError(
                "duplicate official Start was not idempotent: "
                f"accepted={duplicate_accepted} reason={duplicate_reason}"
            )

        if not harness._evt_race_armed.wait(timeout=3.0):
            raise RuntimeError(
                "race arm did not become true at Ready or the official Start service"
            )

        # Readyは走行中にも再送され得る中立状態であり、arm後に落ちない。
        ready_latch_deadline = time.monotonic() + 1.0
        while time.monotonic() < ready_latch_deadline and proc.poll() is None:
            harness.publish_state("Ready")
            time.sleep(0.1)
        if harness._evt_race_disarmed_after_arm.is_set():
            raise RuntimeError("race arm returned false when Start transitioned to Ready")
        with harness._counts_lock:
            if not harness._latest_race_arm:
                raise RuntimeError("race arm was not latched true through Ready")

        while not harness._evt_capture_called_twice.is_set() and proc.poll() is None and not timed_out():
            harness.publish_state(args.stop_on)
            time.sleep(0.2)

        if not harness._evt_capture_called_twice.is_set():
            raise RuntimeError("capture service was not called (stop phase)")
        if not harness._evt_race_disarmed_after_arm.wait(timeout=3.0):
            raise RuntimeError("race arm did not return false at Finish")

        if proc.poll() is None:
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                raise RuntimeError("orchestrator did not exit after stop phase") from None

        rc = int(proc.returncode or 0)
        if rc != 0:
            raise RuntimeError(f"orchestrator exited with non-zero code: {rc}")

        log_path = out_dir / "rosbag_autostart.log"
        if not log_path.exists():
            raise RuntimeError(f"rosbag log was not created: {log_path}")
        if "Recording..." not in log_path.read_text(errors="replace"):
            raise RuntimeError("rosbag did not report an active recording")
        if not (out_dir / "rosbag2_autoware" / "metadata.yaml").exists():
            raise RuntimeError("rosbag metadata was not created")

        with harness._counts_lock:
            counts = harness.counts
        print("PASS")
        print(f"  initial_pose_calls={counts.initial_pose}")
        print(f"  capture_calls={counts.capture}")
        print(f"  control_mode_msgs={counts.control_mode_msgs}")
        print(f"  race_arm_msgs={counts.race_arm_msgs}")
        print(f"  race_arm_history={harness._race_arm_history}")
        print(f"  rosbag_log={log_path}")
        return 0
    except Exception as e:
        print("FAIL")
        print(f"  error={e}")
        with output_lock:
            tail = output_buf[-50:]
        if tail:
            print("  orchestrator_output_tail:")
            for line in tail:
                sys.stdout.write(f"    {line}")
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()
        return 1
    finally:
        try:
            executor.shutdown()
        finally:
            harness.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

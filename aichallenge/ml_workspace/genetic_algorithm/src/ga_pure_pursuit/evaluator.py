from __future__ import annotations

import json
import math
import os
import random
import queue
import copy
import subprocess
import threading
import time
import uuid
from concurrent.futures import Future, ThreadPoolExecutor, TimeoutError as FutureTimeoutError
from pathlib import Path
from typing import Any, Protocol

from .path_optimization import generate_candidate_path, split_genome


class InfrastructureError(RuntimeError):
    pass


class Evaluator(Protocol):
    def evaluate(
        self, candidate_id: str, parameter_hash: str, parameters: dict[str, float], repeat: int
    ) -> dict[str, Any]: ...


class MockEvaluator:
    """Deterministic surrogate used to validate GA plumbing without AWSIM."""

    optimum = {
        "lookahead_gain": 0.47,
        "lookahead_min_distance": 2.85,
        "steering_tire_angle_gain": 1.61,
        "curvature_lookahead_min_distance": 1.9,
        "curvature_lookahead_sensitivity": 7.4,
        "curvature_lookahead_smoothing_alpha": 0.58,
        "max_lateral_acceleration": 7.0,
        "minimum_corner_speed": 5.5,
        "curvature_speed_preview_distance": 14.0,
        "speed_proportional_gain": 0.92,
    }

    def __init__(self, seed: int):
        self.seed = seed

    def evaluate(self, candidate_id, parameter_hash, parameters, repeat):
        distance = 0.0
        for name, target in self.optimum.items():
            if name in parameters:
                scale = max(abs(target), 1.0)
                distance += ((parameters[name] - target) / scale) ** 2
        noise_rng = random.Random(f"{self.seed}:{parameter_hash}:{repeat}")
        noise = noise_rng.uniform(-0.015, 0.015)
        unstable = (
            parameters.get("lookahead_min_distance", 99.0) < 1.7 or
            (
                parameters.get("max_lateral_acceleration", 0.0) > 9.0 and
                parameters.get("curvature_speed_preview_distance", 99.0) < 8.0
            )
        )
        return {
            "completed": not unstable,
            "invalid": False,
            "lap_time_seconds": 62.0 + 18.0 * distance + noise,
            "elapsed_seconds": 62.0 + 18.0 * distance + noise,
            "progress": 0.65 if unstable else 1.0,
            "collision_count": int(unstable),
            "wall_count": 0,
            "over_count": 0,
            "penalty_seconds": 0.0,
            "lateral_error_p95_m": 0.12 + 0.5 * math.sqrt(distance),
            "steering_delta_rms": 0.02 + 0.1 * distance,
            "steering_rate_limit_ratio": min(1.0, 0.01 + 0.1 * distance),
            "exit_reason": "collision" if unstable else "completed",
        }


class CommandEvaluator:
    """Runs an integration adapter that reads a request and emits metrics JSON."""

    def __init__(
        self,
        command: list[str],
        timeout_sec: float,
        run_dir: Path,
        episode_options: dict[str, Any] | None = None,
        extra_env: dict[str, str] | None = None,
    ):
        if not command:
            raise ValueError("evaluator.episode_command is required in command mode")
        self.command = command
        self.timeout_sec = timeout_sec
        self.run_dir = run_dir
        self.episode_options = episode_options or {}
        self.extra_env = extra_env or {}

    def evaluate(self, candidate_id, parameter_hash, parameters, repeat):
        episode_dir = self.run_dir / "episodes" / f"{candidate_id}-r{repeat:02d}"
        episode_dir.mkdir(parents=True, exist_ok=True)
        request_path = episode_dir / "request.json"
        result_path = episode_dir / "metrics.json"
        request_path.write_text(
            json.dumps(
                {
                    "candidate_id": candidate_id,
                    "parameter_hash": parameter_hash,
                    "repeat": repeat,
                    "parameters": parameters,
                    "episode_options": self.episode_options,
                    "result_path": str(result_path),
                },
                indent=2,
                sort_keys=True,
            ),
            encoding="utf-8",
        )
        env = os.environ.copy()
        env.update(self.extra_env)
        env["GA_EPISODE_REQUEST"] = str(request_path)
        env["GA_EPISODE_RESULT"] = str(result_path)
        try:
            completed = subprocess.run(
                self.command,
                env=env,
                cwd=self.run_dir,
                text=True,
                capture_output=True,
                timeout=self.timeout_sec,
                check=False,
            )
        except subprocess.TimeoutExpired as error:
            raise InfrastructureError(f"episode adapter timed out: {error}") from error
        (episode_dir / "stdout.log").write_text(completed.stdout, encoding="utf-8")
        (episode_dir / "stderr.log").write_text(completed.stderr, encoding="utf-8")
        if completed.returncode != 0:
            raise InfrastructureError(f"episode adapter exited {completed.returncode}")
        if result_path.exists():
            return json.loads(result_path.read_text(encoding="utf-8"))
        try:
            return json.loads(completed.stdout)
        except json.JSONDecodeError as error:
            raise InfrastructureError("episode adapter produced no metrics JSON") from error


class Ros2BatchSync:
    """Two-phase barrier for one shared-AWSIM candidate batch."""

    def __init__(self, parties: int, reset_action, start_action, timeout_sec: float):
        self.timeout_sec = timeout_sec
        self.reset_barrier = threading.Barrier(parties, action=reset_action)
        self.start_barrier = threading.Barrier(parties, action=start_action)

    def _wait(self, barrier: threading.Barrier, phase: str) -> None:
        try:
            barrier.wait(timeout=self.timeout_sec)
        except threading.BrokenBarrierError as error:
            raise InfrastructureError(f"shared AWSIM {phase} barrier failed") from error

    def wait_reset(self) -> None:
        self._wait(self.reset_barrier, "reset")

    def wait_start(self) -> None:
        self._wait(self.start_barrier, "start")

    def abort(self) -> None:
        self.reset_barrier.abort()
        self.start_barrier.abort()


class Ros2Evaluator(CommandEvaluator):
    """Applies a candidate atomically, resets AWSIM, then invokes an episode adapter."""

    def __init__(
        self,
        config: dict[str, Any],
        timeout_sec: float,
        run_dir: Path,
        batch_sync: Ros2BatchSync | None = None,
    ):
        vehicle_domain = int(config["ros"].get("vehicle_domain_id", 1))
        super().__init__(
            config["episode_command"],
            timeout_sec,
            run_dir,
            config.get("episode_options"),
            {"ROS_DOMAIN_ID": str(vehicle_domain)},
        )
        self.ros = config["ros"]
        self.fixed_controller_parameters = {
            str(name): value
            for name, value in config.get("fixed_controller_parameters", {}).items()
        }
        self.path_optimization = config.get("path_optimization")
        self.run_id = run_dir.name
        self.batch_sync = batch_sync

    @staticmethod
    def _parameter_entry(name: str, value: Any) -> dict[str, Any]:
        if isinstance(value, bool):
            return {"name": name, "value": {"type": 1, "bool_value": value}}
        if isinstance(value, str):
            return {"name": name, "value": {"type": 4, "string_value": value}}
        return {"name": name, "value": {"type": 3, "double_value": float(value)}}

    def _run_ros(self, command: list[str], domain_id: int, timeout: float = 20.0) -> None:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(domain_id)
        completed = subprocess.run(
            command, env=env, text=True, capture_output=True, timeout=timeout, check=False
        )
        compact_output = completed.stdout.replace(" ", "").lower()
        if completed.returncode != 0 or any(
            marker in compact_output
            for marker in ("successful:false", "success:false", "successful=false", "success=false")
        ):
            raise InfrastructureError(
                f"ROS command failed: {' '.join(command)}\n{completed.stdout}\n{completed.stderr}"
            )

    def evaluate(self, candidate_id, parameter_hash, parameters, repeat):
        node = self.ros["node_name"].rstrip("/")
        vehicle_domain = int(self.ros.get("vehicle_domain_id", 1))
        controller_parameters, path_genes = split_genome(parameters)
        controller_parameters.update(self.fixed_controller_parameters)
        path_metrics: dict[str, float] = {}
        candidate_path: Path | None = None
        base_path: Path | None = None
        if self.path_optimization:
            path_config = self.path_optimization
            base_path = Path(path_config["base_csv_path"])
            candidate_path = (
                self.run_dir / "generated_paths" /
                f"{candidate_id}-r{repeat:02d}.csv"
            )
            path_metrics = generate_candidate_path(
                base_path,
                candidate_path,
                path_genes,
                anchor_count=int(path_config["anchor_count"]),
                loop_start_index=int(path_config["loop_start_index"]),
                prefix=str(path_config.get("gene_prefix", "path_offset_")),
            )
        self._run_ros(
            ["ros2", "service", "call", f"{node}/ga/set_enabled", "std_srvs/srv/SetBool", "{data: false}"],
            vehicle_domain,
        )
        entries = [
            self._parameter_entry("ga_run_id", self.run_id),
            self._parameter_entry("ga_candidate_id", candidate_id),
            self._parameter_entry("ga_parameter_hash", parameter_hash),
            *[
                self._parameter_entry(name, value)
                for name, value in controller_parameters.items()
            ],
        ]
        request = json.dumps({"parameters": entries}, separators=(",", ":"))
        self._run_ros(
            [
                "ros2",
                "service",
                "call",
                f"{node}/set_parameters_atomically",
                "rcl_interfaces/srv/SetParametersAtomically",
                request,
            ],
            vehicle_domain,
        )
        if candidate_path is not None:
            trajectory_node = str(
                self.path_optimization.get(
                    "trajectory_node",
                    "/planning/scenario_planning/simple_trajectory_generator",
                )
            )
            self._run_ros(
                [
                    "ros2", "param", "set", trajectory_node, "csv_path",
                    str(candidate_path),
                ],
                vehicle_domain,
            )
        reset_command = list(self.ros.get("reset_command", []))
        if reset_command:
            if self.batch_sync is not None:
                self.batch_sync.wait_reset()
            else:
                self._run_ros(reset_command, int(self.ros.get("admin_domain_id", 0)), 30.0)
        for command in self.ros.get("vehicle_prepare_commands", []):
            self._run_ros(list(command), vehicle_domain, 30.0)
        ready_command = list(self.ros.get("ready_command", []))
        if ready_command:
            self._run_ros(ready_command, vehicle_domain, 45.0)
        self._run_ros(
            ["ros2", "service", "call", f"{node}/ga/reset_state", "std_srvs/srv/Trigger", "{}"],
            vehicle_domain,
        )
        start_command = list(self.ros.get("start_command", []))
        if self.batch_sync is not None:
            self.batch_sync.wait_start()
        elif start_command:
            self._run_ros(start_command, int(self.ros.get("admin_domain_id", 0)), 30.0)
        self._run_ros(
            ["ros2", "service", "call", f"{node}/ga/set_enabled", "std_srvs/srv/SetBool", "{data: true}"],
            vehicle_domain,
        )
        try:
            metrics = super().evaluate(
                candidate_id, parameter_hash, controller_parameters, repeat
            )
            metrics.update(path_metrics)
            if candidate_path is not None:
                metrics["candidate_path"] = str(candidate_path)
            return metrics
        finally:
            self._run_ros(
                ["ros2", "service", "call", f"{node}/ga/set_enabled", "std_srvs/srv/SetBool", "{data: false}"],
                vehicle_domain,
            )
            if base_path is not None:
                trajectory_node = str(
                    self.path_optimization.get(
                        "trajectory_node",
                        "/planning/scenario_planning/simple_trajectory_generator",
                    )
                )
                self._run_ros(
                    [
                        "ros2", "param", "set", trajectory_node, "csv_path",
                        str(base_path),
                    ],
                    vehicle_domain,
                )


class SharedAwsimBatchEvaluator:
    """Evaluate synchronized batches across one or more shared AWSIM instances."""

    def __init__(self, config: dict[str, Any], timeout_sec: float, run_dir: Path):
        self.config = config
        self.timeout_sec = timeout_sec
        self.run_dir = run_dir
        configured_environments = config.get("environments")
        if configured_environments:
            self.environments = []
            for index, item in enumerate(configured_environments, start=1):
                domains = [int(value) for value in item.get("vehicle_domain_ids", [])]
                if not domains:
                    raise ValueError(
                        f"evaluator.environments[{index - 1}].vehicle_domain_ids "
                        "must not be empty"
                    )
                self.environments.append(
                    {
                        "name": str(item.get("name", f"env{index}")),
                        "admin_domain_id": int(item.get("admin_domain_id", 0)),
                        "vehicle_domain_ids": domains,
                    }
                )
        else:
            domains = [
                int(value)
                for value in config.get("vehicle_domain_ids", [1, 2, 3, 4])
            ]
            if not domains:
                raise ValueError("evaluator.vehicle_domain_ids must not be empty")
            self.environments = [
                {
                    "name": "env1",
                    "admin_domain_id": int(
                        config.get("ros", {}).get("admin_domain_id", 0)
                    ),
                    "vehicle_domain_ids": domains,
                }
            ]
        self.capacity = sum(
            len(item["vehicle_domain_ids"]) for item in self.environments
        )
        self.collect_timeout_sec = float(config.get("batch_collect_timeout_sec", 0.5))
        self.barrier_timeout_sec = float(config.get("batch_barrier_timeout_sec", 90.0))
        self.requests: queue.Queue[tuple[tuple[Any, ...], Future]] = queue.Queue()
        self.worker = threading.Thread(target=self._batch_loop, daemon=True)
        self.worker.start()

    def evaluate(self, candidate_id, parameter_hash, parameters, repeat):
        future: Future = Future()
        self.requests.put(((candidate_id, parameter_hash, parameters, repeat), future))
        try:
            return future.result(timeout=self.timeout_sec + self.barrier_timeout_sec + 30.0)
        except FutureTimeoutError as error:
            raise InfrastructureError(
                f"shared AWSIM batch timed out for {candidate_id}"
            ) from error

    def _batch_loop(self) -> None:
        while True:
            first = self.requests.get()
            batch = [first]
            deadline = time.monotonic() + self.collect_timeout_sec
            while len(batch) < self.capacity:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                try:
                    batch.append(self.requests.get(timeout=remaining))
                except queue.Empty:
                    break
            try:
                metrics = self._execute_batch([request for request, _ in batch])
                if len(metrics) != len(batch):
                    raise InfrastructureError("shared AWSIM batch returned wrong result count")
                for (_, future), result in zip(batch, metrics):
                    future.set_result(result)
            except BaseException as error:  # Never strand optimizer threads.
                failure = error if isinstance(error, InfrastructureError) else InfrastructureError(str(error))
                for _, future in batch:
                    future.set_exception(failure)

    def _execute_environment(
        self, requests: list[tuple[Any, ...]], environment: dict[str, Any]
    ) -> list[dict[str, Any]]:
        evaluators: list[Ros2Evaluator] = []
        ros = copy.deepcopy(self.config["ros"])
        ros["admin_domain_id"] = int(environment["admin_domain_id"])
        reset_command = list(ros.get("reset_command", []))
        start_command = list(ros.get("start_command", []))

        def run_admin(command: list[str]) -> None:
            if command:
                evaluators[0]._run_ros(
                    command, int(ros.get("admin_domain_id", 0)), 30.0
                )

        sync = Ros2BatchSync(
            len(requests),
            lambda: run_admin(reset_command),
            lambda: run_admin(start_command),
            self.barrier_timeout_sec,
        )
        for domain in environment["vehicle_domain_ids"][: len(requests)]:
            child_config = copy.deepcopy(self.config)
            child_config["ros"] = copy.deepcopy(ros)
            child_config["ros"]["vehicle_domain_id"] = int(domain)
            evaluators.append(
                Ros2Evaluator(child_config, self.timeout_sec, self.run_dir, sync)
            )

        def run_one(item):
            evaluator, request = item
            try:
                return evaluator.evaluate(*request)
            except BaseException:
                sync.abort()
                raise

        with ThreadPoolExecutor(max_workers=len(requests)) as executor:
            return list(executor.map(run_one, zip(evaluators, requests)))

    def _set_domain_idle(self, domain_id: int) -> None:
        """Disable an unassigned vehicle and expose its idle state to RViz."""
        node = self.config["ros"]["node_name"].rstrip("/")
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(domain_id)
        commands = (
            [
                "ros2", "service", "call", f"{node}/ga/set_enabled",
                "std_srvs/srv/SetBool", "{data: false}",
            ],
            ["ros2", "param", "set", node, "ga_candidate_id", "IDLE"],
        )
        for command in commands:
            completed = subprocess.run(
                command,
                env=env,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            compact_output = completed.stdout.replace(" ", "").lower()
            if completed.returncode != 0 or any(
                marker in compact_output
                for marker in (
                    "successful:false", "success:false",
                    "successful=false", "success=false",
                )
            ):
                raise InfrastructureError(
                    f"failed to mark ROS domain {domain_id} idle: "
                    f"{' '.join(command)}\n{completed.stdout}\n{completed.stderr}"
                )

    def _execute_batch(self, requests: list[tuple[Any, ...]]) -> list[dict[str, Any]]:
        assignments: list[tuple[list[tuple[Any, ...]], dict[str, Any]]] = []
        offset = 0
        for environment in self.environments:
            size = min(
                len(environment["vehicle_domain_ids"]), len(requests) - offset
            )
            if size <= 0:
                break
            assignments.append((requests[offset: offset + size], environment))
            offset += size
        if offset != len(requests):
            raise InfrastructureError(
                f"shared AWSIM pool capacity {self.capacity} is smaller than "
                f"batch size {len(requests)}"
            )
        assigned_domains = {
            int(domain)
            for assigned_requests, environment in assignments
            for domain in environment["vehicle_domain_ids"][:len(assigned_requests)]
        }
        idle_domains = [
            int(domain)
            for environment in self.environments
            for domain in environment["vehicle_domain_ids"]
            if int(domain) not in assigned_domains
        ]
        if idle_domains:
            with ThreadPoolExecutor(max_workers=len(idle_domains)) as executor:
                list(executor.map(self._set_domain_idle, idle_domains))
        with ThreadPoolExecutor(max_workers=len(assignments)) as executor:
            grouped = list(
                executor.map(
                    lambda item: self._execute_environment(item[0], item[1]),
                    assignments,
                )
            )
        return [result for group in grouped for result in group]


class WorkerPoolEvaluator:
    """Dispatches ROS episodes to isolated worker containers via atomic files."""

    def __init__(self, config: dict[str, Any], timeout_sec: float, run_dir: Path):
        worker_ids = [str(value) for value in config.get("worker_ids", [])]
        if not worker_ids:
            raise ValueError("evaluator.worker_ids must not be empty")
        self.available: queue.Queue[str] = queue.Queue()
        for worker_id in worker_ids:
            self.available.put(worker_id)
        self.timeout_sec = timeout_sec
        self.poll_interval_sec = float(config.get("poll_interval_sec", 0.1))
        self.ipc_root = Path(config["ipc_root"])
        self.run_dir = run_dir
        for worker_id in worker_ids:
            worker_root = self.ipc_root / worker_id
            for directory_name in ("requests", "running", "results", "failed"):
                (worker_root / directory_name).mkdir(parents=True, exist_ok=True)

    @staticmethod
    def _atomic_json(path: Path, data: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + f".{uuid.uuid4().hex}.tmp")
        temporary.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")
        temporary.replace(path)

    def evaluate(self, candidate_id, parameter_hash, parameters, repeat):
        worker_id = self.available.get()
        try:
            job_id = uuid.uuid4().hex
            worker_root = self.ipc_root / worker_id
            request_path = worker_root / "requests" / f"{job_id}.json"
            result_path = worker_root / "results" / f"{job_id}.json"
            failed_path = worker_root / "failed" / f"{job_id}.json"
            self._atomic_json(
                request_path,
                {
                    "job_id": job_id,
                    "run_dir": str(self.run_dir),
                    "candidate_id": candidate_id,
                    "parameter_hash": parameter_hash,
                    "parameters": parameters,
                    "repeat": repeat,
                },
            )
            deadline = time.monotonic() + self.timeout_sec
            while time.monotonic() < deadline:
                if result_path.exists():
                    try:
                        result = json.loads(result_path.read_text(encoding="utf-8"))
                    except json.JSONDecodeError:
                        time.sleep(self.poll_interval_sec)
                        continue
                    result_path.unlink(missing_ok=True)
                    return result
                if failed_path.exists():
                    try:
                        failure = json.loads(failed_path.read_text(encoding="utf-8"))
                    except json.JSONDecodeError:
                        time.sleep(self.poll_interval_sec)
                        continue
                    failed_path.unlink(missing_ok=True)
                    raise InfrastructureError(
                        f"worker {worker_id} failed job {job_id}: {failure.get('error')}"
                    )
                time.sleep(self.poll_interval_sec)
            raise InfrastructureError(
                f"worker {worker_id} timed out after {self.timeout_sec}s for job {job_id}"
            )
        finally:
            self.available.put(worker_id)


def make_evaluator(config: dict[str, Any], run_dir: Path) -> Evaluator:
    evaluator = config["evaluator"]
    mode = evaluator.get("mode", "mock")
    if mode == "mock":
        return MockEvaluator(int(config["run"]["seed"]))
    if mode == "command":
        return CommandEvaluator(
            list(evaluator.get("episode_command", [])),
            float(evaluator["timeout_sec"]),
            run_dir,
        )
    if mode == "ros2":
        return Ros2Evaluator(evaluator, float(evaluator["timeout_sec"]), run_dir)
    if mode == "worker_pool":
        return WorkerPoolEvaluator(evaluator, float(evaluator["timeout_sec"]), run_dir)
    if mode == "shared_awsim_batch":
        return SharedAwsimBatchEvaluator(
            evaluator, float(evaluator["episode_timeout_sec"]), run_dir
        )
    raise ValueError(f"unknown evaluator mode: {mode}")

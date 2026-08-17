#!/usr/bin/env python3

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional


EXIT_CONTINUOUS_STOP = 42
EXIT_PROGRESS_EVIDENCE = 43
EXIT_RACE_ABORTED = 44


@dataclass(frozen=True)
class WatchdogConfig:
    stop_timeout_sec: float = 15.0
    stop_enter_speed_mps: float = 0.05
    stop_exit_speed_mps: float = 0.10
    velocity_freshness_sec: float = 1.0
    pose_freshness_sec: float = 1.0
    evidence_failure_sec: float = 15.0
    source_stamp_max_age_sec: float = 1.0
    source_stamp_future_tolerance_sec: float = 0.10
    pose_progress_min_m: float = 0.10
    pose_max_step_m: float = 2.0
    finish_ordering_grace_sec: float = 1.0
    race_arm_confirmation_timeout_sec: float = 5.0

    def validate(self) -> None:
        values = (
            self.stop_timeout_sec,
            self.stop_enter_speed_mps,
            self.stop_exit_speed_mps,
            self.velocity_freshness_sec,
            self.pose_freshness_sec,
            self.evidence_failure_sec,
            self.source_stamp_max_age_sec,
            self.source_stamp_future_tolerance_sec,
            self.pose_progress_min_m,
            self.pose_max_step_m,
            self.finish_ordering_grace_sec,
            self.race_arm_confirmation_timeout_sec,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("watchdog thresholds must be finite")
        if self.stop_timeout_sec <= 0.0:
            raise ValueError("stop_timeout_sec must be positive")
        if self.stop_enter_speed_mps < 0.0:
            raise ValueError("stop_enter_speed_mps must be non-negative")
        if self.stop_exit_speed_mps <= self.stop_enter_speed_mps:
            raise ValueError("stop_exit_speed_mps must be greater than stop_enter_speed_mps")
        if self.velocity_freshness_sec <= 0.0:
            raise ValueError("velocity_freshness_sec must be positive")
        if self.pose_freshness_sec <= 0.0:
            raise ValueError("pose_freshness_sec must be positive")
        if self.evidence_failure_sec <= 0.0:
            raise ValueError("evidence_failure_sec must be positive")
        if self.source_stamp_max_age_sec <= 0.0:
            raise ValueError("source_stamp_max_age_sec must be positive")
        if self.source_stamp_future_tolerance_sec < 0.0:
            raise ValueError("source_stamp_future_tolerance_sec must be non-negative")
        if self.pose_progress_min_m <= 0.0:
            raise ValueError("pose_progress_min_m must be positive")
        if self.pose_max_step_m <= self.pose_progress_min_m:
            raise ValueError("pose_max_step_m must be greater than pose_progress_min_m")
        if self.finish_ordering_grace_sec <= 0.0:
            raise ValueError("finish_ordering_grace_sec must be positive")
        if self.race_arm_confirmation_timeout_sec <= 0.0:
            raise ValueError("race_arm_confirmation_timeout_sec must be positive")


@dataclass(frozen=True)
class WatchdogVerdict:
    reason: str
    exit_code: int
    passed: bool
    detected_monotonic_sec: float


class D1ProgressWatchdog:
    """Pure state machine for post-Start D1 progress evidence."""

    _RUNNING_STATES = {"start", "lapcomplete"}
    _FINISH_STATES = {"finish", "finished", "finishall", "finishedall"}
    _ABORT_STATES = {"terminate", "terminated"}
    _RESET_STATES = {"grounded", "reset", "selectmode", "waitstart"}

    def __init__(
        self,
        config: WatchdogConfig,
        *,
        authoritative_start_confirmed: bool = False,
        authority_started_sec: Optional[float] = None,
    ) -> None:
        config.validate()
        self.config = config
        self.authoritative_start_confirmed = bool(authoritative_start_confirmed)
        self.authority_started_sec = (
            float(authority_started_sec)
            if self.authoritative_start_confirmed
            and authority_started_sec is not None
            else None
        )
        self.vehicle_state = ""
        self.race_armed = False
        self.race_arm_seen = False
        self.running_seen = False
        self.motion_state_seen = False
        self.last_speed_mps: Optional[float] = None
        self.last_source_stamp_sec: Optional[float] = None
        self.last_valid_velocity_receipt_sec: Optional[float] = None
        self.last_pose_source_stamp_sec: Optional[float] = None
        self.last_valid_pose_receipt_sec: Optional[float] = None
        self.last_pose_x_m: Optional[float] = None
        self.last_pose_y_m: Optional[float] = None
        self.last_pose_yaw_rad: Optional[float] = None
        self.pose_anchor_x_m: Optional[float] = None
        self.pose_anchor_y_m: Optional[float] = None
        self.pose_anchor_yaw_rad: Optional[float] = None
        self.pose_frame_id: Optional[str] = None
        self.last_pose_progress_sec: Optional[float] = None
        self.last_clock_sec: Optional[float] = None
        self.last_clock_receipt_sec: Optional[float] = None
        self.stationary_since_sec: Optional[float] = None
        self.velocity_evidence_invalid_since_sec: Optional[float] = None
        self.pose_evidence_invalid_since_sec: Optional[float] = None
        self.progress_evidence_invalid_since_sec: Optional[float] = None
        self.velocity_pose_inconsistent_since_sec: Optional[float] = None
        self.sample_count = 0
        self.pose_sample_count = 0
        self.pose_progress_count = 0
        self.stationary_sample_count = 0
        self.non_monotonic_stamp_count = 0
        self.non_monotonic_pose_stamp_count = 0
        self.pose_discontinuity_count = 0
        self.velocity_pose_inconsistent_count = 0
        self.pending_abort_reason: Optional[str] = None
        self.pending_abort_since_sec: Optional[float] = None
        self._terminal: Optional[WatchdogVerdict] = None

    @property
    def terminal(self) -> Optional[WatchdogVerdict]:
        return self._terminal

    def _latch(
        self, reason: str, exit_code: int, passed: bool, now_sec: float
    ) -> WatchdogVerdict:
        if self._terminal is None:
            self._terminal = WatchdogVerdict(reason, exit_code, passed, now_sec)
        return self._terminal

    def observe_race_arm(
        self, armed: bool, now_sec: float
    ) -> Optional[WatchdogVerdict]:
        self.race_armed = bool(armed)
        if self.race_armed:
            self.race_arm_seen = True
            if self.pending_abort_reason == "RACE_DISARMED_BEFORE_FINISH":
                self._clear_pending_abort()
            return self._terminal
        self._reset_stop_evidence()
        if self.race_arm_seen and self.running_seen and self.vehicle_state not in self._FINISH_STATES:
            self._begin_pending_abort("RACE_DISARMED_BEFORE_FINISH", now_sec)
        return self._terminal

    def observe_vehicle_state(
        self, state: str, now_sec: float
    ) -> Optional[WatchdogVerdict]:
        self.vehicle_state = str(state).strip().lower()
        if self.vehicle_state in self._FINISH_STATES:
            if (
                not self.authoritative_start_confirmed
                or not self.race_arm_seen
                or not self.running_seen
            ):
                return self._latch(
                    "FINISH_WITHOUT_CONFIRMED_START",
                    EXIT_RACE_ABORTED,
                    False,
                    now_sec,
                )
            return self._latch("D1_FINISH", 0, True, now_sec)
        if self.vehicle_state in self._ABORT_STATES:
            return self._latch(
                "D1_TERMINATED_BEFORE_FINISH",
                EXIT_RACE_ABORTED,
                False,
                now_sec,
            )
        if self.vehicle_state in self._RUNNING_STATES:
            self.running_seen = True
            self.motion_state_seen = True
            return self._terminal
        if self.authoritative_start_confirmed and self.vehicle_state == "ready":
            if self.motion_state_seen:
                self._reset_stop_evidence()
                self._begin_pending_abort("D1_RESET_BEFORE_FINISH", now_sec)
                return self._terminal
            self.running_seen = True
            return self._terminal
        self._reset_stop_evidence()
        if self.running_seen and self.vehicle_state in self._RESET_STATES:
            self._begin_pending_abort("D1_RESET_BEFORE_FINISH", now_sec)
        elif self.authoritative_start_confirmed and self.vehicle_state:
            self._begin_pending_abort("D1_STATE_INVALID_AFTER_START", now_sec)
        return self._terminal

    def observe_clock(
        self, clock_sec: float, now_sec: float
    ) -> Optional[WatchdogVerdict]:
        if self._terminal is not None:
            return self._terminal
        if not math.isfinite(clock_sec):
            self._mark_velocity_invalid(now_sec)
            self._mark_pose_invalid(now_sec)
            return self.evaluate(now_sec)
        if self.last_clock_sec is not None and clock_sec < self.last_clock_sec:
            self._mark_velocity_invalid(now_sec)
            self._mark_pose_invalid(now_sec)
            return self.evaluate(now_sec)
        self.last_clock_sec = float(clock_sec)
        self.last_clock_receipt_sec = float(now_sec)
        return self._terminal

    def observe_velocity(
        self, speed_mps: float, source_stamp_sec: float, now_sec: float
    ) -> Optional[WatchdogVerdict]:
        if self._terminal is not None:
            return self._terminal
        if not math.isfinite(speed_mps) or not math.isfinite(source_stamp_sec):
            self._mark_velocity_invalid(now_sec)
            return self.evaluate(now_sec)
        if (
            self.last_source_stamp_sec is not None
            and source_stamp_sec <= self.last_source_stamp_sec
        ):
            self.non_monotonic_stamp_count += 1
            self._mark_velocity_invalid(now_sec)
            return self.evaluate(now_sec)
        if (
            self.last_clock_sec is None
            or self.last_clock_receipt_sec is None
            or now_sec - self.last_clock_receipt_sec
            > self.config.velocity_freshness_sec
        ):
            self._mark_velocity_invalid(now_sec)
            return self.evaluate(now_sec)
        source_age_sec = self.last_clock_sec - source_stamp_sec
        if (
            source_age_sec > self.config.source_stamp_max_age_sec
            or source_age_sec < -self.config.source_stamp_future_tolerance_sec
        ):
            self._mark_velocity_invalid(now_sec)
            return self.evaluate(now_sec)

        self.last_source_stamp_sec = float(source_stamp_sec)
        self.last_valid_velocity_receipt_sec = float(now_sec)
        self.velocity_evidence_invalid_since_sec = None
        self.last_speed_mps = float(speed_mps)
        self.sample_count += 1
        self._update_progress_evidence(now_sec)
        return self.evaluate(now_sec)

    def observe_pose(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        source_stamp_sec: float,
        frame_id: str,
        now_sec: float,
    ) -> Optional[WatchdogVerdict]:
        if self._terminal is not None:
            return self._terminal
        normalized_frame_id = str(frame_id).strip()
        if (
            not all(
                math.isfinite(value)
                for value in (x_m, y_m, yaw_rad, source_stamp_sec)
            )
            or not normalized_frame_id
        ):
            self._mark_pose_invalid(now_sec)
            return self.evaluate(now_sec)
        if (
            self.last_pose_source_stamp_sec is not None
            and source_stamp_sec <= self.last_pose_source_stamp_sec
        ):
            self.non_monotonic_pose_stamp_count += 1
            self._mark_pose_invalid(now_sec)
            return self.evaluate(now_sec)
        if self.pose_frame_id is not None and normalized_frame_id != self.pose_frame_id:
            self._mark_pose_invalid(now_sec)
            return self.evaluate(now_sec)
        if (
            self.last_clock_sec is None
            or self.last_clock_receipt_sec is None
            or now_sec - self.last_clock_receipt_sec
            > self.config.pose_freshness_sec
        ):
            self._mark_pose_invalid(now_sec)
            return self.evaluate(now_sec)
        source_age_sec = self.last_clock_sec - source_stamp_sec
        if (
            source_age_sec > self.config.source_stamp_max_age_sec
            or source_age_sec < -self.config.source_stamp_future_tolerance_sec
        ):
            self._mark_pose_invalid(now_sec)
            return self.evaluate(now_sec)

        if self.last_pose_x_m is not None and self.last_pose_y_m is not None:
            step_m = math.hypot(x_m - self.last_pose_x_m, y_m - self.last_pose_y_m)
            if step_m > self.config.pose_max_step_m:
                self.pose_discontinuity_count += 1
                self._mark_pose_invalid(now_sec)
                return self.evaluate(now_sec)

        self.last_pose_source_stamp_sec = float(source_stamp_sec)
        self.last_valid_pose_receipt_sec = float(now_sec)
        self.last_pose_x_m = float(x_m)
        self.last_pose_y_m = float(y_m)
        self.last_pose_yaw_rad = float(yaw_rad)
        self.pose_frame_id = normalized_frame_id
        self.pose_evidence_invalid_since_sec = None
        self.pose_sample_count += 1

        if self.pose_anchor_x_m is None or self.pose_anchor_y_m is None:
            self._set_pose_anchor(x_m, y_m, yaw_rad)
        else:
            dx_m = x_m - self.pose_anchor_x_m
            dy_m = y_m - self.pose_anchor_y_m
            anchor_yaw_rad = self.pose_anchor_yaw_rad
            if anchor_yaw_rad is None:
                self._mark_pose_invalid(now_sec)
                return self.evaluate(now_sec)
            forward_progress_m = (
                dx_m * math.cos(anchor_yaw_rad) + dy_m * math.sin(anchor_yaw_rad)
            )
            if forward_progress_m >= self.config.pose_progress_min_m:
                self.last_pose_progress_sec = float(now_sec)
                self.pose_progress_count += 1
                self._set_pose_anchor(x_m, y_m, yaw_rad)

        self._update_progress_evidence(now_sec)
        return self.evaluate(now_sec)

    def evaluate(self, now_sec: float) -> Optional[WatchdogVerdict]:
        if self._terminal is not None:
            return self._terminal
        if self.authoritative_start_confirmed and not self.race_arm_seen:
            if self.authority_started_sec is None:
                self.authority_started_sec = float(now_sec)
            self._reset_stop_evidence()
            if (
                now_sec - self.authority_started_sec
                >= self.config.race_arm_confirmation_timeout_sec
            ):
                return self._latch(
                    "RACE_ARM_EVIDENCE_STALE_AFTER_AUTHORITY",
                    EXIT_RACE_ABORTED,
                    False,
                    now_sec,
                )
            return None
        if (
            self.pending_abort_reason is not None
            and self.pending_abort_since_sec is not None
            and now_sec - self.pending_abort_since_sec
            >= self.config.finish_ordering_grace_sec
        ):
            return self._latch(
                self.pending_abort_reason,
                EXIT_RACE_ABORTED,
                False,
                now_sec,
            )
        if not self._monitoring_active():
            self._reset_stop_evidence()
            return None

        if (
            self.last_valid_velocity_receipt_sec is None
            or now_sec - self.last_valid_velocity_receipt_sec
            > self.config.velocity_freshness_sec
            or self.last_clock_receipt_sec is None
            or now_sec - self.last_clock_receipt_sec
            > self.config.velocity_freshness_sec
        ):
            self._mark_velocity_invalid(now_sec)
        if (
            self.last_valid_pose_receipt_sec is None
            or now_sec - self.last_valid_pose_receipt_sec
            > self.config.pose_freshness_sec
            or self.last_clock_receipt_sec is None
            or now_sec - self.last_clock_receipt_sec
            > self.config.pose_freshness_sec
        ):
            self._mark_pose_invalid(now_sec)
        velocity_evidence_expired = (
            self.velocity_evidence_invalid_since_sec is not None
            and now_sec - self.velocity_evidence_invalid_since_sec
            >= self.config.evidence_failure_sec
        )
        pose_evidence_expired = (
            self.pose_evidence_invalid_since_sec is not None
            and now_sec - self.pose_evidence_invalid_since_sec
            >= self.config.evidence_failure_sec
        )
        progress_evidence_expired = (
            self.progress_evidence_invalid_since_sec is not None
            and now_sec - self.progress_evidence_invalid_since_sec
            >= self.config.evidence_failure_sec
        )
        if velocity_evidence_expired:
            return self._latch(
                "VELOCITY_EVIDENCE_STALE",
                EXIT_PROGRESS_EVIDENCE,
                False,
                now_sec,
            )
        if pose_evidence_expired:
            return self._latch(
                "POSE_EVIDENCE_STALE",
                EXIT_PROGRESS_EVIDENCE,
                False,
                now_sec,
            )
        if progress_evidence_expired:
            return self._latch(
                "PROGRESS_EVIDENCE_NOT_JOINTLY_FRESH",
                EXIT_PROGRESS_EVIDENCE,
                False,
                now_sec,
            )
        if self.velocity_evidence_invalid_since_sec is not None:
            return None
        if self.pose_evidence_invalid_since_sec is not None:
            return None

        self._update_progress_evidence(now_sec)
        if self.velocity_pose_inconsistent_since_sec is not None:
            self._reset_stationary()
            if (
                now_sec - self.velocity_pose_inconsistent_since_sec
                >= self.config.evidence_failure_sec
            ):
                return self._latch(
                    "VELOCITY_POSE_INCONSISTENT",
                    EXIT_PROGRESS_EVIDENCE,
                    False,
                    now_sec,
                )
            return None

        if (
            self.stationary_since_sec is not None
            and now_sec - self.stationary_since_sec >= self.config.stop_timeout_sec
        ):
            return self._latch(
                "D1_CONTINUOUS_STOP_15S",
                EXIT_CONTINUOUS_STOP,
                False,
                now_sec,
            )
        return None

    def _monitoring_active(self) -> bool:
        return (
            self.authoritative_start_confirmed
            and self.race_armed
            and self.running_seen
            and (
                self.vehicle_state in self._RUNNING_STATES
                or self.vehicle_state == "ready"
            )
        )

    def _mark_velocity_invalid(self, now_sec: float) -> None:
        if self.velocity_evidence_invalid_since_sec is None:
            self.velocity_evidence_invalid_since_sec = float(now_sec)
        self._mark_progress_evidence_invalid(now_sec)

    def _mark_pose_invalid(self, now_sec: float) -> None:
        if self.pose_evidence_invalid_since_sec is None:
            self.pose_evidence_invalid_since_sec = float(now_sec)
        self._mark_progress_evidence_invalid(now_sec)

    def _mark_progress_evidence_invalid(self, now_sec: float) -> None:
        if self.progress_evidence_invalid_since_sec is None:
            self.progress_evidence_invalid_since_sec = float(now_sec)

    def _set_pose_anchor(self, x_m: float, y_m: float, yaw_rad: float) -> None:
        self.pose_anchor_x_m = float(x_m)
        self.pose_anchor_y_m = float(y_m)
        self.pose_anchor_yaw_rad = float(yaw_rad)

    def _pose_supports_motion(self, now_sec: float) -> bool:
        return (
            self.last_pose_progress_sec is not None
            and now_sec - self.last_pose_progress_sec
            <= self.config.pose_freshness_sec
        )

    def _progress_evidence_fresh(self, now_sec: float) -> bool:
        return (
            self.velocity_evidence_invalid_since_sec is None
            and self.pose_evidence_invalid_since_sec is None
            and self.last_valid_velocity_receipt_sec is not None
            and now_sec - self.last_valid_velocity_receipt_sec
            <= self.config.velocity_freshness_sec
            and self.last_valid_pose_receipt_sec is not None
            and now_sec - self.last_valid_pose_receipt_sec
            <= self.config.pose_freshness_sec
        )

    def _update_progress_evidence(self, now_sec: float) -> None:
        if not self._monitoring_active():
            self._reset_stationary()
            self.velocity_pose_inconsistent_since_sec = None
            return
        if not self._progress_evidence_fresh(now_sec):
            # An inadmissible sample cannot prove motion and must not erase
            # already accumulated stop evidence.  Preserve both timers until
            # fresh evidence either proves motion or the evidence-failure
            # deadline latches fail-closed.
            return
        if (
            self.progress_evidence_invalid_since_sec is not None
            and now_sec - self.progress_evidence_invalid_since_sec
            < self.config.evidence_failure_sec
        ):
            self.progress_evidence_invalid_since_sec = None
        if self.last_speed_mps is None:
            self._reset_stationary()
            return

        absolute_speed_mps = abs(self.last_speed_mps)
        pose_supports_motion = self._pose_supports_motion(now_sec)
        velocity_supports_motion = absolute_speed_mps >= self.config.stop_exit_speed_mps
        velocity_supports_stop = absolute_speed_mps <= self.config.stop_enter_speed_mps

        if velocity_supports_motion != pose_supports_motion:
            if self.velocity_pose_inconsistent_since_sec is None:
                self.velocity_pose_inconsistent_since_sec = float(now_sec)
            self.velocity_pose_inconsistent_count += 1
            self._reset_stationary()
            return
        self.velocity_pose_inconsistent_since_sec = None

        if velocity_supports_motion:
            self._reset_stationary()
            return
        if velocity_supports_stop:
            if self.stationary_since_sec is None:
                self.stationary_since_sec = float(now_sec)
                self.stationary_sample_count = 0
            self.stationary_sample_count += 1

    def _reset_stationary(self) -> None:
        self.stationary_since_sec = None
        self.stationary_sample_count = 0

    def _reset_stop_evidence(self) -> None:
        self._reset_stationary()
        self.velocity_evidence_invalid_since_sec = None
        self.pose_evidence_invalid_since_sec = None
        self.progress_evidence_invalid_since_sec = None
        self.velocity_pose_inconsistent_since_sec = None

    def _begin_pending_abort(self, reason: str, now_sec: float) -> None:
        if self.pending_abort_reason is None:
            self.pending_abort_reason = reason
            self.pending_abort_since_sec = float(now_sec)

    def _clear_pending_abort(self) -> None:
        self.pending_abort_reason = None
        self.pending_abort_since_sec = None

    def snapshot(self, verdict: WatchdogVerdict) -> dict[str, object]:
        stationary_duration_sec = 0.0
        if self.stationary_since_sec is not None:
            stationary_duration_sec = max(
                0.0, verdict.detected_monotonic_sec - self.stationary_since_sec
            )
        return {
            "schema": "D1_START_PROGRESS_WATCHDOG_V2",
            "reason": verdict.reason,
            "exit_code": verdict.exit_code,
            "passed": verdict.passed,
            "detected_utc": datetime.now(timezone.utc).isoformat(),
            "detected_monotonic_sec": verdict.detected_monotonic_sec,
            "vehicle_state": self.vehicle_state,
            "authoritative_start_confirmed": self.authoritative_start_confirmed,
            "authority_started_sec": self.authority_started_sec,
            "race_armed": self.race_armed,
            "race_arm_seen": self.race_arm_seen,
            "running_seen": self.running_seen,
            "motion_state_seen": self.motion_state_seen,
            "last_speed_mps": self.last_speed_mps,
            "last_source_stamp_sec": self.last_source_stamp_sec,
            "last_valid_velocity_receipt_sec": self.last_valid_velocity_receipt_sec,
            "last_pose_source_stamp_sec": self.last_pose_source_stamp_sec,
            "last_valid_pose_receipt_sec": self.last_valid_pose_receipt_sec,
            "last_pose_x_m": self.last_pose_x_m,
            "last_pose_y_m": self.last_pose_y_m,
            "last_pose_yaw_rad": self.last_pose_yaw_rad,
            "pose_frame_id": self.pose_frame_id,
            "last_pose_progress_sec": self.last_pose_progress_sec,
            "last_clock_sec": self.last_clock_sec,
            "last_clock_receipt_sec": self.last_clock_receipt_sec,
            "stationary_duration_sec": stationary_duration_sec,
            "sample_count": self.sample_count,
            "pose_sample_count": self.pose_sample_count,
            "pose_progress_count": self.pose_progress_count,
            "stationary_sample_count": self.stationary_sample_count,
            "non_monotonic_stamp_count": self.non_monotonic_stamp_count,
            "non_monotonic_pose_stamp_count": self.non_monotonic_pose_stamp_count,
            "pose_discontinuity_count": self.pose_discontinuity_count,
            "velocity_pose_inconsistent_count": self.velocity_pose_inconsistent_count,
            "velocity_evidence_invalid_since_sec": self.velocity_evidence_invalid_since_sec,
            "pose_evidence_invalid_since_sec": self.pose_evidence_invalid_since_sec,
            "progress_evidence_invalid_since_sec": (
                self.progress_evidence_invalid_since_sec
            ),
            "velocity_pose_inconsistent_since_sec": self.velocity_pose_inconsistent_since_sec,
            "pending_abort_reason": self.pending_abort_reason,
            "pending_abort_since_sec": self.pending_abort_since_sec,
            "config": asdict(self.config),
        }


def _sha256_file(path: Path) -> Optional[str]:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_verdict(
    output_path: Path,
    run_id: str,
    watchdog: D1ProgressWatchdog,
    verdict: WatchdogVerdict,
    fingerprint_path: Optional[Path],
) -> None:
    payload = watchdog.snapshot(verdict)
    payload["run_id"] = run_id
    payload["velocity_topic"] = "/vehicle/status/velocity_status"
    payload["pose_topic"] = "/localization/kinematic_state"
    payload["vehicle_state_topic"] = "/awsim/state"
    payload["race_arm_topic"] = "/overtake/race_armed"
    payload["artifact_fingerprint_file"] = (
        str(fingerprint_path) if fingerprint_path is not None else None
    )
    payload["artifact_fingerprint_file_sha256"] = (
        _sha256_file(fingerprint_path) if fingerprint_path is not None else None
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary_path, output_path)


def _run_ros(args: argparse.Namespace, watchdog: D1ProgressWatchdog) -> WatchdogVerdict:
    import rclpy
    from autoware_auto_vehicle_msgs.msg import VelocityReport
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool, String

    class WatchdogNode(Node):
        def __init__(self) -> None:
            super().__init__("d1_start_progress_watchdog")
            latched_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                String,
                "/awsim/state",
                lambda msg: watchdog.observe_vehicle_state(
                    msg.data, time.monotonic()
                ),
                latched_qos,
            )
            self.create_subscription(
                Bool,
                "/overtake/race_armed",
                lambda msg: watchdog.observe_race_arm(
                    bool(msg.data), time.monotonic()
                ),
                latched_qos,
            )
            self.create_subscription(
                VelocityReport,
                "/vehicle/status/velocity_status",
                self._on_velocity,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Odometry,
                "/localization/kinematic_state",
                self._on_pose,
                qos_profile_sensor_data,
            )
            clock_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                Clock,
                "/clock",
                self._on_clock,
                clock_qos,
            )
            self.create_timer(0.1, lambda: watchdog.evaluate(time.monotonic()))

        @staticmethod
        def _on_velocity(msg: VelocityReport) -> None:
            stamp_sec = float(msg.header.stamp.sec) + float(
                msg.header.stamp.nanosec
            ) * 1.0e-9
            watchdog.observe_velocity(
                float(msg.longitudinal_velocity),
                stamp_sec,
                time.monotonic(),
            )

        @staticmethod
        def _on_pose(msg: Odometry) -> None:
            stamp_sec = float(msg.header.stamp.sec) + float(
                msg.header.stamp.nanosec
            ) * 1.0e-9
            orientation = msg.pose.pose.orientation
            quaternion_norm = math.sqrt(
                float(orientation.x) ** 2
                + float(orientation.y) ** 2
                + float(orientation.z) ** 2
                + float(orientation.w) ** 2
            )
            yaw_rad = float("nan")
            if math.isfinite(quaternion_norm) and quaternion_norm > 1.0e-6:
                x = float(orientation.x) / quaternion_norm
                y = float(orientation.y) / quaternion_norm
                z = float(orientation.z) / quaternion_norm
                w = float(orientation.w) / quaternion_norm
                yaw_rad = math.atan2(
                    2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z),
                )
            watchdog.observe_pose(
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                yaw_rad,
                stamp_sec,
                msg.header.frame_id,
                time.monotonic(),
            )

        @staticmethod
        def _on_clock(msg: Clock) -> None:
            clock_sec = float(msg.clock.sec) + float(msg.clock.nanosec) * 1.0e-9
            watchdog.observe_clock(clock_sec, time.monotonic())

    rclpy.init()
    node = WatchdogNode()
    try:
        while rclpy.ok() and watchdog.terminal is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    except Exception:
        # rclpy may invalidate the wait set before spin_once returns when the
        # container forwards SIGINT/SIGTERM. Preserve genuine runtime errors.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if watchdog.terminal is not None:
        return watchdog.terminal
    return WatchdogVerdict("ABORTED_BY_EXTERNAL_SHUTDOWN", 130, False, time.monotonic())


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Fail a Gate/dev run when fresh velocity and pose evidence both prove "
            "that D1 is continuously stopped after Start."
        )
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument(
        "--authoritative-start-confirmed",
        required=True,
        action="store_true",
        help="Proof that the supervisor's authoritative Start helper exited 0.",
    )
    parser.add_argument("--stop-timeout-sec", type=float, default=15.0)
    parser.add_argument("--stop-enter-speed-mps", type=float, default=0.05)
    parser.add_argument("--stop-exit-speed-mps", type=float, default=0.10)
    parser.add_argument("--velocity-freshness-sec", type=float, default=1.0)
    parser.add_argument("--pose-freshness-sec", type=float, default=1.0)
    parser.add_argument("--evidence-failure-sec", type=float, default=15.0)
    parser.add_argument("--source-stamp-max-age-sec", type=float, default=1.0)
    parser.add_argument(
        "--source-stamp-future-tolerance-sec", type=float, default=0.10
    )
    parser.add_argument("--pose-progress-min-m", type=float, default=0.10)
    parser.add_argument("--pose-max-step-m", type=float, default=2.0)
    parser.add_argument("--finish-ordering-grace-sec", type=float, default=1.0)
    parser.add_argument(
        "--race-arm-confirmation-timeout-sec", type=float, default=5.0
    )
    parser.add_argument("--fingerprint", type=Path)
    args = parser.parse_args()

    config = WatchdogConfig(
        stop_timeout_sec=args.stop_timeout_sec,
        stop_enter_speed_mps=args.stop_enter_speed_mps,
        stop_exit_speed_mps=args.stop_exit_speed_mps,
        velocity_freshness_sec=args.velocity_freshness_sec,
        pose_freshness_sec=args.pose_freshness_sec,
        evidence_failure_sec=args.evidence_failure_sec,
        source_stamp_max_age_sec=args.source_stamp_max_age_sec,
        source_stamp_future_tolerance_sec=args.source_stamp_future_tolerance_sec,
        pose_progress_min_m=args.pose_progress_min_m,
        pose_max_step_m=args.pose_max_step_m,
        finish_ordering_grace_sec=args.finish_ordering_grace_sec,
        race_arm_confirmation_timeout_sec=args.race_arm_confirmation_timeout_sec,
    )
    watchdog = D1ProgressWatchdog(
        config,
        authoritative_start_confirmed=args.authoritative_start_confirmed,
        authority_started_sec=time.monotonic(),
    )
    verdict = _run_ros(args, watchdog)
    _write_verdict(
        args.output,
        args.run_id,
        watchdog,
        verdict,
        args.fingerprint,
    )
    print(
        "[d1-progress-watchdog] "
        f"reason={verdict.reason} exit_code={verdict.exit_code} "
        f"output={args.output}",
        flush=True,
    )
    return verdict.exit_code


if __name__ == "__main__":
    raise SystemExit(main())

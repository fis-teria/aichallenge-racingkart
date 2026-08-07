#!/usr/bin/env python3
"""Run the Start Helper as a signal-forwarding Linux child subreaper."""

from __future__ import annotations

import argparse
import ctypes
import errno
import os
import signal
import subprocess
import sys
import time
from collections.abc import Sequence


PR_SET_CHILD_SUBREAPER = 36
PR_SET_PDEATHSIG = 1


def _prctl(option: int, value: int) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(option, value, 0, 0, 0) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))


def _child_setup(parent_pid: int) -> None:
    os.setsid()
    _prctl(PR_SET_PDEATHSIG, signal.SIGKILL)
    if os.getppid() != parent_pid:
        os.kill(os.getpid(), signal.SIGKILL)


def _signal_group(pgid: int, signum: int) -> None:
    try:
        os.killpg(pgid, signum)
    except ProcessLookupError:
        pass


def _group_exists(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _reap_nonblocking() -> None:
    while True:
        try:
            pid, _status = os.waitpid(-1, os.WNOHANG)
        except ChildProcessError:
            return
        except InterruptedError:
            continue
        if pid == 0:
            return


def _direct_child_groups() -> set[int]:
    groups: set[int] = set()
    own_pid = os.getpid()
    own_group = os.getpgrp()
    for status_path in os.scandir("/proc"):
        if not status_path.name.isdigit():
            continue
        try:
            status = open(
                os.path.join(status_path.path, "status"), encoding="utf-8"
            ).read()
            parent_line = next(
                line for line in status.splitlines() if line.startswith("PPid:")
            )
            if int(parent_line.split()[1]) != own_pid:
                continue
            pgid = os.getpgid(int(status_path.name))
        except (FileNotFoundError, ProcessLookupError, PermissionError, StopIteration):
            continue
        if pgid != own_group:
            groups.add(pgid)
    return groups


def _deadline_seconds(deadline_monotonic_ns: int) -> float:
    if deadline_monotonic_ns <= 0:
        return float("inf")
    return deadline_monotonic_ns / 1_000_000_000


def _drain_descendants(
    initial_pgid: int, grace_sec: float, deadline_monotonic_ns: int
) -> None:
    absolute_deadline = _deadline_seconds(deadline_monotonic_ns)
    groups = {initial_pgid} | _direct_child_groups()
    for pgid in groups:
        _signal_group(pgid, signal.SIGTERM)
    deadline = min(time.monotonic() + grace_sec, absolute_deadline)
    while time.monotonic() < deadline:
        _reap_nonblocking()
        current_groups = _direct_child_groups()
        for pgid in current_groups - groups:
            _signal_group(pgid, signal.SIGTERM)
        groups.update(current_groups)
        if not current_groups and not any(_group_exists(pgid) for pgid in groups):
            break
        time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
    groups.update(_direct_child_groups())
    for pgid in groups:
        if _group_exists(pgid):
            _signal_group(pgid, signal.SIGKILL)
    final_deadline = min(time.monotonic() + 1.0, absolute_deadline)
    while time.monotonic() < final_deadline:
        _reap_nonblocking()
        current_groups = _direct_child_groups()
        for pgid in current_groups:
            _signal_group(pgid, signal.SIGKILL)
        if not current_groups:
            break
        time.sleep(min(0.02, max(0.0, final_deadline - time.monotonic())))
    _reap_nonblocking()


def run(command: Sequence[str], grace_sec: float, deadline_monotonic_ns: int) -> int:
    if not sys.platform.startswith("linux"):
        raise RuntimeError("helper process reaper requires Linux")
    _prctl(PR_SET_CHILD_SUBREAPER, 1)
    parent_pid = os.getpid()
    child = subprocess.Popen(
        list(command),
        preexec_fn=lambda: _child_setup(parent_pid),
    )

    def forward(signum: int, _frame: object) -> None:
        _signal_group(child.pid, signum)

    previous_handlers = {
        signum: signal.signal(signum, forward)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    returncode: int | None = None
    deadline_shutdown_started = False
    try:
        absolute_deadline = _deadline_seconds(deadline_monotonic_ns)
        shutdown_start = absolute_deadline
        if deadline_monotonic_ns > 0:
            shutdown_start -= grace_sec + 1.0
        while returncode is None and time.monotonic() < shutdown_start:
            returncode = child.poll()
            if returncode is None:
                time.sleep(min(0.02, max(0.0, shutdown_start - time.monotonic())))
        if returncode is None:
            deadline_shutdown_started = True
    finally:
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)
        _drain_descendants(child.pid, grace_sec, deadline_monotonic_ns)
    if deadline_shutdown_started:
        return 124
    if returncode is None:
        returncode = child.poll()
    if returncode is None:
        returncode = -signal.SIGKILL
    return returncode if returncode >= 0 else 128 + -returncode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--grace-sec", type=float, required=True)
    parser.add_argument("--deadline-monotonic-ns", type=int, default=0)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command[:1] == ["--"]:
        command = command[1:]
    if (
        not command
        or not (0.0 < args.grace_sec <= 30.0)
        or args.deadline_monotonic_ns < 0
    ):
        parser.error("a command, grace in (0, 30], and non-negative deadline are required")
    try:
        return run(command, args.grace_sec, args.deadline_monotonic_ns)
    except OSError as error:
        if error.errno == errno.EPERM:
            print("[awsim-start][ERROR] child subreaper setup not permitted", file=sys.stderr)
            return 2
        raise


if __name__ == "__main__":
    raise SystemExit(main())

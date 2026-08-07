#!/usr/bin/env python3
"""Negative subprocess coverage for the C-002AY1 Unix/memfd attach path."""

from array import array
import fcntl
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time


RING_MAGIC = 0x4330303241593152
ABI_VERSION = 1
LAYOUT_VERSION = 1
HEADER_SIZE = 320
RECORD_SIZE = 56
CAPACITY = 4096
ROLE_PLANNER = 1
MAPPING_SIZE = HEADER_SIZE + RECORD_SIZE * CAPACITY
HARNESS_READY = 1
RUN_ID = "negative-attach"
NONCE = 71
INSTANCE = 72


def run_hash(value: str) -> int:
    result = 1469598103934665603
    for byte in value.encode():
        result ^= byte
        result = (result * 1099511628211) & ((1 << 64) - 1)
    return result


def make_memfd(*, size: int = MAPPING_SIZE, nonce: int = NONCE,
               sealed: bool = True) -> int:
    fd = os.memfd_create("c002ay1-negative", os.MFD_CLOEXEC |
                         os.MFD_ALLOW_SEALING)
    os.ftruncate(fd, size)
    if size == MAPPING_SIZE:
        header = bytearray(HEADER_SIZE)
        struct.pack_into("<QIIIII", header, 0, RING_MAGIC, ABI_VERSION,
                         LAYOUT_VERSION, HEADER_SIZE, RECORD_SIZE, CAPACITY)
        struct.pack_into("<B", header, 28, ROLE_PLANNER)
        struct.pack_into("<QQQQQ", header, 32, MAPPING_SIZE, nonce, INSTANCE,
                         run_hash(RUN_ID), HARNESS_READY)
        os.pwrite(fd, header, 0)
    if sealed:
        fcntl.fcntl(
            fd,
            fcntl.F_ADD_SEALS,
            fcntl.F_SEAL_GROW | fcntl.F_SEAL_SHRINK | fcntl.F_SEAL_SEAL,
        )
    return fd


def run_case(probe: Path, root: Path, name: str, mode: str,
             expected: str = "disabled") -> None:
    socket_path = root / f"{name}.sock"
    server = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    server.bind(str(socket_path))
    server.listen(1)
    server_error: list[BaseException] = []

    def serve() -> None:
        descriptors: list[int] = []
        try:
            peer, _ = server.accept()
            with peer:
                handshake = peer.recv(256)
                assert len(handshake) >= 32
                if mode == "no_fd":
                    peer.send(b"\x01")
                elif mode == "timeout":
                    time.sleep(0.2)
                else:
                    if mode == "regular":
                        regular = root / "regular.bin"
                        regular.write_bytes(b"\0" * MAPPING_SIZE)
                        descriptors = [os.open(regular, os.O_RDWR)]
                    elif mode == "unsealed":
                        descriptors = [make_memfd(sealed=False)]
                    elif mode == "wrong_size":
                        descriptors = [make_memfd(size=MAPPING_SIZE - 1)]
                    elif mode == "old_session":
                        descriptors = [make_memfd(nonce=NONCE + 1)]
                    elif mode == "multiple_fds":
                        descriptors = [make_memfd(), make_memfd()]
                    elif mode == "valid_close":
                        descriptors = [make_memfd()]
                    else:
                        raise AssertionError(f"unknown mode {mode}")
                    peer.sendmsg(
                        [b"\x01"],
                        [(socket.SOL_SOCKET, socket.SCM_RIGHTS,
                          array("i", descriptors))],
                    )
        except BaseException as error:
            server_error.append(error)
        finally:
            for descriptor in descriptors:
                os.close(descriptor)
            server.close()

    thread = threading.Thread(target=serve)
    thread.start()
    started = time.monotonic()
    result = subprocess.run(
        [str(probe), str(socket_path), RUN_ID, str(NONCE), str(INSTANCE),
         expected],
        check=False,
        capture_output=True,
        text=True,
        timeout=1,
    )
    elapsed = time.monotonic() - started
    thread.join(timeout=1)
    assert not thread.is_alive(), f"{name}: server did not terminate"
    assert not server_error, f"{name}: {server_error}"
    assert result.returncode == 0, (
        f"{name}: exit={result.returncode} stdout={result.stdout} "
        f"stderr={result.stderr}"
    )
    assert elapsed < 0.5, f"{name}: attach was not bounded ({elapsed:.3f}s)"


def main() -> int:
    probe = Path(sys.argv[1])
    with tempfile.TemporaryDirectory(prefix="c002ay1-negative-") as temp:
        root = Path(temp)
        absent_path = root / "absent.sock"
        started = time.monotonic()
        absent = subprocess.run(
            [str(probe), str(absent_path), RUN_ID, str(NONCE), str(INSTANCE),
             "disabled"],
            check=False,
            timeout=1,
        )
        assert absent.returncode == 0
        assert time.monotonic() - started < 0.5
        for mode in (
            "regular",
            "unsealed",
            "wrong_size",
            "old_session",
            "no_fd",
            "timeout",
            "multiple_fds",
        ):
            run_case(probe, root, mode, mode)
        run_case(probe, root, "valid_close", "valid_close", "enabled")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

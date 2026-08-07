#!/usr/bin/env python3
"""Wait for a ROS 2 SetBool service without relying on ros2cli discovery."""

from __future__ import annotations

import argparse
import math
import os
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


def positive_timeout(value: str) -> float:
    timeout_sec = float(value)
    if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
        raise argparse.ArgumentTypeError("timeout must be a positive finite value")
    return timeout_sec


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--service", required=True)
    parser.add_argument("--timeout-sec", required=True, type=positive_timeout)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    node: Node | None = None
    initialized = False
    try:
        rclpy.init(args=None)
        initialized = True
        node = Node(f"awsim_official_start_waiter_{os.getpid()}")
        client = node.create_client(SetBool, args.service)
        if not client.wait_for_service(timeout_sec=args.timeout_sec):
            print(
                f"typed service unavailable: {args.service} "
                "expected=std_srvs/srv/SetBool",
                file=sys.stderr,
            )
            return 1
        return 0
    except Exception as error:  # noqa: BLE001 - CLI must fail closed on ROS errors.
        print(f"typed service discovery failed: {error}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

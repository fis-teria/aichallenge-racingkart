#!/bin/bash
# Common container initialization: network tuning + ROS workspace setup.
# Used as ENTRYPOINT in Dockerfile and sourced from .bashrc for rocker sessions.

# --- DDS network tuning (multicast on loopback + large receive buffer) ---
ip link set multicast on lo || true
sysctl -w net.core.rmem_max=2147483647 >/dev/null || true

# --- Source ROS / Autoware base first so commands such as `ros2` are available
# even before the mounted challenge workspace has been built.
if [ -f /autoware/install/setup.bash ]; then
    # shellcheck disable=SC1091
    set +u && source /autoware/install/setup.bash
elif [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    # shellcheck disable=SC1090
    set +u && source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# --- Source challenge workspace overlay (skip when not yet built, e.g. first dev session) ---
if [ -f /aichallenge/workspace/install/setup.bash ]; then
    # shellcheck disable=SC1091
    set +u && source /aichallenge/workspace/install/setup.bash
fi

# When used as ENTRYPOINT, hand off to the CMD / command.
# When sourced from .bashrc, exec is a no-op (no positional args).
if [ $# -gt 0 ]; then
    exec "$@"
fi

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
APP="${SCRIPT_DIR}/app.py"
RUNTIME_DIR="${SCRIPT_DIR}/runtime"

HOST="${TUNING_GUI_HOST:-127.0.0.1}"
PORT="${TUNING_GUI_PORT:-8765}"
MODE="foreground"
OPEN_BROWSER="false"

usage() {
    cat <<'EOF'
Usage:
  tools/tuning_gui/run_tuning_gui.bash [options]

Options:
  -d, --background       Start in background and write a pid/log file.
      --foreground       Start in foreground. This is the default.
      --host HOST        Bind host. Default: 127.0.0.1 or TUNING_GUI_HOST.
      --port PORT        Bind port. Default: 8765 or TUNING_GUI_PORT.
      --open             Open the GUI URL in the default browser when possible.
      --status           Show managed process and port status.
      --stop             Stop the managed background server for this port.
      --restart          Stop, then start in background.
  -h, --help             Show this help.

Examples:
  tools/tuning_gui/run_tuning_gui.bash
  tools/tuning_gui/run_tuning_gui.bash --background
  tools/tuning_gui/run_tuning_gui.bash --status
  tools/tuning_gui/run_tuning_gui.bash --stop
EOF
}

die() {
    echo "error: $*" >&2
    exit 1
}

pid_file() {
    echo "${RUNTIME_DIR}/server-${PORT}.pid"
}

log_file() {
    echo "${RUNTIME_DIR}/server-${PORT}.log"
}

managed_pid() {
    local file
    file="$(pid_file)"
    [[ -f "${file}" ]] || return 1
    local pid
    pid="$(<"${file}")"
    [[ -n "${pid}" ]] || return 1
    if kill -0 "${pid}" 2>/dev/null; then
        echo "${pid}"
        return 0
    fi
    rm -f "${file}"
    return 1
}

port_in_use() {
    if command -v ss >/dev/null 2>&1; then
        ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${PORT}$"
    elif command -v lsof >/dev/null 2>&1; then
        lsof -nP -iTCP:"${PORT}" -sTCP:LISTEN >/dev/null 2>&1
    else
        python3 - "${HOST}" "${PORT}" <<'PY'
import socket
import sys

host, port = sys.argv[1], int(sys.argv[2])
sock = socket.socket()
try:
    sock.bind((host, port))
except OSError:
    sys.exit(0)
finally:
    sock.close()
sys.exit(1)
PY
    fi
}

open_browser() {
    [[ "${OPEN_BROWSER}" == "true" ]] || return 0
    local url="http://${HOST}:${PORT}"
    if command -v xdg-open >/dev/null 2>&1; then
        xdg-open "${url}" >/dev/null 2>&1 || true
    elif command -v sensible-browser >/dev/null 2>&1; then
        sensible-browser "${url}" >/dev/null 2>&1 || true
    fi
}

status_server() {
    local url="http://${HOST}:${PORT}"
    if pid="$(managed_pid)"; then
        echo "managed: running pid=${pid}"
        echo "url: ${url}"
        echo "log: $(log_file)"
        return 0
    fi
    if port_in_use; then
        echo "managed: not tracked"
        echo "port: ${PORT} is already listening"
        echo "url: ${url}"
        return 0
    fi
    echo "managed: stopped"
    echo "url: ${url}"
}

stop_server() {
    local pid
    if ! pid="$(managed_pid)"; then
        if port_in_use; then
            echo "port ${PORT} is listening, but it is not tracked by $(pid_file)"
            echo "leaving it untouched"
            return 0
        fi
        echo "tuning GUI is not running"
        return 0
    fi

    echo "stopping tuning GUI pid=${pid}"
    kill "${pid}" 2>/dev/null || true
    for _ in $(seq 1 40); do
        if ! kill -0 "${pid}" 2>/dev/null; then
            rm -f "$(pid_file)"
            echo "stopped"
            return 0
        fi
        sleep 0.1
    done
    echo "pid=${pid} did not stop after SIGTERM; sending SIGKILL"
    kill -9 "${pid}" 2>/dev/null || true
    rm -f "$(pid_file)"
}

start_foreground() {
    if port_in_use; then
        echo "tuning GUI seems to be already running: http://${HOST}:${PORT}"
        echo "use --status for details"
        return 0
    fi
    cd "${REPO_ROOT}"
    exec env TUNING_GUI_HOST="${HOST}" TUNING_GUI_PORT="${PORT}" python3 "${APP}"
}

start_background() {
    mkdir -p "${RUNTIME_DIR}"
    if port_in_use; then
        echo "tuning GUI seems to be already running: http://${HOST}:${PORT}"
        echo "use --status for details"
        return 0
    fi

    local log pid
    log="$(log_file)"
    cd "${REPO_ROOT}"
    if command -v setsid >/dev/null 2>&1; then
        setsid env PYTHONUNBUFFERED=1 TUNING_GUI_HOST="${HOST}" TUNING_GUI_PORT="${PORT}" \
            python3 "${APP}" >"${log}" 2>&1 < /dev/null &
    else
        env PYTHONUNBUFFERED=1 TUNING_GUI_HOST="${HOST}" TUNING_GUI_PORT="${PORT}" \
            python3 "${APP}" >"${log}" 2>&1 < /dev/null &
    fi
    pid="$!"
    echo "${pid}" >"$(pid_file)"
    sleep 0.5
    if ! kill -0 "${pid}" 2>/dev/null; then
        rm -f "$(pid_file)"
        echo "tuning GUI failed to start. log:"
        sed -n '1,120p' "${log}" >&2 || true
        exit 1
    fi
    echo "tuning GUI started"
    echo "pid: ${pid}"
    echo "url: http://${HOST}:${PORT}"
    echo "log: ${log}"
    open_browser
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -d|--background)
            MODE="background"
            shift
            ;;
        --foreground)
            MODE="foreground"
            shift
            ;;
        --host)
            [[ $# -ge 2 ]] || die "--host requires a value"
            HOST="$2"
            shift 2
            ;;
        --port)
            [[ $# -ge 2 ]] || die "--port requires a value"
            PORT="$2"
            shift 2
            ;;
        --open)
            OPEN_BROWSER="true"
            shift
            ;;
        --status)
            MODE="status"
            shift
            ;;
        --stop)
            MODE="stop"
            shift
            ;;
        --restart)
            MODE="restart"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            die "unknown option: $1"
            ;;
    esac
done

[[ "${PORT}" =~ ^[0-9]+$ ]] || die "port must be a number: ${PORT}"
[[ -f "${APP}" ]] || die "app.py not found: ${APP}"
command -v python3 >/dev/null 2>&1 || die "python3 is required"

case "${MODE}" in
    foreground)
        start_foreground
        ;;
    background)
        start_background
        ;;
    status)
        status_server
        ;;
    stop)
        stop_server
        ;;
    restart)
        stop_server
        start_background
        ;;
    *)
        die "internal error: unknown mode ${MODE}"
        ;;
esac

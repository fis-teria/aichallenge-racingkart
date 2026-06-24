#!/usr/bin/env bash
set -euo pipefail

TOOLS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GUI_SCRIPT="${TOOLS_DIR}/tuning_gui/run_tuning_gui.bash"

if [[ ! -x "${GUI_SCRIPT}" ]]; then
    echo "error: tuning GUI launcher not found or not executable: ${GUI_SCRIPT}" >&2
    exit 1
fi

exec "${GUI_SCRIPT}" "$@"

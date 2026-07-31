#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 RUN_DIR" >&2
  exit 2
fi
ga_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHONPATH="$ga_root/src${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m ga_pure_pursuit.cli resume --run-dir "$1"

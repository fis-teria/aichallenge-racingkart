#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 RUN_DIR OUTPUT_YAML" >&2
  exit 2
fi
ga_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHONPATH="$ga_root/src${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m ga_pure_pursuit.cli export --run-dir "$1" --output "$2"

#!/usr/bin/env bash
set -euo pipefail

ga_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
config_path="${1:-$ga_root/config/experiment.yaml}"
PYTHONPATH="$ga_root/src${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m ga_pure_pursuit.cli search \
  --config "$config_path" --runs-dir "$ga_root/runs"

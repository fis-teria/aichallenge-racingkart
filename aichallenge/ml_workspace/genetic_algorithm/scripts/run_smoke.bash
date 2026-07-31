#!/usr/bin/env bash
set -euo pipefail

ga_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
tmp_config="$(mktemp)"
trap 'rm -f "$tmp_config"' EXIT
python3 - "$ga_root/config/experiment.yaml" "$tmp_config" <<'PY'
import json
import sys

config = json.load(open(sys.argv[1], encoding="utf-8"))
config["run"]["population_size"] = 8
config["run"]["generations"] = 3
json.dump(config, open(sys.argv[2], "w", encoding="utf-8"), indent=2)
PY
PYTHONPATH="$ga_root/src${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m ga_pure_pursuit.cli search \
  --config "$tmp_config" --runs-dir "$ga_root/runs"

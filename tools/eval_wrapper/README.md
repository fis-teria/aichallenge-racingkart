# evalwrap

Local-only evaluation wrapper for Automotive AI Challenge racing kart runs.

The wrapper keeps evaluation artifacts outside the submission workspace:

```text
tools/eval_wrapper/      # this tool
analysis/runs/<run_id>/  # generated local results
```

It does not write to `aichallenge/workspace/src/aichallenge_submit`.

## Quick Start

From `aichallenge-racingkart/tools/eval_wrapper`:

```bash
python -m evalwrap doctor
python -m evalwrap ingest --label baseline --path ../../output/latest
```

From the repository root:

```bash
tools/evalwrap run --label baseline
tools/evalwrap list
tools/evalwrap leaderboard --metric total_time_sec
```

`run` executes:

```text
./create_submit_file.bash
./docker_build.sh eval
make eval
```

Use `ingest` when evaluation output already exists and you only want to collect and report it.

## Outputs

```text
analysis/runs/<run_id>/manifest.yaml
analysis/runs/<run_id>/raw/d1/
analysis/runs/<run_id>/processed/metrics.json
analysis/runs/<run_id>/processed/corner_summary.csv
analysis/runs/<run_id>/processed/trajectory_reference.csv
analysis/runs/<run_id>/report/index.html
analysis/experiments.sqlite
```

Missing official JSON files produce a `partial` run instead of crashing.

## Reference Trajectory Fallback

When the rosbag does not contain `/planning/scenario_planning/trajectory`,
evalwrap can use the MPC reference CSV from
`multi_purpose_mpc_ros/config/config.yaml` as the fallback trajectory.

This enables `corner_summary.csv`, path error metrics, and the Corner Splits map
in the HTML report even when the planning trajectory was not recorded.

Corner numbering can be rotated with `corner_id_rotation` in
`configs/thresholds.yaml`. The default AI Challenge setting starts numbering
from the second detected corner so the first detected corner becomes
`corner_08`.

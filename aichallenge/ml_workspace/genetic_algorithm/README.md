# Pure Pursuit GA exploration

This workspace searches the tunable parameters of
`aichallenge_submit/ga_simple_pure_pursuit` while keeping vehicle geometry,
hard steering limits, stale guards, and transport contracts fixed.
The GA target speed is fixed at 35 km/h (`9.722222222222 m/s`); speed is not
part of the genome. Corner speed is limited by
`sqrt(max_lateral_acceleration / curvature)` using the maximum preview
curvature. The GA tunes lateral acceleration, minimum corner speed, and preview
distance, so 35 km/h remains the straight-line ceiling rather than a mandatory
speed through every corner.

## What is included

- bounded real-valued GA with baseline seeding, elitism, tournament selection,
  blend crossover, Gaussian mutation, repair, and duplicate-result caching
- safety-first fitness and repeated-evaluation aggregation
- SQLite results, JSONL history, generation checkpoints, and reproducible hashes
- four evaluators:
  - `mock`: validates the complete search pipeline without ROS/AWSIM
  - `command`: calls a site-specific episode adapter
  - `ros2`: atomically applies a stopped candidate, resets AWSIM, enables the
    controller, and runs the supplied episode monitor
  - `worker_pool`: dispatches ROS2 evaluations to isolated AWSIM/Autoware
    workers
- export of the reviewed best candidate to static ROS parameter YAML

## Quick verification

From this directory:

```bash
python3 -m pip install -e '.[test]'
PYTHONPATH=src python3 -m pytest
./scripts/run_smoke.bash
```

The default `experiment.yaml` deliberately uses `mock` mode. It cannot move a
vehicle.

An interrupted run can be resumed from its latest generation checkpoint:

```bash
./scripts/resume_search.bash runs/RUN_ID
```

When resuming a parallel run after changing only the worker count:

```bash
make ga-parallel-resume RUN_ID=RUN_ID
```

This keeps the saved population and GA settings while overriding the execution
pool with `worker-1`, `worker-2`, and `worker-3`.

## AWSIM search

Build the overlay after the controller changes, then start AWSIM/Autoware in
GA mode:

```bash
make autoware-build
make ga
```

`make ga` starts both the GA-enabled simulation and the background search
runner. `GA_EXPERIMENT_MODE=true make dev` starts only AWSIM and Autoware;
use `make ga-search` afterwards when those containers are already running.
Monitor progress with `make ga-status` or `make ga-logs`, and stop only the
optimizer with `make ga-stop`.

## Independent three-worker search

For the production search, start three isolated AWSIM/Autoware stacks:

```bash
make ga-parallel
```

Each worker runs in a separate Docker Compose project and bridge network.
Both workers can use ROS Domains 0 and 1 internally without DDS or reset
traffic crossing between them. The central optimizer does not join either ROS
network; it dispatches atomic filesystem jobs and remains the only SQLite
writer.

```bash
make ga-parallel-status
make ga-parallel-logs
make ga-parallel-rviz
make ga-parallel-stop
```

Worker logs are stored below `output/ga-workers/worker-1`,
`output/ga-workers/worker-2`, and `output/ga-workers/worker-3`. Do not rebuild
the shared colcon workspace while a parallel search is active. The architecture
and acceptance criteria are in `../ga_docs/GA_Parallel_Worker_Design.md`.

The local WSL setup uses `AWSIM_HEADLESS=true`: Unity runs with
`-batchmode -nographics`. Parallel workers omit RViz by default to reserve
resources for evaluation; `make ga-parallel-rviz` opens one RViz connected
only to Worker 1 without restarting the running GA. Pure Pursuit does not
require AWSIM camera or LiDAR rendering.

`make dev` also starts the periodic rosbag cleaner. It only scans
`output/**/rosbag2*`, skips bags updated during the last 10 minutes, removes
inactive bags older than 3 hours, and caps their total size at 20 GiB. Override
the defaults in `.env` when a longer retention window is needed:

```dotenv
ROSBAG_RETENTION_MINUTES=180
ROSBAG_ACTIVE_GRACE_MINUTES=10
ROSBAG_CLEAN_INTERVAL_SECONDS=300
ROSBAG_MAX_TOTAL_GB=20
```

Run one cleanup pass manually with `make clean-rosbags`. `make down` stops the
periodic cleaner together with AWSIM and Autoware.

Then, in the command container:

```bash
python3 -m pip install -e aichallenge/ml_workspace/genetic_algorithm
./aichallenge/ml_workspace/genetic_algorithm/scripts/run_search.bash \
  ./aichallenge/ml_workspace/genetic_algorithm/config/experiment_ros2.yaml
```

The ROS evaluator performs this sequence for every candidate:

1. disable and confirm zero-command mode
2. call `set_parameters_atomically`
3. reset AWSIM through Domain 0
4. wait for fresh AWSIM status, stopped odometry, and a non-empty trajectory
5. reset Pure Pursuit history through Domain 1
6. enable and monitor one lap (the current dev AWSIM starts without a separate
   `/admin/awsim/start` command)
7. disable, record metrics, and commit the candidate transaction

`experiment_ros2.yaml` assumes the node is `/simple_pure_pursuit_node` and the
admin reset topic is `/admin/awsim/reset`. Adjust these two integration
points if the running simulator exposes different names.

## Export

```bash
./scripts/export_best.bash runs/RUN_ID \
  ../../workspace/src/aichallenge_submit/aichallenge_submit_launch/config/control/pure_pursuit_selected.param.yaml
```

Export does not modify the active launch automatically. Review and validate
Top-K candidates with the official evaluator before promoting the YAML.

## Speed diagnostics

New episodes store a 5 Hz diagnostic trace without changing the fitness score.
It includes actual, target, and curvature-limited speed; standing-start times
to 10/20/30/34 km/h; speed tracking error; limiter duty; and section splits.
Render a dependency-free HTML chart with:

```bash
PYTHONPATH=src python3 -m ga_pure_pursuit.diagnostics \
  --run-dir runs/RUN_ID \
  --candidate-id CANDIDATE_ID \
  --output runs/RUN_ID/diagnostics/CANDIDATE_ID.html
```

Omit `--candidate-id` to select the run's lowest-fitness completed candidate.
Runs created before this diagnostic trace was added must be evaluated again
before they can be plotted.

## `make ga-*` command memo

Run `make ga-help` from the repository root to print the command list.
The current four-candidate shared-AWSIM workflow is:

```bash
# Start one AWSIM, four independent ghost vehicles, and the GA runner.
make ga-shared

# `make ga-shared` also opens RViz for vehicle/domain 1. Restart only RViz with:
make ga-shared-rviz

# Monitor the runner and locate the latest run.
make ga-logs
make ga-status

# The dashboard is refreshed automatically; this forces a one-time redraw.
make ga-dashboard

# Generate the latest dashboard and serve it over HTTP.
make ga-dashboard-serve

# Stop the runner, AWSIM, and all four Autoware domains.
make ga-shared-stop

# Resume a stopped run after starting shared infrastructure. Stop the newly
# created runner only; keep AWSIM and the four Autoware domains running.
make ga-shared
make ga-stop
make ga-shared-resume RUN_ID=YYYYMMDDTHHMMSSZ
```

The older independent three-worker workflow remains available through
`make ga-parallel`, `make ga-parallel-status`, `make ga-parallel-logs`,
`make ga-parallel-rviz`, `make ga-parallel-resume RUN_ID=...`, and
`make ga-parallel-stop`.

## Live GA progress dashboard

Render the latest run's overview with:

```bash
make ga-dashboard
```

GA search and resume automatically create and refresh
`runs/RUN_ID/dashboard.html`, which shows generation fitness and lap
time, best-candidate speed tracking, baseline versus optimized path, and
normalized gene convergence. To refresh an existing run while no GA runner is
active, use:

```bash
PYTHONPATH=aichallenge/ml_workspace/genetic_algorithm/src python3 \
  -m ga_pure_pursuit.progress_dashboard \
  --run-dir aichallenge/ml_workspace/genetic_algorithm/runs/RUN_ID --watch
```

The browser refreshes the generated page every 15 seconds.

On a remote GA host, serve the generated dashboards with:

```bash
make ga-dashboard-serve
```

Then open the URL printed by the command from another PC, replacing
`0.0.0.0` with the GA host's IP address. The defaults are bind address
`0.0.0.0` and port `18080`; override them when needed:

```bash
make ga-dashboard-serve GA_DASHBOARD_BIND=127.0.0.1 GA_DASHBOARD_PORT=18080
```

The loopback-only form is suitable for SSH port forwarding with
`ssh -L 18080:127.0.0.1:18080 USER@HOST`.

For the shared four-vehicle workflow, `make ga-shared` also starts RViz for
vehicle/domain 1. When invoked over SSH it detects the active host Xwayland
display and Mutter authority automatically, so RViz opens on the GA host's
logged-in desktop. Use `make ga-shared-rviz` to restart RViz without restarting
the GA.

Set `run.generations` to `0` to continue creating generations until the GA
runner is stopped manually. A positive value keeps the original finite
generation limit. Completed generations are checkpointed normally, so an
unlimited run can be resumed after stopping it.

## Episode adapter contract

For a custom `command` evaluator, the process receives:

- `GA_EPISODE_REQUEST`: request JSON path
- `GA_EPISODE_RESULT`: destination metrics JSON path

It must exit zero and write a JSON object containing at least `completed`,
`exit_reason`, `lap_time_seconds` (or `elapsed_seconds`), and `progress`.

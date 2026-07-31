# Shared AWSIM four-ghost PoC

## Goal

Evaluate four GA candidates at once in one AWSIM process. Each vehicle uses an
independent ROS 2 domain, starts at the D1 pose, and cannot physically interact
with another vehicle.

## PoC configuration

- One AWSIM process with four vehicles.
- ROS 2 base domain 1, producing independent domains 1 through 4.
- `collisions: "off"`, which makes AWSIM ignore Vehicle-layer collisions.
- All four vehicle poses are identical to the original D1 entry in
  `Scenarios/scenario1.yaml`.
- Camera and LiDAR are disabled; GNSS, IMU, vehicle state, and control remain
  available for the existing evaluator.

Start it with:

```bash
make ga-ghost4-poc
make awsim-request-start
```

Stop it with:

```bash
make ga-ghost4-poc-stop
```

## Runtime result (2026-08-01)

AWSIM reported that it loaded `ga-ghost-4`, enabled GoKart1 through GoKart4,
and applied all four poses from `ga-ghost-d1-4.yaml`. Each ROS domain exposed
its own localization, vehicle status, control command, and Pure Pursuit GA
services.

The four controllers accepted distinct `external_target_vel` values (5.5,
6.5, 7.5, and 8.5 m/s). After one synchronized start, all four vehicles moved
from the overlapped D1 pose and produced distinct poses and velocities without
collision displacement. This proves the AWSIM and ROS-domain part of the
architecture.

## GA integration boundary

The fitness calculation and `episode_monitor` can remain unchanged and run
once per domain. The orchestration cannot reuse four asynchronous
`Ros2Evaluator` instances unchanged because `/admin/awsim/reset` resets all
four vehicles together.

The production implementation therefore needs a batch coordinator:

1. Assign four candidates to domains 1 through 4.
2. Disable all four controllers and apply all parameters and candidate paths.
3. Issue one global AWSIM reset.
4. Prepare all four domains and wait until all are ready.
5. Reset controller histories, then issue one synchronized start.
6. Run the existing episode monitor independently in each domain.
7. Return the four unchanged metrics objects to the existing fitness function.

Infrastructure failure policy should be batch-wide: if AWSIM itself fails or a
global reset fails, retry the whole batch. A candidate-only failure remains an
individual result.

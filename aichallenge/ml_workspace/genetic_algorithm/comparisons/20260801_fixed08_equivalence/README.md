# Fixed 0.8 acceleration-limit equivalence check

Candidate `g0002-i0020` from run `20260801T023009Z` was evaluated under:

1. one vehicle, AWSIM handicap on (3 runs), and
2. four collision-free vehicles, AWSIM handicap off, with every Pure Pursuit
   controller limited to `longitudinal_acceleration_limit=0.8` (3 batches,
   12 runs).

AWSIM `Assembly-CSharp.dll` IL confirms that the official handicap writes
`AWSIM.Vehicle.MaxAccelerationInput=0.8` for rank 1 and `1.0` otherwise. The
vehicle then clamps `AccelerationInput` to that range each physics update.

| Metric | Single, handicap on | Ghost4, off + fixed 0.8 | Delta |
|---|---:|---:|---:|
| Standing lap | 54.769 s | 54.841 s | +0.072 s (+0.13%) |
| Flying lap | 45.049 s | 45.161 s | +0.113 s (+0.25%) |
| Weighted lap | 47.965 s | 48.065 s | +0.100 s (+0.21%) |
| Mean speed | 6.351 m/s | 6.373 m/s | +0.35% |
| Maximum speed | 8.644 m/s | 8.612 m/s | -0.38% |
| Mean actual acceleration | 0.07655 m/s2 | 0.07667 m/s2 | +0.17% |
| Lateral error p95 | 0.618 m | 0.616 m | -0.27% |
| Distance | 694.495 m | 694.758 m | +0.04% |

The four domain flying-lap means were 45.127, 45.109, 45.250, and 45.159
seconds. Fixed-0.8 timing remained healthy: control command 100.07 Hz,
odometry 49.87 Hz, Pure Pursuit debug 94.11 Hz, and simulation/wall ratio
0.9980.

Conclusion: handicap off plus a fixed 0.8 controller-output limit is an
adequate deterministic GA proxy for the rank-1 handicap. Final candidates
should still be validated with the official single-vehicle handicap-on mode.

Raw remote artifacts:

- `runs/single_handicap_on_20260801_g0002-i0020/summary.json`
- `runs/single_handicap_on_20260801_g0002-i0020/timing_probe.json`
- `runs/ghost4_off_fixed08_20260801_g0002-i0020/summary.json`
- `runs/ghost4_off_fixed08_20260801_g0002-i0020/timing_probe.json`

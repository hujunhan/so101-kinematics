# S2 benchmark: position + approach angle IK (simulation)

200 random reachable targets (pose of random in-limit configurations), seed 0. Success: position error < 2 mm and pitch / roll error < 0.5 deg. Weights: position 1 per m, angles 0.05 per rad. Single run, single thread; times are per solve including Python overhead.

## cold: start from the zero pose

| Solver | Success | Within limits | Failed at a limit | Median | p95 | Median iters |
|---|---:|---:|---:|---:|---:|---:|
| ours: bounded DLS, active set | 173/200 | 200/200 | 23 | 0.73 ms | 2.37 ms | 7 |
| ours: bounded DLS, scipy BVLS | 173/200 | 200/200 | 23 | 0.87 ms | 2.75 ms | 7 |
| baseline: DLS + clamp | 163/200 | 200/200 | 34 | 0.39 ms | 0.95 ms | 9 |
| reference: SLSQP + limits | 186/200 | 200/200 | 11 | 2.19 ms | 2.75 ms | 43 |
| reference: Pink (QP, 6-D pose) | 166/200 | 200/200 | 28 | 0.63 ms | 9.72 ms | 6 |

## far: start 0.3 rad (std per joint) from a solution

| Solver | Success | Within limits | Failed at a limit | Median | p95 | Median iters |
|---|---:|---:|---:|---:|---:|---:|
| ours: bounded DLS, active set | 199/200 | 200/200 | 0 | 0.22 ms | 0.44 ms | 3 |
| ours: bounded DLS, scipy BVLS | 199/200 | 200/200 | 0 | 0.29 ms | 0.56 ms | 3 |
| baseline: DLS + clamp | 197/200 | 200/200 | 2 | 0.15 ms | 0.36 ms | 3 |
| reference: SLSQP + limits | 199/200 | 200/200 | 0 | 2.01 ms | 2.42 ms | 40 |
| reference: Pink (QP, 6-D pose) | 200/200 | 200/200 | 0 | 0.34 ms | 0.67 ms | 3 |

## warm: start 0.05 rad (std per joint) from a solution

| Solver | Success | Within limits | Failed at a limit | Median | p95 | Median iters |
|---|---:|---:|---:|---:|---:|---:|
| ours: bounded DLS, active set | 200/200 | 200/200 | 0 | 0.07 ms | 0.12 ms | 1 |
| ours: bounded DLS, scipy BVLS | 200/200 | 200/200 | 0 | 0.10 ms | 0.17 ms | 1 |
| baseline: DLS + clamp | 200/200 | 200/200 | 0 | 0.07 ms | 0.11 ms | 1 |
| reference: SLSQP + limits | 200/200 | 200/200 | 0 | 1.90 ms | 2.22 ms | 37 |
| reference: Pink (QP, 6-D pose) | 200/200 | 200/200 | 0 | 0.15 ms | 0.25 ms | 1 |

## Unreachable pitch: position reachable, pitch forced to -90 deg (straight down), cold start

When the full pose cannot be reached the solver returns a compromise (position weighted above angles). Error columns are over the cases that did not converge.

| Solver | Reached | Within limits | Compromise cases | Median pos. err | p95 pos. err | Median pitch err |
|---|---:|---:|---:|---:|---:|---:|
| ours: bounded DLS, active set | 40/200 | 200/200 | 160 | 26.4 mm | 64.2 mm | 63.9 deg |
| reference: SLSQP + limits | 40/200 | 200/200 | 160 | 19.3 mm | 47.9 mm | 64.7 deg |

### Orientation weight sweep (ours, same targets, errors over all 200 including the reached ones)

| Angle weight (per rad) | Reached | Median pos. err | p95 pos. err | Median pitch err |
|---:|---:|---:|---:|---:|
| 0.05 | 40/200 | 16.9 mm | 55.8 mm | 42.0 deg |
| 0.02 | 40/200 | 2.7 mm | 45.9 mm | 49.7 deg |
| 0.01 | 39/200 | 0.7 mm | 22.5 mm | 51.4 deg |
| 0.005 | 29/200 | 0.2 mm | 19.7 mm | 51.8 deg |
| 0.002 | 0/200 | 0.0 mm | 19.2 mm | 62.4 deg |

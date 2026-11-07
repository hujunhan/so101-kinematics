# S2 benchmark: position + approach angle IK (simulation)

200 random reachable targets (pose of random in-limit configurations), seed 0. Success: position error < 2 mm and pitch / roll error < 0.5 deg. Weights: position 1 per m, angles 0.01 per rad. Single run, single thread; times are per solve including Python overhead.

## cold: start from the zero pose

| Solver | Success | Within limits | Failed at a limit | Median | p95 | Median iters |
|---|---:|---:|---:|---:|---:|---:|
| ours: bounded DLS, active set | 159/200 | 200/200 | 36 | 1.00 ms | 3.00 ms | 9 |
| ours: bounded DLS, scipy BVLS | 159/200 | 200/200 | 36 | 1.21 ms | 3.44 ms | 9 |
| baseline: DLS + clamp | 163/200 | 200/200 | 34 | 0.49 ms | 1.22 ms | 11 |
| reference: SLSQP + limits | 174/200 | 200/200 | 23 | 3.02 ms | 4.12 ms | 58 |
| reference: Pink (QP, 6-D pose) | 159/200 | 200/200 | 31 | 0.84 ms | 9.98 ms | 8 |

## far: start 0.3 rad (std per joint) from a solution

| Solver | Success | Within limits | Failed at a limit | Median | p95 | Median iters |
|---|---:|---:|---:|---:|---:|---:|
| ours: bounded DLS, active set | 198/200 | 200/200 | 0 | 0.28 ms | 0.62 ms | 4 |
| ours: bounded DLS, scipy BVLS | 198/200 | 200/200 | 0 | 0.38 ms | 0.80 ms | 4 |
| baseline: DLS + clamp | 196/200 | 200/200 | 3 | 0.20 ms | 0.60 ms | 4 |
| reference: SLSQP + limits | 194/200 | 200/200 | 0 | 2.46 ms | 2.95 ms | 48 |
| reference: Pink (QP, 6-D pose) | 199/200 | 200/200 | 1 | 0.36 ms | 1.35 ms | 3 |

## warm: start 0.05 rad (std per joint) from a solution

| Solver | Success | Within limits | Failed at a limit | Median | p95 | Median iters |
|---|---:|---:|---:|---:|---:|---:|
| ours: bounded DLS, active set | 200/200 | 200/200 | 0 | 0.08 ms | 0.15 ms | 1 |
| ours: bounded DLS, scipy BVLS | 200/200 | 200/200 | 0 | 0.10 ms | 0.19 ms | 1 |
| baseline: DLS + clamp | 200/200 | 200/200 | 0 | 0.07 ms | 0.12 ms | 1 |
| reference: SLSQP + limits | 199/200 | 200/200 | 0 | 2.18 ms | 2.53 ms | 43 |
| reference: Pink (QP, 6-D pose) | 200/200 | 200/200 | 0 | 0.15 ms | 0.30 ms | 1 |

## Unreachable pitch: position reachable, pitch forced to -90 deg (straight down), cold start

When the full pose cannot be reached the solver returns a compromise (position weighted above angles). Error columns are over the cases that did not converge.

| Solver | Reached | Within limits | Compromise cases | Median pos. err | p95 pos. err | Median pitch err |
|---|---:|---:|---:|---:|---:|---:|
| ours: bounded DLS, active set | 39/200 | 200/200 | 161 | 8.6 mm | 25.6 mm | 68.7 deg |
| reference: SLSQP + limits | 40/200 | 200/200 | 160 | 0.9 mm | 5.5 mm | 71.1 deg |

### Orientation weight sweep (ours, same targets, errors over all 200 including the reached ones)

| Angle weight (per rad) | Reached | Median pos. err | p95 pos. err | Median pitch err |
|---:|---:|---:|---:|---:|
| 0.05 | 41/200 | 18.5 mm | 59.6 mm | 42.1 deg |
| 0.02 | 40/200 | 9.1 mm | 46.0 mm | 47.7 deg |
| 0.01 | 39/200 | 0.8 mm | 23.6 mm | 51.3 deg |
| 0.005 | 38/200 | 0.2 mm | 20.7 mm | 51.2 deg |
| 0.002 | 32/200 | 0.0 mm | 20.6 mm | 51.0 deg |

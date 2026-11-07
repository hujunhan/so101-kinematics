# S2 stress test: targets near the joint limits (simulation)

500 targets, seed 1: the pose of a configuration with 1-3 random joints 0.5-5 deg inside one of their limits (the others uniform), so each target is reachable. Success: position error < 2 mm and pitch / roll error < 0.5 deg. Starts: cold = zero pose, far = 0.3 rad (std per joint) from the solution, warm = 0.05 rad. Defaults of `solve_ik` (angle weight 0.01 per rad, damping 0.003).

## cold

| Solver | Success | Within limits | Failed at a limit | Median | p95 |
|---|---:|---:|---:|---:|---:|
| ours (active set) | 244/500 | 500/500 | 236 | 1.56 ms | 14.92 ms |
| clamp baseline | 273/500 | 500/500 | 195 | 0.61 ms | 4.06 ms |
| SLSQP | 286/500 | 500/500 | 192 | 2.94 ms | 3.89 ms |
| Pink (6-D pose) | 187/500 | 500/500 | 251 | 9.61 ms | 9.83 ms |

## far

| Solver | Success | Within limits | Failed at a limit | Median | p95 |
|---|---:|---:|---:|---:|---:|
| ours (active set) | 486/500 | 500/500 | 6 | 0.25 ms | 0.66 ms |
| clamp baseline | 481/500 | 500/500 | 16 | 0.18 ms | 0.56 ms |
| SLSQP | 485/500 | 500/500 | 3 | 2.40 ms | 2.96 ms |
| Pink (6-D pose) | 494/500 | 500/500 | 6 | 0.44 ms | 1.60 ms |

## warm

| Solver | Success | Within limits | Failed at a limit | Median | p95 |
|---|---:|---:|---:|---:|---:|
| ours (active set) | 500/500 | 500/500 | 0 | 0.07 ms | 0.17 ms |
| clamp baseline | 500/500 | 500/500 | 0 | 0.07 ms | 0.11 ms |
| SLSQP | 499/500 | 500/500 | 0 | 1.99 ms | 2.51 ms |
| Pink (6-D pose) | 500/500 | 500/500 | 0 | 0.15 ms | 0.35 ms |

## Paired comparison: targets where only one of the two solvers succeeded

| Start | vs | Only ours | Only the other |
|---|---|---:|---:|
| cold | clamp baseline | 35 | 64 |
| cold | SLSQP | 14 | 56 |
| cold | Pink (6-D pose) | 94 | 37 |
| far | clamp baseline | 13 | 8 |
| far | SLSQP | 9 | 8 |
| far | Pink (6-D pose) | 4 | 12 |
| warm | clamp baseline | 0 | 0 |
| warm | SLSQP | 1 | 0 |
| warm | Pink (6-D pose) | 0 | 0 |

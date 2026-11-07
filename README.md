# so101-kinematics

M1 of the [SO-101 series](https://github.com/hujunhan/so101-core). It covers kinematics on the SO-101 arm:

- My 2023 learning library, [arm_control](https://github.com/hujunhan/arm_control), is benchmarked against
  [Pinocchio](https://github.com/stack-of-tasks/pinocchio).
- A joint-limited IK that solves for position **plus an approach angle**.
- The real arm draws shapes, with the tracking error measured.

> **Status (2026-11-06):** done (S1-S4). Open points are handed to M2, see the end.

## Summary

- **The 2023 code is correct but slow, and blind to joint limits.** Its FK matches Pinocchio to 1e-16 m. Its
  IK takes about 70 ms, and 31% of its solutions are joint angles the arm cannot take; only its SLSQP solver
  respects the limits, at 0.6 s per solve.
- **A new IK solves position + approach angle (pitch, roll) within the joint limits in about 1 ms cold and
  0.1 ms warm**, 3-25x faster than SLSQP. On this arm pitch and roll are linear in the joints, so pointing
  straight down is not a singularity.
- **It is not better than naive clamping at finding solutions.** Every solver kept the limits; from a cold
  start the bounded step found fewer solutions than clamping, even next to the limits, because it settles
  into local minima on a limit. That is a seeding problem, left open. Warm-started along a path, all of them
  succeed.
- **The real arm drew a square and a circle through the safety layer**, at 2 and 5 cm/s, 4-9 mm from the
  command (median). The error is mostly a ~100 ms servo delay and 0.9 deg of elbow sag, not the IK.
- **The model and the real arm disagree:** 10 cm on the table measures 8.66 cm by FK. Calibration is M2.

## Plan

| Session | Content | Output |
|---|---|---|
| **S1** | arm_control packaged and fixed (the 2023 version is tagged `v2023`); simulation benchmark. FK accuracy against Pinocchio; position-only IK solvers compared on success rate, time and joint-limit violations | `results/s1_benchmark.md` |
| **S2** | New IK: position + approach angle (gripper pitch, wrist roll), damped least squares with joint limits; drawing in simulation | `src/so101_kinematics/ik.py`, `results/s2_benchmark.md`, `results/s2_drawing.md` |
| **S3** | The real arm draws on a vertical plane ~20 cm in front of the base; commanded vs measured trajectories; tracking error and gravity sag | `so101-draw`, `results/s3_drawing.md` (no video, see DECISIONS) |
| **S4** | Write-up and decisions/lessons | This README |

## S1 result: the 2023 code on the SO-101 (simulation)

Full table: [results/s1_benchmark.md](results/s1_benchmark.md). There were 200 random reachable targets.
Every solver starts from the zero pose, and success means a position error under 2 mm.

| Solver | Success | ...within joint limits | Median time |
|---|---:|---:|---:|
| arm_control `ik_jacobian`, pseudo-inverse (2023) | 200/200 | **139/200** | 69 ms |
| arm_control `ik_jacobian`, transpose (2023) | 16/200 | 16/200 | 3.7 s |
| arm_control `ik_scipy`, SLSQP with limits (2023) | 192/200 | **192/200** | 621 ms |
| Pinocchio damped least squares (reference, no limits) | 200/200 | **85/200** | **0.2 ms** |

- **FK:** the 2023 code matches Pinocchio to 3.6e-16 m over 1000 configurations. It is correct, not just close.
- **Speed** comes mostly from the implementation, not the algorithm. Pseudo-inverse and DLS are nearly the
  same update. The 2023 code evaluates sympy-generated expressions in Python, and its half-size steps
  (alpha 0.5) need more iterations: about 70 ms per solve, against 0.2 ms for Pinocchio's C++.
- **Joint limits are the real problem.** Unconstrained solvers reach the target with joint angles the arm
  cannot take: 31% of the 2023 pseudo-inverse solutions and 58% of Pinocchio DLS (it takes full Newton
  steps and wanders further). Only the constrained SLSQP respects the limits, at about 3,000× the cost of
  DLS. Sending these solutions to the real arm is what the so101-core safety layer would clip.
- **S2 target:** a solver that respects limits like SLSQP and runs at DLS speed, about 1 ms.

Caveats: the transpose method's step size (0.5) was not tuned. Timings are single-threaded on an
i9-11900H.

## S2 result: joint-limited IK for position + approach angle (simulation)

Full tables: [results/s2_benchmark.md](results/s2_benchmark.md) and [results/s2_drawing.md](results/s2_drawing.md).
Each iteration solves a damped, weighted least squares for the joint step with the joint limits as bounds on
that step (a small active-set loop). Pitch and roll are linear in the joints on this arm
(pitch = -(lift + elbow + wrist_flex), roll = wrist_roll + const, checked against the URDF), so they cost
nothing extra and have no singularity at straight down. 200 random reachable targets, success = under 2 mm
and 0.5 deg:

| Start | ours | naive clamp | SLSQP | Pink (QP) |
|---|---:|---:|---:|---:|
| cold (zero pose) | 159/200, 1.0 ms | 163/200, 0.5 ms | **174/200**, 3.0 ms | 159/200, 0.8 ms |
| far (0.3 rad away) | 198/200, 0.28 ms | 196/200 | 194/200, 2.5 ms | 199/200 |
| warm (0.05 rad, a trajectory step) | **200/200**, 0.08 ms | 200/200 | 199/200, 2.2 ms | 200/200 |

- **Limits are respected by every solver here** (200/200 inside the limits, the clamp baseline too).
  On random in-limit targets the bounded step does **not** clearly beat plain clamping: cold start is even a
  little worse (159 vs 163).
- **Stress test with targets 0.5-5 deg from a limit** ([results/s2_near_limits.md](results/s2_near_limits.md),
  500 targets): the bounded step still does not win. Cold: ours 244, clamp 273, SLSQP 286, Pink 187 of 500
  (only-ours vs only-clamp: 35 vs 64). Far start: 486 vs 481 (13 vs 8, within noise). Warm: 500 vs 500.
  Diagnosis of the 256 cold failures of ours: 174 end at a genuine constrained stationary point, a local
  minimum on the limit boundary. Clamping rarely does (6 of 227); its steps are not descent steps, so it
  sometimes slides out. So the bounded step is a correct local solver, and its remaining failures are a
  seeding problem (where to start), not a stepping problem. The reason clamping escapes more often is my
  reading of those numbers, not something I tested.
- **Speed:** about 1 ms from a cold start and 0.1 ms warm, 3-25x faster than SLSQP. The cold-start median
  was 0.75 ms before the weights below were changed.
- **Cold start is the weak spot** (SLSQP finds 15 more, 42 more near the limits). Drawing uses warm starts,
  where it is 200/200 in both tests.
- **Position first is only soft.** With 5 constraints for 5 joints there is no null space, so the priority
  between position and angles is a weight. At 0.05 per rad an unreachable pitch cost about 17 mm of
  position; 0.01 per rad brings that to 0.8 mm. The weight and the damping must be chosen together (a smaller
  weight with the old damping dropped cold-start success from 173 to 145). The median position error in the
  hard cases where the pitch cannot be reached is 8.6 mm for ours against 0.9 mm for SLSQP, so the
  compromise is not yet as good as it could be.
- **Drawing (simulated):** a circle (radius 3.6 cm) and a square on the plane 20 cm in front of the base,
  tool horizontal (its yaw follows shoulder_pan, so it is not perpendicular to the plane off the centre
  line), 2 mm between points, each point warm-started from the previous one: all points
  converge, position error at most 0.2 mm (the solver tolerance), pitch/roll within 0.1 deg, no joint jumps (largest step 1.4 deg per
  2 mm), 0.12 ms per point. The shapes are sized to keep every joint at least 10 deg from its limit; the
  reachable region at pitch 0 is bounded at the top by shoulder_lift, and the largest circle would come
  within 2 deg of that limit.

![reachable region](results/s2_workspace.png)

## S3 result: the real arm draws (2026-11-06)

Full tables and figures: [results/s3_drawing.md](results/s3_drawing.md). The square (5.1 cm) and the circle
(7.2 cm) were each drawn at 2 and 5 cm/s, on the plane 20 cm in front of the base, through `SafeArm` at 30 Hz
(`so101-draw`, every command safety-filtered, stall watch 5 deg / 0.3 s). All four runs completed; the safety
layer changed no command. The tool positions below are the FK of the commanded and of the measured joints, so
they describe how the servos follow, not the absolute accuracy of the model.

| Run | Error median / max | Constant offset (down) | Lag along the path |
|---|---:|---:|---:|
| square, 2 cm/s | 5.9 / 9.7 mm | 4.9 mm | 2.6 mm |
| circle, 2 cm/s | 3.8 / 9.5 mm | 3.5 mm | 2.8 mm |
| square, 5 cm/s | 8.6 / 12.1 mm | 5.1 mm | 5.6 mm |
| circle, 5 cm/s | 6.1 / 11.1 mm | 3.4 mm | 6.0 mm |

- **The error has three parts**, from a per-joint fit over all runs (R2 0.87-0.95):
  error = offset - sign(velocity) * (deadband + delay * |velocity|).
  - **Delay about 100 ms** on every joint (97-113 ms). It is why the lag along the path doubles from 2 to
    5 cm/s. About one tick (33 ms) of it is the measurement: each tick reads right after sending.
  - **Gravity sag**, mostly the elbow: +0.9 deg, about 4-5 mm down at the tool. It does not change with speed.
  - **A small deadband**, 0.1-0.4 deg (largest on the elbow). My first reading of the square plot blamed
    backlash and deadband for the direction-dependent error; the fit shows the delay explains most of it.
- **Shapes come out taller than commanded** (square 5.09 -> 5.63 cm high at 2 cm/s): the bottom edge is drawn
  about 8 mm low and the top edge about 4 mm low. I have not worked out why the sag differs between the two
  edges; the joint model above does not explain it.
- **Known issue, not investigated:** a torque-off ruler check gave 8.66 cm by FK for 10 cm on the table, with
  the heights consistent to 1.6 mm. A calibration zero offset of a few degrees on the elbow or wrist would
  explain it (two points cannot tell which). That is M2's subject (calibration); M1 left it.

## Lessons

- **Test the claim, not just the code.** The bounded step was supposed to beat clamping. A baseline, then a
  stress test next to the limits, showed it does not, and a diagnosis (projected gradient at the end point)
  showed why. Without the baseline the README would have claimed the opposite.
- **Use the structure of the arm.** Three parallel flex axes make pitch a sum of joint angles. Euler angles
  would have been singular at straight down, exactly the pose grasping needs.
- **Parameters interact.** Lowering the angle weight alone dropped cold-start success from 173 to 145 of 200,
  because the damping then dominated the angle rows. Weight and damping have to be tuned together.
- **The safety layer cannot see a wrong solution.** It clips limits and caps steps; a non-converged IK point
  or a jump to another elbow branch passes it. The path is now checked before it is sent.
- **Vary one thing to separate causes.** One run at one speed suggested backlash; two speeds showed the lag
  grows with speed, so it is mostly delay. The first reading was wrong.
- **Simulation is not the arm.** Everything matched in simulation; a ruler on the table did not (8.66 vs
  10 cm). Reachability was also not the limit for the drawing size: the joint-limit margin was.

## Reproduce

```bash
uv sync                                    # simulation only
uv run pytest
uv run python benchmarks/s1_benchmark.py   # S1
uv run python benchmarks/s2_benchmark.py   # S2: cold / far / warm, unreachable pitch, weight sweep
uv run python benchmarks/s2_near_limits.py # S2: targets next to the limits
uv run python benchmarks/s2_drawing.py     # S2: reachable region, simulated drawing
SO101_CONFIG_DIR=<so101-core>/config uv run python benchmarks/s2_replay.py  # MuJoCo replay

uv sync --extra hw                         # real arm (LeRobot)
SO101_CONFIG_DIR=<so101-core>/config uv run so101-draw readonly
SO101_CONFIG_DIR=<so101-core>/config uv run so101-draw draw --shape square --speed 2
uv run python benchmarks/s3_analyze.py     # S3: analysis of results/s3/*.npz
```

`so101-draw` asks for `yes` before powering the arm and before each motion, and for Enter before releasing
torque, so run it in a terminal next to the arm.

| File | Role |
|---|---|
| `src/so101_kinematics/models.py` | The two backends on the same URDF: arm_control (2023) and Pinocchio |
| `src/so101_kinematics/ik.py` | The IK: position + pitch + roll, bounded damped least squares (active set), clamp and BVLS variants |
| `src/so101_kinematics/references.py` | SLSQP and Pink on the same task, for comparison only |
| `src/so101_kinematics/drawing.py` | Shapes, warm-started joint paths, the checks before a path is sent |
| `src/so101_kinematics/s3.py` | `so101-draw`: the real-arm stages, all through so101-core's `SafeArm` |

## Handed to M2

- Calibration: the 10 cm ruler check (8.66 cm by FK), likely a zero offset on the elbow or wrist.
- Feed-forward compensation: command ~100 ms ahead and add the elbow sag, as a direct test of the joint model.
- Cold-start IK: a structured seed (roll and the pitch sum are known exactly; the rest is a planar
  two-link problem) for the local minima on the limits.
- The compromise when the pitch is unreachable (8.6 mm median, against 0.9 mm for SLSQP).
- A video: the ZED 2 needs to face the drawing plane and keep the room out of frame.

## Why "position + approach angle"

The SO-101 has five arm joints, so a full 6-D pose (3 position + 3 orientation) is over-constrained.
shoulder_pan sets the azimuth, so the gripper's yaw is fixed by the target position. The remaining freedom
can hold the gripper's **pitch**, for example pointing straight down for a top-down grasp, and its **roll**
about its own axis. That makes 5 constraints for 5 joints, which is what grasping in M2 and M5 needs.

## Design and decisions

Why the code is built this way, with diagrams and rejected alternatives: [docs/DESIGN.md](docs/DESIGN.md).
The theory behind it (kinematics, damped least squares, bounded least squares, servo following):
[docs/THEORY.md](docs/THEORY.md).
The dated decision log: [DECISIONS.md](DECISIONS.md).

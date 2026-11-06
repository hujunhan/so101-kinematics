# so101-kinematics

M1 of the [SO-101 series](https://github.com/hujunhan/so101-core). It covers kinematics on the SO-101 arm:

- My 2023 learning library, [arm_control](https://github.com/hujunhan/arm_control), is benchmarked against
  [Pinocchio](https://github.com/stack-of-tasks/pinocchio).
- A joint-limited IK that solves for position **plus an approach angle**.
- The real arm draws shapes, with the tracking error measured.

> **Status (2026-11-06):** S1 done (simulation benchmark below); S2 next.

## Plan

| Session | Content | Output |
|---|---|---|
| **S1** | arm_control packaged and fixed (the 2023 version is tagged `v2023`); simulation benchmark. FK accuracy against Pinocchio; position-only IK solvers compared on success rate, time and joint-limit violations | `results/s1_benchmark.md` |
| **S2** | New IK: position + approach angle (gripper pitch, wrist roll), damped least squares with joint limits; drawing in simulation | Solver + simulated drawing |
| **S3** | The real arm draws on a vertical plane ~20 cm in front of the base; commanded vs measured trajectories; tracking error and gravity sag | Video, error plots |
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
- **Speed** comes from the implementation, not the algorithm. Pseudo-inverse and DLS are nearly the same
  update, but evaluating a sympy-generated Jacobian costs about 70 ms, against 0.2 ms for Pinocchio's C++.
- **Joint limits are the real problem.** Unconstrained solvers reach the target with joint angles the arm
  cannot take: 31% of the 2023 pseudo-inverse solutions and 58% of Pinocchio DLS (it takes full Newton
  steps and wanders further). Only the constrained SLSQP respects the limits, at about 3,000× the cost of
  DLS. Sending these solutions to the real arm is what the so101-core safety layer would clip.
- **S2 target:** a solver that respects limits like SLSQP and runs at DLS speed, about 1 ms.

Caveats: the transpose method's step size (0.5) was not tuned. Timings are single-threaded on an
i9-11900H.

## Why "position + approach angle"

The SO-101 has five arm joints, so a full 6-D pose (3 position + 3 orientation) is over-constrained.
shoulder_pan sets the azimuth, so the gripper's yaw is fixed by the target position. The remaining freedom
can hold the gripper's **pitch**, for example pointing straight down for a top-down grasp, and its **roll**
about its own axis. That makes 5 constraints for 5 joints, which is what grasping in M2 and M5 needs.

## Decisions

See [DECISIONS.md](DECISIONS.md).

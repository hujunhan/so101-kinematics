# Working notes for Claude

M1 of the SO-101 series. Plan, status and results: README.md. Architecture and design rationale: docs/DESIGN.md.
Theory behind the methods: docs/THEORY.md.
Decisions so far: DECISIONS.md.
The series plan (M0–M5) is in the so101-core README.

## Current state
- S1 done: arm_control (2023) vs Pinocchio benchmark, `results/s1_benchmark.md`.
- S2 done (simulation): `src/so101_kinematics/ik.py` (bounded damped least squares, position + pitch +
  roll, active-set step), reference solvers in `references.py` (SLSQP, Pink), `results/s2_benchmark.md`,
  simulated circle and square on the plane x = 20 cm in `results/s2_drawing.md`. Decisions in DECISIONS.md.
  Open points: cold-start success is below SLSQP (159 vs 174 of 200; 244 vs 286 of 500 near the limits)
  and below plain clamping near the limits; most cold failures are genuine local minima on a limit, so the
  fix is seeding (structured start: roll and the pitch sum are known exactly; or closed-form planar IK), not a
  better step. Compromise quality when the pitch is unreachable: 8.6 mm vs 0.9 mm for SLSQP.
  `results/s2_near_limits.md` shows the bounded step does not beat clamping, so do not claim it does.
- S3 done (2026-11-06): `so101-draw` (`src/so101_kinematics/s3.py`, stages readonly / start / draw, all
  through SafeArm; `SafeArm.follow` was added to so101-core for it). Four runs (square, circle x 2, 5 cm/s) in
  `results/s3/`, analysis `benchmarks/s3_analyze.py` -> `results/s3_drawing.md`: ~100 ms delay, elbow sag
  0.9 deg, deadband 0.1-0.4 deg. Known issue left for M2: a ruler check gave 8.66 cm by FK for 10 cm.
- Real-arm runs: the user runs `so101-draw` in their own terminal (it asks for `yes` and for Enter before
  releasing torque, which Claude's shell cannot answer) and pastes the output; Claude analyzes the logs.
  Torque-off reads Claude can run directly.
- S4 done (2026-11-06): README is the write-up (summary, results, lessons, reproduce, handed to M2).
  **M1 is complete.** Next is M2 (`so101-calib`) in its own repo; its starting points are in README
  "Handed to M2". Change this repo only for fixes.

## How we work
- Claude writes and runs most of the code. The user does physical work on the arm, makes the key
  decisions and learns the concepts. Present options with a recommendation and let the user decide
  design choices, then record them in DECISIONS.md (date, decided, why, options).
- The user must be able to defend every choice, so explain the why in plain terms, briefly.
- Talk to the user in Chinese. Code, comments, commits and repo docs are in English.
- Small steps; one session ≈ 1–2 hours of the user's time.

## Rules
- The repo is public. Never commit personal information or device serial numbers.
- Real-arm motion only through so101-core's `SafeArm` (limit margin, step cap, stall detection), never raw
  bus writes. Tell the user before anything moves the arm; they must be present with power in reach.
- Dependencies are pinned git commits (so101-core, arm_control). If a fix belongs in so101-core or
  arm_control, make it there, push, then bump the pin here.
- arm_control: keep its CRLF line endings; the 2023 state is tag `v2023`.
- Tooling: uv, Python 3.12, ruff, pytest. Run `uv run ruff check . && uv run pytest` before committing.
- Commit and push when a step is done and verified. After a session milestone, update the status line in
  README.md and the project row in the user's twin repo (`~/code/digital-twin/projects/README.md`).

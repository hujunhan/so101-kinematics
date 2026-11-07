# Working notes for Claude

M1 of the SO-101 series. Plan, status and results: README.md. Decisions so far: DECISIONS.md.
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
- **Next: S3**, the real arm draws the shapes. Before anything moves the arm: check so101-core's SafeArm
  limit margin against the 10 deg margin used for the shapes, decide the drawing speed, tell the user, and
  have them present with power in reach. Then measure commanded vs measured trajectory, tracking error and
  gravity sag. S4 is the write-up.

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

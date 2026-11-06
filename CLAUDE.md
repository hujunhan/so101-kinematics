# Working notes for Claude

M1 of the SO-101 series. Plan, status and results: README.md. Decisions so far: DECISIONS.md.
The series plan (M0–M5) is in the so101-core README.

## Current state
- S1 done: arm_control (2023) vs Pinocchio benchmark, `results/s1_benchmark.md`.
- **Next: S2**, simulation only. Build the new IK: position + approach angle (gripper pitch and wrist roll,
  5 constraints for 5 joints), damped least squares that respects joint limits, aiming at SLSQP-level limit
  compliance at about 1 ms. Then a simulated drawing on a vertical plane ~20 cm in front of the base.
  Start by laying out the candidate ways to handle joint limits (clamping, null space, weighting, QP) with
  their trade-offs and let the user choose.
- S3 (real arm drawing, tracking error, gravity sag) and S4 (write-up) follow.

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

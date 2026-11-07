# Decisions

## 2026-11-06: fix the old code in its own repo
- **Decided:** package and fix [arm_control](https://github.com/hujunhan/arm_control) in place, not copy it or
  pin it as a submodule.
- **What was done:**
  - The repo was archived, so it was unarchived.
  - The 2023 state is kept as tag `v2023`.
  - The fixes are minimal: packaging, and the joint count no longer hard-coded to 7.
  - Smoke tests cover the fix; they fail on `v2023`.
- **Why:** one library with its history, reusable by `pip install`, while the tag still lets anyone see
  "the 2023 code as written".
- **Options:** submodule (old repo untouched), copy into `legacy/`, fork to a new repo.

## 2026-11-06: IK target = position + approach angle
- **Decided:** solve for xyz plus the gripper's pitch and roll (5 constraints, 5 joints), not position only
  and not a full 6-D pose.
- **Why:** this is what top-down grasping in M2 and M5 needs, and it makes the 5-DOF limit explicit
  (yaw is fixed by shoulder_pan).
- **Options:** position only (what the 2023 code does); full 6-D least squares (usually unreachable exactly).

## 2026-11-06: demo on a vertical plane in front of the base
- **Decided:** the real arm draws in the air on a vertical plane ~20 cm in front of the base.
- **Why:** no contact with the table, so the safest first powered demo.
- **Options:** horizontal plane above the table (risk of hitting it); pen on paper (needs contact, which
  position-controlled servos handle poorly).

## 2026-11-06: joint limits in the new IK = bounded damped least squares at every step
- **Decided:** each IK iteration solves `min ||J dq - e||^2 + lambda^2 ||dq||^2` subject to
  `q_min - q <= dq <= q_max - q` (limits as hard constraints inside the step), not clamping after the step.
  Naive clamping is kept as a baseline, SLSQP as the ground truth, Pink (differential IK on Pinocchio, QP with configuration limits) as the main external reference, placo (what LeRobot uses for SO-101) optional later.
- **Why:** the arm has 5 constraints for 5 joints, so there is no null space to push joints away from
  limits; the limit has to be part of the step itself. Differential-IK libraries (Pink, mink) do the same
  with a QP, and the saturation-in-the-null-space (SNS) method freezes joints that would exceed their
  bounds. Plain clamping breaks the iteration direction and is what TRAC-IK backs up with SLSQP.
- **Known weakness:** this is a local method and can stall against a limit (Pink documents the same).
  Mitigation: warm start from the previous solution, retry from another seed; count "stuck at a limit"
  as its own failure class in the benchmark.
- **Options:** clamping after each step; null-space projection (no null space here, so ruled out);
  weighting / active-set freezing (SNS-style); full QP with a solver dependency; closed-form IK
  (possible for this arm, kept as an optional cross-check).

## 2026-11-06: approach angle definition and what to do when it is unreachable
- **Decided:** (1) pitch = angle of the tool approach axis inside the arm's vertical plane (sum of
  shoulder_lift, elbow, wrist_flex plus a constant offset, to be verified against the URDF); 0 = horizontal
  pointing forward, negative = pointing down, so straight down is -90 deg. Roll = wrist_roll relative to the
  arm's vertical plane. (2) When the pose is unreachable, the solver returns a weighted least-squares
  compromise (position weight >> orientation weight, weights are parameters) together with the position and
  orientation residuals; callers can apply a strict threshold and treat anything above it as failure.
- **Why:** Euler angles are singular exactly at straight down, which is the top-down grasp M2/M5 need; the
  in-plane angle is smooth and linear in the joints. A compromise degrades gracefully near the workspace
  edge, which matters for a smooth trajectory on the real arm. With no null space the priority is soft
  (weights), not a hard guarantee.
- **Options:** Euler angles; rotation-matrix error; fail immediately when unreachable; strict task priority
  (needs a null space this arm does not have).

## 2026-11-06: angle weight 0.01 per rad, damping 0.003
- **Decided:** default weights position 1 per m, angles 0.01 per rad, damping 0.003.
- **Why:** with 0.05 per rad the compromise for an unreachable pitch gave up ~17 mm of position (median);
  0.01 brings that to ~0.8 mm. But the angle rows scale with the weight, so the old damping 0.01 halved the
  angle steps and cold-start success fell 173 -> 145 of 200; damping 0.003 recovers 159/200. Damping and
  weight have to be chosen together.
- **Cost:** cold-start success 173 -> 159 of 200 against the old weight; warm and far starts are unaffected.
- **Options:** keep 0.05 (better convergence, poor position priority); smaller weights (0.005, 0.002) gave
  fewer fully reached poses.

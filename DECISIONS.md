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

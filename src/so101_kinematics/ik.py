"""Joint-limited IK for position + approach angle (pitch, roll) on the SO-101.

Task space (5 constraints for 5 joints): tool position xyz, plus
- pitch: elevation of the tool approach axis (the tool frame's z axis) inside the arm's vertical plane.
  0 = horizontal pointing forward, negative = pointing down (straight down = -90 deg).
- roll: rotation about the approach axis, 0 when the tool y axis is perpendicular to the arm's plane.

shoulder_lift, elbow_flex and wrist_flex all turn about the same horizontal axis, so pitch and roll are
linear in the joints (checked against the URDF geometry in tests/test_ik.py):
    pitch = -(q_lift + q_elbow + q_wrist_flex),   roll = q_wrist_roll + ROLL_OFFSET

Every step solves a damped, weighted least squares for the joint increment dq with the joint limits
as bounds on dq (`solve_ik`). `solve_ik_clamp` is the naive baseline: unconstrained step, then clip.
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import lsq_linear

from so101_kinematics.models import HI, LO, N, PinArm

ROLL_OFFSET = -0.0486795  # rad; wrist_roll axis tilt in the URDF, measured against the geometry (-2.789 deg)
PITCH_ROW = np.array([0.0, -1.0, -1.0, -1.0, 0.0])
ROLL_ROW = np.array([0.0, 0.0, 0.0, 0.0, 1.0])


def pitch_of(q: np.ndarray) -> float:
    return float(PITCH_ROW @ q)


def roll_of(q: np.ndarray) -> float:
    return float(q[4] + ROLL_OFFSET)


def wrap(a: float) -> float:
    """Wrap an angle error to (-pi, pi]."""
    return float(np.arctan2(np.sin(a), np.cos(a)))


@dataclass
class IKResult:
    q: np.ndarray
    pos_err: float  # m, norm of the position error
    pitch_err: float  # rad, target - achieved (signed)
    roll_err: float  # rad, target - achieved (signed)
    iterations: int
    converged: bool  # position and both angle errors within tolerance
    at_limit: bool  # some joint sits on a bound (useful to tell "stuck at a limit" failures)


def _result(arm, q, target_xyz, pitch, roll, it, tol_pos, tol_ang):
    pos_err = float(np.linalg.norm(target_xyz - arm.fk(q)[:3, 3]))
    pe, re = wrap(pitch - pitch_of(q)), wrap(roll - roll_of(q))
    ok = pos_err < tol_pos and abs(pe) < tol_ang and abs(re) < tol_ang
    at_limit = bool(np.any((q <= LO + 1e-9) | (q >= HI - 1e-9)))
    return IKResult(q, pos_err, pe, re, it, ok, at_limit)


def _task_error_and_jacobian(arm: PinArm, q, target_xyz, pitch, roll, w_pos, w_ori):
    """Weighted task error (5,) and Jacobian (5x5). Rows: xyz, pitch, roll."""
    e = np.empty(5)
    e[:3] = w_pos * (target_xyz - arm.fk(q)[:3, 3])
    e[3] = w_ori * wrap(pitch - pitch_of(q))
    e[4] = w_ori * wrap(roll - roll_of(q))
    J = np.vstack([w_pos * arm.jacobian(q)[:3], w_ori * PITCH_ROW, w_ori * ROLL_ROW])
    return e, J


def solve_ik(
    arm: PinArm,
    target_xyz,
    pitch: float,
    roll: float,
    q0,
    *,
    w_pos: float = 1.0,
    w_ori: float = 0.05,  # per rad; 1 deg of angle error costs about as much as 0.9 mm of position error
    damping: float = 0.01,
    max_step: float = 0.5,  # rad, per joint per iteration
    tol_pos: float = 0.002,
    tol_ang: float = np.radians(0.5),
    max_iter: int = 100,
) -> IKResult:
    """Bounded damped least squares: the joint limits are hard bounds on every step."""
    target_xyz = np.asarray(target_xyz, dtype=float)
    q = np.clip(np.asarray(q0, dtype=float), LO, HI)
    eye = damping * np.eye(N)
    zero = np.zeros(N)
    for it in range(max_iter):
        res = _result(arm, q, target_xyz, pitch, roll, it, tol_pos, tol_ang)
        if res.converged:
            return res
        e, J = _task_error_and_jacobian(arm, q, target_xyz, pitch, roll, w_pos, w_ori)
        lb = np.maximum(LO - q, -max_step)
        ub = np.minimum(HI - q, max_step)
        dq = lsq_linear(np.vstack([J, eye]), np.concatenate([e, zero]), bounds=(lb, ub), method="bvls").x
        if np.linalg.norm(dq) < 1e-10:  # no admissible step improves the error: stalled (maybe at a limit)
            break
        q = q + dq
    return _result(arm, q, target_xyz, pitch, roll, it, tol_pos, tol_ang)


def solve_ik_clamp(
    arm: PinArm,
    target_xyz,
    pitch: float,
    roll: float,
    q0,
    *,
    w_pos: float = 1.0,
    w_ori: float = 0.05,
    damping: float = 0.01,
    max_step: float = 0.5,
    tol_pos: float = 0.002,
    tol_ang: float = np.radians(0.5),
    max_iter: int = 100,
) -> IKResult:
    """Baseline: unconstrained damped least-squares step, then clip the joints to their limits."""
    target_xyz = np.asarray(target_xyz, dtype=float)
    q = np.clip(np.asarray(q0, dtype=float), LO, HI)
    for it in range(max_iter):
        res = _result(arm, q, target_xyz, pitch, roll, it, tol_pos, tol_ang)
        if res.converged:
            return res
        e, J = _task_error_and_jacobian(arm, q, target_xyz, pitch, roll, w_pos, w_ori)
        dq = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(5), e)
        norm = np.abs(dq).max()
        if norm > max_step:
            dq *= max_step / norm
        q = np.clip(q + dq, LO, HI)
    return _result(arm, q, target_xyz, pitch, roll, it, tol_pos, tol_ang)

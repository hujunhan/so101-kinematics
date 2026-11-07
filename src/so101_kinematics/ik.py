"""Joint-limited IK for position + approach angle (pitch, roll) on the SO-101.

Task space (5 constraints for 5 joints): tool position xyz, plus
- pitch: elevation of the tool approach axis (the tool frame's z axis) inside the arm's vertical plane.
  0 = horizontal pointing forward, negative = pointing down (straight down = -90 deg).
- roll: rotation about the approach axis, 0 when the tool y axis is perpendicular to the arm's plane.

shoulder_lift, elbow_flex and wrist_flex all turn about the same horizontal axis, so pitch and roll are
linear in the joints (checked against the URDF geometry in tests/test_ik.py):
    pitch = -(q_lift + q_elbow + q_wrist_flex),   roll = q_wrist_roll + ROLL_OFFSET

Each iteration solves a damped, weighted least squares for the joint increment dq:
    min ||J dq - e||^2 + damping^2 ||dq||^2     (J, e weighted: position vs angles)
and `method` decides how the joint limits enter:
- "activeset": limits are bounds on dq, solved by a small active-set loop (fast, the main solver)
- "bvls":      the same bounded problem through scipy's BVLS (reference implementation of the step)
- "clamp":     unconstrained step, then clip the joints to the limits (naive baseline)
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import lsq_linear

from so101_kinematics.models import HI, LO, N, PinArm

ROLL_OFFSET = -0.0486795  # rad; wrist_roll axis tilt in the URDF, measured against the geometry (-2.789 deg)
PITCH_ROW = np.array([0.0, -1.0, -1.0, -1.0, 0.0])
ROLL_ROW = np.array([0.0, 0.0, 0.0, 0.0, 1.0])
METHODS = ("activeset", "bvls", "clamp")
TOL_ANG = np.radians(0.5)  # default angle tolerance for "converged"


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
    at_limit: bool  # some joint sits on a bound (tells "stuck at a limit" failures apart)


def bounded_dls_step(J: np.ndarray, e: np.ndarray, damping: float, lb: np.ndarray, ub: np.ndarray):
    """min ||J dq - e||^2 + damping^2 ||dq||^2  s.t.  lb <= dq <= ub  (primal active set).

    Solve the free variables, clamp the ones that left their bounds, re-solve; then release a clamped
    variable if its gradient says the cost would drop by moving back inside (KKT check).
    """
    H = J.T @ J + damping**2 * np.eye(N)
    g = J.T @ e
    dq = np.linalg.solve(H, g)
    if np.all(dq >= lb) and np.all(dq <= ub):  # fast path: no limit is hit, plain damped least squares
        return dq
    fixed = np.zeros(N, dtype=bool)
    dq = np.zeros(N)
    for _ in range(3 * N):
        free = ~fixed
        if free.any():
            rhs = g[free] - H[np.ix_(free, fixed)] @ dq[fixed]
            dq[free] = np.linalg.solve(H[np.ix_(free, free)], rhs)
        low, high = free & (dq < lb), free & (dq > ub)
        if low.any() or high.any():
            dq[low], dq[high] = lb[low], ub[high]
            fixed |= low | high
            continue
        grad = H @ dq - g  # d(cost)/2 / d dq
        wants_in = ((dq <= lb) & (grad < -1e-12)) | ((dq >= ub) & (grad > 1e-12))
        release = np.where(fixed & wants_in, np.abs(grad), 0)
        if not release.any():
            break
        fixed[np.argmax(release)] = False
    return dq


def solve_ik(
    arm: PinArm,
    target_xyz,
    pitch: float,
    roll: float,
    q0,
    *,
    method: str = "activeset",
    w_pos: float = 1.0,
    w_ori: float = 0.05,  # per rad; 1 deg of angle error costs about as much as 0.9 mm of position error
    damping: float = 0.01,
    max_step: float = 0.5,  # rad, per joint per iteration
    tol_pos: float = 0.002,
    tol_ang: float = TOL_ANG,
    max_iter: int = 100,
    max_stall: int = 5,  # stop after this many iterations in a row without real progress
) -> IKResult:
    assert method in METHODS, method
    target_xyz = np.asarray(target_xyz, dtype=float)
    q = np.clip(np.asarray(q0, dtype=float), LO, HI)
    eye, zero = damping * np.eye(N), np.zeros(N)
    stall, prev_cost = 0, np.inf
    for it in range(max_iter + 1):
        pos, Jp = arm.pos_jac(q)
        pos_err = float(np.linalg.norm(target_xyz - pos))
        pe, re = wrap(pitch - pitch_of(q)), wrap(roll - roll_of(q))
        converged = pos_err < tol_pos and abs(pe) < tol_ang and abs(re) < tol_ang
        if converged or it == max_iter:
            break
        e = np.concatenate([w_pos * (target_xyz - pos), [w_ori * pe, w_ori * re]])
        cost = float(e @ e)
        stall = stall + 1 if cost > 0.99 * prev_cost else 0  # less than 1 % better than the last iteration
        if stall >= max_stall:
            break
        prev_cost = cost
        J = np.vstack([w_pos * Jp, w_ori * PITCH_ROW, w_ori * ROLL_ROW])
        if method == "clamp":
            dq = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(5), e)
            if (big := np.abs(dq).max()) > max_step:
                dq *= max_step / big
            q = np.clip(q + dq, LO, HI)
            continue
        lb, ub = np.maximum(LO - q, -max_step), np.minimum(HI - q, max_step)
        if method == "bvls":
            dq = lsq_linear(np.vstack([J, eye]), np.concatenate([e, zero]), bounds=(lb, ub), method="bvls").x
        else:
            dq = bounded_dls_step(J, e, damping, lb, ub)
        if np.linalg.norm(dq) < 1e-10:  # no admissible step improves the error: stalled (maybe at a limit)
            break
        q = q + dq
    at_limit = bool(np.any((q <= LO + 1e-9) | (q >= HI - 1e-9)))
    return IKResult(q, pos_err, pe, re, it, converged, at_limit)

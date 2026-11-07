"""Drawing plan shared by the simulation (S2) and the real arm (S3): shapes on a vertical plane, joint paths
by warm-started IK, and the commands that reach them.

The plane is x = X_PLANE in front of the base-frame origin. The tool is held horizontal (pitch 0, roll 0);
its yaw follows shoulder_pan, so it points away from the base, not perpendicular to the plane (5 joints
cannot hold the yaw as well).
The circle (and the square inside it) is the largest that keeps every joint at least 10 deg from its limit,
found by benchmarks/s2_drawing.py (results/s2_drawing.json).
"""

import time

import numpy as np

from so101_kinematics.ik import solve_ik
from so101_kinematics.models import HI, LO, N

X_PLANE = 0.20  # m in front of the base-frame origin (the shoulder_pan axis is 0.039 m ahead of it)
PITCH, ROLL = 0.0, 0.0  # tool horizontal, pointing away from the base (yaw = shoulder_pan, not controlled)
CENTER_Y, CENTER_Z, RADIUS = 0.0, 0.28, 0.036  # m; from results/s2_drawing.json (tests check they agree)
TOL_POS, TOL_ANG = 0.0002, np.radians(0.1)  # tight: the errors reported are tracking errors, not thresholds
STEP = 0.002  # m between path points
SPEED = 0.05  # m/s, assumed drawing speed for the implied joint speeds in the S2 report
RAMP_SPEED = np.radians(20.0)  # rad/s, SafeArm.move_to default
MAX_JOINT_STEP_DEG = 5.0  # between path points 2 mm apart; S2 measured at most 1.4

SEEDS = [
    np.zeros(N),
    np.array([0.0, 0.6, -1.2, 0.6, 0.0]),  # lift + elbow + wrist_flex = 0, i.e. pitch 0
    np.array([0.0, -0.6, 1.2, -0.6, 0.0]),
]


def solve_point(arm, y, z, q_prev, method="activeset"):
    """Solve one plane point, warm from q_prev; fall back to the fixed seeds when that does not converge."""
    target = np.array([X_PLANE, y, z])
    for k, q0 in enumerate([q_prev, *SEEDS] if q_prev is not None else SEEDS):
        res = solve_ik(arm, target, PITCH, ROLL, q0, method=method, tol_pos=TOL_POS, tol_ang=TOL_ANG)
        if res.converged:
            return res, k
    return res, -1


def make_shapes(yc, zc, r):
    t = np.append(np.arange(0, 2 * np.pi * r, STEP) / r, 2 * np.pi)  # closed: ends where it starts
    circle = np.stack([yc + r * np.cos(t), zc + r * np.sin(t)], 1)
    h = r / np.sqrt(2)  # the square sits inside the circle's footprint
    corners = np.array([[-h, -h], [h, -h], [h, h], [-h, h], [-h, -h]]) + [yc, zc]
    pts = []
    for a, b in zip(corners[:-1], corners[1:], strict=True):
        n = int(np.ceil(np.linalg.norm(b - a) / STEP))
        pts += [a + (b - a) * k / n for k in range(n)]
    pts.append(corners[-1])
    return {"circle": circle, "square": np.array(pts)}


def track(arm, path, method):
    q_prev, qs, errs, times, fallbacks, fails = None, [], [], [], 0, 0
    for y, z in path:
        t0 = time.perf_counter()
        res, k = solve_point(arm, y, z, q_prev, method)
        times.append(time.perf_counter() - t0)
        fails += not res.converged
        fallbacks += q_prev is not None and k > 0  # the warm start did not converge, a fixed seed did
        errs.append((res.pos_err, abs(res.pitch_err), abs(res.roll_err)))
        qs.append(res.q)
        q_prev = res.q
    qs, errs, times = np.array(qs), np.array(errs), np.array(times)
    dq = np.degrees(np.abs(np.diff(qs, axis=0)))
    ds = np.linalg.norm(np.diff(path, axis=0), axis=1)
    return qs, {
        "points": len(path),
        "not_converged": int(fails),
        "fallback_seeds_used": int(fallbacks),
        "max_pos_err_mm": float(errs[:, 0].max() * 1e3),
        "max_pitch_err_deg": float(np.degrees(errs[:, 1].max())),
        "max_roll_err_deg": float(np.degrees(errs[:, 2].max())),
        "max_joint_step_deg": float(dq.max()),
        "max_joint_speed_deg_s": float((dq / (ds[:, None] / SPEED)).max()),
        "min_margin_to_limit_deg": float(np.degrees(np.minimum(qs - LO, HI - qs).min())),
        "median_solve_ms": float(np.median(times) * 1e3),
        "max_solve_ms": float(times.max() * 1e3),
    }


def ramp(q_from, q_goal, safety, hz):
    """Joint-space ramp like SafeArm.move_to: constant speed, step-capped, goal clipped to the safe range."""
    step = min(RAMP_SPEED / hz, safety.cfg.max_step_rad)
    goal, _ = safety.clip(q_goal)
    q, out = q_from.copy(), []
    while not np.allclose(q, goal):
        q = q + np.clip(goal - q, -step, step)
        out.append(q.copy())
    return out


def resample(qs, path, speed, hz):
    """Joint path solved at the points of `path` -> one point per tick at the given tool speed (m/s).

    Uses the real arc length of `path` (point spacing is close to, not exactly, STEP) and ends on its last
    point, so a closed shape is drawn closed.
    """
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])
    ticks = np.append(np.arange(0.0, s[-1], speed / hz), s[-1])
    return [np.array([np.interp(t, s, qs[:, j]) for j in range(qs.shape[1])]) for t in ticks]


def shape_ticks(arm, shape: str, speed: float, hz: float, grip: float) -> np.ndarray:
    """Joint commands (n, 6) for drawing `shape` at `speed` (m/s), one per tick; gripper fixed at `grip`."""
    path = make_shapes(CENTER_Y, CENTER_Z, RADIUS)[shape]
    qs, m = track(arm, path, "activeset")
    # Every point must be a converged IK solution reached by warm start: a non-converged point is a wrong
    # target, and a fallback seed may land on another branch (a joint jump). The safety preflight cannot see
    # either, so refuse here.
    if m["not_converged"] or m["fallback_seeds_used"] or m["max_joint_step_deg"] > MAX_JOINT_STEP_DEG:
        raise ValueError(f"{shape}: the joint path is not safe to send: {m}")
    return np.array([np.append(q, grip) for q in resample(qs, path, speed, hz)])


def preflight(ticks: np.ndarray, safety) -> tuple[int, int]:
    """Run planned commands through the safety filter as SafeArm.command() would, starting at the first one.

    Returns (ticks clipped to the limits, ticks step-capped). Both should be 0, otherwise the safety layer
    would change the path.
    """
    q_from, clipped, capped = ticks[0], 0, 0
    for q in ticks:
        f = safety.filter(q, q_from)
        clipped += bool(f.clipped.any())
        capped += bool(f.step_limited.any())
        q_from = f.q
    return clipped, capped

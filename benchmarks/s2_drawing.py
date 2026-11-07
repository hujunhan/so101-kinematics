"""S2 drawing in simulation: a circle and a square on a vertical plane in front of the base.

1. Reachability map of the plane (x = X_PLANE in the base frame, tool pointing forward: pitch 0, roll 0),
   solved on a coarse grid from a few seeds. A cell is reachable if some seed converges (so it is a lower
   bound). The shapes are placed in the largest reachable circle, shrunk by a margin.
2. Each shape is discretised every 2 mm and solved point by point, warm-started from the previous point.
   Metrics: tracking error (position, pitch, roll), the largest joint step between points (a branch flip
   would show up as a jump), the implied joint speed at an assumed drawing speed, the closest approach to a
   joint limit, and solve time. The naive clamp solver runs the same path as a baseline.

Writes results/s2_drawing.md, results/s2_drawing.json, results/s2_workspace.png, results/s2_joints.png.
"""

import json
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import distance_transform_edt

from so101_kinematics.ik import solve_ik
from so101_kinematics.models import HI, LO, N, PinArm

X_PLANE = 0.20  # m in front of the base-frame origin (the shoulder_pan axis is 0.039 m ahead of it)
PITCH, ROLL = 0.0, 0.0  # tool pointing straight forward, perpendicular to the plane
Z_MIN, Z_MAX, Y_HALF = 0.03, 0.40, 0.22  # scan window (z_min keeps clear of the table)
GRID = 0.01  # m, reachability map resolution
MIN_MARGIN_DEG = 10.0  # shapes must stay at least this far from every joint limit (the real arm adds its own)
TOL_POS, TOL_ANG = 0.0002, np.radians(0.1)  # tight: the errors reported are tracking errors, not thresholds
STEP = 0.002  # m between path points
SPEED = 0.05  # m/s, assumed drawing speed for the implied joint speeds
OUT = Path(__file__).resolve().parent.parent / "results"

# Palette (reference instance, slots 1-2, light surface) and neutrals
SURFACE, INK, MUTED, GRIDC = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE = "#2a78d6", "#eb6834"

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


def reach_map(arm):
    ys = np.arange(-Y_HALF, Y_HALF + 1e-9, GRID)
    zs = np.arange(Z_MIN, Z_MAX + 1e-9, GRID)
    ok = np.zeros((len(zs), len(ys)), bool)
    for i, z in enumerate(zs):
        q_prev = None
        for j in range(len(ys)) if i % 2 == 0 else reversed(range(len(ys))):  # serpentine, so q_prev is close
            res, _ = solve_point(arm, ys[j], z, q_prev)
            ok[i, j] = res.converged
            q_prev = res.q if res.converged else None
    return ys, zs, ok


def best_center(ys, zs, ok):
    """Centre (on y = 0, in front of the base) and radius of the largest reachable circle."""
    dist = distance_transform_edt(np.pad(ok, 1)) * GRID  # pad: the scan window edge is not "reachable"
    j = int(np.argmin(np.abs(ys))) + 1  # +1: the padding
    i = int(np.argmax(dist[:, j]))
    return ys[j - 1], zs[i - 1], dist[i, j]


def make_shapes(yc, zc, r):
    t = np.arange(0, 2 * np.pi * r, STEP) / r
    circle = np.stack([yc + r * np.cos(t), zc + r * np.sin(t)], 1)
    h = r / np.sqrt(2)  # the square sits inside the circle's footprint
    corners = np.array([[-h, -h], [h, -h], [h, h], [-h, h], [-h, -h]]) + [yc, zc]
    pts = []
    for a, b in zip(corners[:-1], corners[1:], strict=True):
        n = int(np.ceil(np.linalg.norm(b - a) / STEP))
        pts += [a + (b - a) * k / n for k in range(n)]
    pts.append(corners[-1])
    return {"circle": circle, "square": np.array(pts)}


def fit_shapes(arm, yc, zc, r_max):
    """Largest shapes (shrinking from the reachable circle) that stay MIN_MARGIN_DEG away from every limit."""
    r = r_max
    while r > 0.01:
        shapes = make_shapes(yc, zc, r)
        runs = {k: track(arm, p, "activeset") for k, p in shapes.items()}
        if all(
            m["not_converged"] == 0 and m["min_margin_to_limit_deg"] >= MIN_MARGIN_DEG
            for _, m in runs.values()
        ):
            return shapes, r
        r -= 0.002
    raise RuntimeError("no shape size keeps the joint-limit margin")


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


def style(ax):
    ax.set_facecolor(SURFACE)
    for s in ax.spines.values():
        s.set_color(GRIDC)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(color=GRIDC, lw=0.6)
    ax.set_axisbelow(True)


def plot_workspace(ys, zs, ok, shapes, circle_info):
    fig, ax = plt.subplots(figsize=(6.4, 5.6), facecolor=SURFACE)
    style(ax)
    ext = [
        (ys[0] - GRID / 2) * 100,
        (ys[-1] + GRID / 2) * 100,
        (zs[0] - GRID / 2) * 100,
        (zs[-1] + GRID / 2) * 100,
    ]
    ax.imshow(
        ok,
        origin="lower",
        extent=ext,
        cmap=matplotlib.colors.ListedColormap([SURFACE, "#cfe0f5"]),
        aspect="equal",
    )
    for name, color in (("circle", BLUE), ("square", ORANGE)):
        p = shapes[name] * 100
        ax.plot(p[:, 0], p[:, 1], color=color, lw=2)
    yc, zc, r = circle_info
    ax.text(yc * 100, (zc + r * 1.05) * 100, "circle", color=INK, ha="center", va="bottom", fontsize=10)
    ax.text(yc * 100, zc * 100, "square", color=INK, ha="center", va="center", fontsize=10)
    ax.text(ext[0] + 1, ext[3] - 1.5, "reachable (shaded)", color=MUTED, fontsize=9, va="top")
    ax.set_xlabel("y, left-right (cm)", color=MUTED)
    ax.set_ylabel("z, height (cm)", color=MUTED)
    ax.set_title(
        f"Drawing plane x = {X_PLANE * 100:.0f} cm, tool pointing forward: reachable region",
        color=INK,
        fontsize=11,
        loc="left",
    )
    fig.tight_layout()
    fig.savefig(OUT / "s2_workspace.png", dpi=150, facecolor=SURFACE)
    plt.close(fig)


def plot_joints(trajs, names):
    fig, axes = plt.subplots(2, N, figsize=(12, 4.6), facecolor=SURFACE, sharex="row")
    for r, (shape, qs) in enumerate(trajs.items()):
        s = np.arange(len(qs)) * STEP * 100  # cm along the path
        for j in range(N):
            ax = axes[r, j]
            style(ax)
            ax.axhline(np.degrees(LO[j]), color=MUTED, lw=1, ls=(0, (4, 3)))
            ax.axhline(np.degrees(HI[j]), color=MUTED, lw=1, ls=(0, (4, 3)))
            ax.plot(s, np.degrees(qs[:, j]), color=BLUE if shape == "circle" else ORANGE, lw=1.8)
            ax.set_ylim(np.degrees(LO[j]) - 10, np.degrees(HI[j]) + 10)
            if r == 0:
                ax.set_title(names[j], color=INK, fontsize=9)
            if j == 0:
                ax.set_ylabel(f"{shape}\njoint angle (deg)", color=MUTED, fontsize=9)
            if r == 1:
                ax.set_xlabel("distance along path (cm)", color=MUTED, fontsize=8)
    fig.suptitle(
        "Joint angles along the drawing, dashed = joint limits", color=INK, fontsize=11, x=0.01, ha="left"
    )
    fig.tight_layout()
    fig.savefig(OUT / "s2_joints.png", dpi=150, facecolor=SURFACE)
    plt.close(fig)


def main() -> None:
    from so101_core.model import ARM_JOINTS

    OUT.mkdir(exist_ok=True)
    arm = PinArm()
    ys, zs, ok = reach_map(arm)
    yc, zc, r_max = best_center(ys, zs, ok)
    shapes, r = fit_shapes(arm, yc, zc, r_max)
    circle_info = (yc, zc, r)
    plot_workspace(ys, zs, ok, shapes, circle_info)

    results, trajs = {"plane": {"x_m": X_PLANE, "reachable_cells": int(ok.sum()), "cells": int(ok.size)}}, {}
    results["circle_center_radius_m"] = [float(v) for v in circle_info]
    results["largest_reachable_radius_m"] = float(r_max)
    for method in ("activeset", "clamp"):
        results[method] = {}
        for shape, path in shapes.items():
            qs, m = track(arm, path, method)
            results[method][shape] = m
            if method == "activeset":
                trajs[shape] = qs
    plot_joints(trajs, ARM_JOINTS)

    (OUT / "s2_drawing.json").write_text(json.dumps(results, indent=2))
    (OUT / "s2_drawing.md").write_text(render(results))
    print((OUT / "s2_drawing.md").read_text())


def render(res: dict) -> str:
    yc, zc, r = res["circle_center_radius_m"]
    p = res["plane"]
    lines = [
        "# S2 drawing in simulation",
        "",
        f"Plane x = {p['x_m'] * 100:.0f} cm in front of the base-frame origin, tool pointing forward (pitch 0, roll 0). "
        f"Reachable: {p['reachable_cells']}/{p['cells']} cells of {GRID * 100:.0f} cm (lower bound, a few seeds). "
        f"Circle: center y = {yc * 100:.1f} cm, z = {zc * 100:.1f} cm, radius {r * 100:.1f} cm "
        f"(the largest reachable circle has radius {res['largest_reachable_radius_m'] * 100:.1f} cm, "
        f"shrunk until every joint stays {MIN_MARGIN_DEG:.0f} deg from its limit); the square sits inside that circle. "
        f"Points every {STEP * 1e3:.0f} mm, warm-started from the previous point. Joint speeds assume {SPEED * 100:.0f} cm/s.",
        "",
        "![reachable region](s2_workspace.png)",
        "",
        "![joint angles](s2_joints.png)",
        "",
        "| Solver | Shape | Points | Not converged | Max pos. err | Max pitch / roll err | Max joint step | "
        "Max joint speed | Closest to a limit | Median / max solve |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for method, label in (("activeset", "ours (active set)"), ("clamp", "clamp baseline")):
        for shape, m in res[method].items():
            lines.append(
                f"| {label} | {shape} | {m['points']} | {m['not_converged']} | {m['max_pos_err_mm']:.2f} mm "
                f"| {m['max_pitch_err_deg']:.2f} / {m['max_roll_err_deg']:.2f} deg | {m['max_joint_step_deg']:.2f} deg "
                f"| {m['max_joint_speed_deg_s']:.0f} deg/s | {m['min_margin_to_limit_deg']:.1f} deg "
                f"| {m['median_solve_ms']:.2f} / {m['max_solve_ms']:.1f} ms |"
            )
    lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    main()

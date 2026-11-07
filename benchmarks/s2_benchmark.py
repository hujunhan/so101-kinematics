"""S2 benchmark (simulation only): joint-limited IK for position + approach angle (pitch, roll).

Targets are the pose (xyz, pitch, roll) of random in-limit configurations, so each one is reachable.
Success = position error < 2 mm and pitch / roll error < 0.5 deg. Three start conditions:
- cold:  start from the zero pose (worst case: no previous solution)
- far:   start from a configuration 0.3 rad (per joint, std) away from a solution (a large move)
- warm:  start from a configuration 0.05 rad away (a step along a trajectory)
Then a check of the compromise policy: position reachable, but the pitch forced to -90 deg (straight
down), which often cannot be reached.

Writes results/s2_benchmark.md and results/s2_benchmark.json.
"""

import json
import time
from pathlib import Path

import numpy as np

from so101_kinematics.ik import W_ORI, solve_ik
from so101_kinematics.models import HI, LO, N, PinArm
from so101_kinematics.references import PinkSolver, solve_ik_slsqp

SEED, N_TARGETS = 0, 200
OUT = Path(__file__).resolve().parent.parent / "results"


def main() -> None:
    arm, pink = PinArm(), PinkSolver(PinArm())
    solvers = {
        "ours: bounded DLS, active set": lambda T, p, r, q0: solve_ik(arm, T[:3, 3], p, r, q0),
        "ours: bounded DLS, scipy BVLS": lambda T, p, r, q0: solve_ik(arm, T[:3, 3], p, r, q0, method="bvls"),
        "baseline: DLS + clamp": lambda T, p, r, q0: solve_ik(arm, T[:3, 3], p, r, q0, method="clamp"),
        "reference: SLSQP + limits": lambda T, p, r, q0: solve_ik_slsqp(arm, T[:3, 3], p, r, q0),
        "reference: Pink (QP, 6-D pose)": lambda T, p, r, q0: pink.solve(T, p, r, q0),
    }
    from so101_kinematics.ik import pitch_of, roll_of

    rng = np.random.default_rng(SEED)
    sols = [rng.uniform(LO, HI) for _ in range(N_TARGETS)]
    starts = {
        "cold": [np.zeros(N) for _ in sols],
        "far": [np.clip(q + rng.normal(0, 0.3, N), LO, HI) for q in sols],
        "warm": [np.clip(q + rng.normal(0, 0.05, N), LO, HI) for q in sols],
    }
    targets = [(arm.fk(q), pitch_of(q), roll_of(q)) for q in sols]

    for f in solvers.values():  # warm-up (imports, first-call costs)
        T, p, r = targets[0]
        f(T, p, r, starts["cold"][0])

    results: dict = {}
    for cond, q0s in starts.items():
        results[cond] = {}
        for name, f in solvers.items():
            times, ok, in_lim, stuck, iters = [], 0, 0, 0, []
            for (T, p, r), q0 in zip(targets, q0s, strict=True):
                t0 = time.perf_counter()
                res = f(T, p, r, q0)
                times.append(time.perf_counter() - t0)
                ok += res.converged
                in_lim += bool(np.all((res.q >= LO - 1e-9) & (res.q <= HI + 1e-9)))
                stuck += (not res.converged) and res.at_limit
                iters.append(res.iterations)
            results[cond][name] = {
                "success": int(ok),
                "in_limits": int(in_lim),
                "failed_at_limit": int(stuck),
                "median_ms": float(np.median(times) * 1e3),
                "p95_ms": float(np.percentile(times, 95) * 1e3),
                "median_iters": float(np.median(iters)),
            }

    # Compromise policy: reachable position, pitch forced to straight down.
    down = {}
    for name in ("ours: bounded DLS, active set", "reference: SLSQP + limits"):
        n_ok, pos_errs, pitch_errs, in_lim = 0, [], [], 0
        for T, _, r in targets:
            res = solvers[name](T, -np.pi / 2, r, np.zeros(N))
            n_ok += res.converged
            in_lim += bool(np.all((res.q >= LO - 1e-9) & (res.q <= HI + 1e-9)))
            if not res.converged:
                pos_errs.append(res.pos_err * 1e3)
                pitch_errs.append(abs(np.degrees(res.pitch_err)))
        down[name] = {
            "reached": int(n_ok),
            "in_limits": int(in_lim),
            "compromise_cases": len(pos_errs),
            "median_pos_err_mm": float(np.median(pos_errs)) if pos_errs else 0.0,
            "median_pitch_err_deg": float(np.median(pitch_errs)) if pitch_errs else 0.0,
            "p95_pos_err_mm": float(np.percentile(pos_errs, 95)) if pos_errs else 0.0,
        }

    # Weight sweep on the same unreachable-pitch targets (ours only): how soft is "position first"?
    sweep = {}
    for w_ori in (0.05, 0.02, W_ORI, 0.005, 0.002):
        pos_errs, pitch_errs, n_ok = [], [], 0
        for T, _, r in targets:
            res = solve_ik(arm, T[:3, 3], -np.pi / 2, r, np.zeros(N), w_ori=w_ori)
            n_ok += res.converged
            pos_errs.append(res.pos_err * 1e3)
            pitch_errs.append(abs(np.degrees(res.pitch_err)))
        sweep[str(w_ori)] = {
            "reached": int(n_ok),
            "median_pos_err_mm": float(np.median(pos_errs)),
            "p95_pos_err_mm": float(np.percentile(pos_errs, 95)),
            "median_pitch_err_deg": float(np.median(pitch_errs)),
        }

    OUT.mkdir(exist_ok=True)
    (OUT / "s2_benchmark.json").write_text(
        json.dumps({"ik": results, "pitch_down": down, "sweep": sweep}, indent=2)
    )
    (OUT / "s2_benchmark.md").write_text(render(results, down, sweep))
    print((OUT / "s2_benchmark.md").read_text())


def render(results: dict, down: dict, sweep: dict) -> str:
    desc = {
        "cold": "start from the zero pose",
        "far": "start 0.3 rad (std per joint) from a solution",
        "warm": "start 0.05 rad (std per joint) from a solution",
    }
    lines = [
        "# S2 benchmark: position + approach angle IK (simulation)",
        "",
        f"{N_TARGETS} random reachable targets (pose of random in-limit configurations), seed {SEED}. Success: "
        f"position error < 2 mm and pitch / roll error < 0.5 deg. Weights: position 1 per m, angles {W_ORI} per rad. "
        "Single run, single thread; times are per solve including Python overhead.",
        "",
    ]
    for cond, rows in results.items():
        lines += [
            f"## {cond}: {desc[cond]}",
            "",
            "| Solver | Success | Within limits | Failed at a limit | Median | p95 | Median iters |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
        for name, r in rows.items():
            lines.append(
                f"| {name} | {r['success']}/{N_TARGETS} | {r['in_limits']}/{N_TARGETS} | {r['failed_at_limit']} "
                f"| {r['median_ms']:.2f} ms | {r['p95_ms']:.2f} ms | {r['median_iters']:.0f} |"
            )
        lines.append("")
    lines += [
        "## Unreachable pitch: position reachable, pitch forced to -90 deg (straight down), cold start",
        "",
        "When the full pose cannot be reached the solver returns a compromise (position weighted above angles). "
        "Error columns are over the cases that did not converge.",
        "",
        "| Solver | Reached | Within limits | Compromise cases | Median pos. err | p95 pos. err | Median pitch err |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for name, r in down.items():
        lines.append(
            f"| {name} | {r['reached']}/{N_TARGETS} | {r['in_limits']}/{N_TARGETS} | {r['compromise_cases']} "
            f"| {r['median_pos_err_mm']:.1f} mm | {r['p95_pos_err_mm']:.1f} mm | {r['median_pitch_err_deg']:.1f} deg |"
        )
    lines += [
        "",
        "### Orientation weight sweep (ours, same targets, errors over all 200 including the reached ones)",
        "",
        "| Angle weight (per rad) | Reached | Median pos. err | p95 pos. err | Median pitch err |",
        "|---:|---:|---:|---:|---:|",
    ]
    for w, r in sweep.items():
        lines.append(
            f"| {w} | {r['reached']}/{N_TARGETS} | {r['median_pos_err_mm']:.1f} mm "
            f"| {r['p95_pos_err_mm']:.1f} mm | {r['median_pitch_err_deg']:.1f} deg |"
        )
    lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    main()

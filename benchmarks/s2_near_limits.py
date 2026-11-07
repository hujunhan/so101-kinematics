"""S2 stress test (simulation only): targets whose solution sits close to the joint limits.

Each target is the pose of a configuration in which 1-3 random joints are 0.5-5 deg inside a random one of
their limits (the others uniform). The pose is reachable by construction, but a solver that overshoots a
limit step by step has little room. Same success rule as s2_benchmark.py (2 mm, 0.5 deg), three starts
(cold = zero pose, far = 0.3 rad, warm = 0.05 rad away), and a paired comparison against the clamp baseline
and SLSQP: on how many targets did only one of the two solvers succeed.

Writes results/s2_near_limits.md and results/s2_near_limits.json.
"""

import json
import time
from pathlib import Path

import numpy as np

from so101_kinematics.ik import pitch_of, roll_of, solve_ik
from so101_kinematics.models import HI, LO, N, PinArm
from so101_kinematics.references import PinkSolver, solve_ik_slsqp

SEED, N_TARGETS = 1, 500
OUT = Path(__file__).resolve().parent.parent / "results"


def near_limit_config(rng) -> np.ndarray:
    q = rng.uniform(LO, HI)
    for j in rng.choice(N, size=rng.integers(1, 4), replace=False):
        d = np.radians(rng.uniform(0.5, 5.0))
        q[j] = LO[j] + d if rng.random() < 0.5 else HI[j] - d
    return q


def main() -> None:
    arm = PinArm()
    pink = PinkSolver(arm)
    solvers = {
        "ours (active set)": lambda T, p, r, q0: solve_ik(arm, T[:3, 3], p, r, q0),
        "clamp baseline": lambda T, p, r, q0: solve_ik(arm, T[:3, 3], p, r, q0, method="clamp"),
        "SLSQP": lambda T, p, r, q0: solve_ik_slsqp(arm, T[:3, 3], p, r, q0),
        "Pink (6-D pose)": lambda T, p, r, q0: pink.solve(T, p, r, q0),
    }
    rng = np.random.default_rng(SEED)
    sols = [near_limit_config(rng) for _ in range(N_TARGETS)]
    targets = [(arm.fk(q), pitch_of(q), roll_of(q)) for q in sols]
    starts = {
        "cold": [np.zeros(N)] * N_TARGETS,
        "far": [np.clip(q + rng.normal(0, 0.3, N), LO, HI) for q in sols],
        "warm": [np.clip(q + rng.normal(0, 0.05, N), LO, HI) for q in sols],
    }
    for f in solvers.values():  # warm-up
        f(*targets[0], starts["cold"][0])

    results, success = {}, {}
    for cond, q0s in starts.items():
        results[cond] = {}
        for name, f in solvers.items():
            times, ok, in_lim, stuck = [], [], 0, 0
            for (T, p, r), q0 in zip(targets, q0s, strict=True):
                t0 = time.perf_counter()
                res = f(T, p, r, q0)
                times.append(time.perf_counter() - t0)
                ok.append(bool(res.converged))
                in_lim += bool(np.all((res.q >= LO - 1e-9) & (res.q <= HI + 1e-9)))
                stuck += (not res.converged) and res.at_limit
            success[cond, name] = np.array(ok)
            results[cond][name] = {
                "success": int(sum(ok)),
                "in_limits": int(in_lim),
                "failed_at_limit": int(stuck),
                "median_ms": float(np.median(times) * 1e3),
                "p95_ms": float(np.percentile(times, 95) * 1e3),
            }
    paired = {}
    for cond in starts:
        a = success[cond, "ours (active set)"]
        paired[cond] = {
            other: {
                "only_ours": int((a & ~success[cond, other]).sum()),
                "only_other": int((~a & success[cond, other]).sum()),
            }
            for other in ("clamp baseline", "SLSQP", "Pink (6-D pose)")
        }
    OUT.mkdir(exist_ok=True)
    (OUT / "s2_near_limits.json").write_text(json.dumps({"ik": results, "paired": paired}, indent=2))
    (OUT / "s2_near_limits.md").write_text(render(results, paired))
    print((OUT / "s2_near_limits.md").read_text())


def render(results: dict, paired: dict) -> str:
    lines = [
        "# S2 stress test: targets near the joint limits (simulation)",
        "",
        f"{N_TARGETS} targets, seed {SEED}: the pose of a configuration with 1-3 random joints 0.5-5 deg inside one of "
        "their limits (the others uniform), so each target is reachable. Success: position error < 2 mm and "
        "pitch / roll error < 0.5 deg. Starts: cold = zero pose, far = 0.3 rad (std per joint) from the solution, "
        "warm = 0.05 rad. Defaults of `solve_ik` (angle weight 0.01 per rad, damping 0.003).",
        "",
    ]
    for cond, rows in results.items():
        lines += [
            f"## {cond}",
            "",
            "| Solver | Success | Within limits | Failed at a limit | Median | p95 |",
            "|---|---:|---:|---:|---:|---:|",
        ]
        for name, r in rows.items():
            lines.append(
                f"| {name} | {r['success']}/{N_TARGETS} | {r['in_limits']}/{N_TARGETS} | {r['failed_at_limit']} "
                f"| {r['median_ms']:.2f} ms | {r['p95_ms']:.2f} ms |"
            )
        lines.append("")
    lines += [
        "## Paired comparison: targets where only one of the two solvers succeeded",
        "",
        "| Start | vs | Only ours | Only the other |",
        "|---|---|---:|---:|",
    ]
    for cond, d in paired.items():
        for other, c in d.items():
            lines.append(f"| {cond} | {other} | {c['only_ours']} | {c['only_other']} |")
    lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    main()

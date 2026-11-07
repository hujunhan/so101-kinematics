"""S3 analysis: commanded vs measured drawing on the real arm, from the logs in results/s3/.

The tool position is the FK of the joint readings, so this measures how well the servos follow the
commands (lag, gravity sag), not the absolute accuracy of the kinematic model (a 10 cm ruler check in S3
came out 8.7 cm by FK; see README). For each log:
- the error e = FK(measured) - FK(commanded), split into a constant offset (mostly gravity sag) and the
  rest; the rest is split along the direction of motion (lag) and across it;
- the servo delay: the shift (in ticks) that best lines the measured path up with the commanded one;
- the per-joint following error.

Writes results/s3_drawing.md, results/s3_drawing.json and one figure per run, results/s3_<run>.png.
"""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from so101_core.model import JOINTS

from so101_kinematics.models import PinArm

RESULTS = Path(__file__).resolve().parent.parent / "results"
SURFACE, INK, MUTED, GRIDC = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE = "#2a78d6", "#eb6834"


def analyze(kin: PinArm, log) -> tuple[dict, dict]:
    t, qc, qm = log["t"], log["q_cmd"], log["q_meas"]
    pc = np.array([kin.fk(q[:5])[:3, 3] for q in qc])
    pm = np.array([kin.fk(q[:5])[:3, 3] for q in qm])
    e = pm - pc
    offset = e.mean(axis=0)
    r = e - offset
    v = np.gradient(pc, axis=0)
    speed = np.linalg.norm(v, axis=1)
    moving = speed > 0.25 * np.median(speed)
    tan = np.where(moving[:, None], v / np.maximum(speed, 1e-12)[:, None], 0.0)
    along = np.sum(r * tan, axis=1)  # + ahead of the command, - behind it
    cross = np.linalg.norm(r - along[:, None] * tan, axis=1)
    # delay: compare the measured path with the commanded one shifted by k ticks
    costs = [np.mean(np.linalg.norm((pm[k:] - pc[: len(pc) - k]) - offset, axis=1)) for k in range(16)]
    delay = int(np.argmin(costs))
    hz = float(log["hz"])
    m = {
        "status": str(log["status"]),
        "speed_cm_s": float(log["speed_cm_s"]),
        "ticks": len(t),
        "duration_s": float(t[-1]),
        "max_tick_ms": float(np.diff(t).max() * 1e3),
        "err_median_mm": float(np.median(np.linalg.norm(e, axis=1)) * 1e3),
        "err_max_mm": float(np.linalg.norm(e, axis=1).max() * 1e3),
        "offset_mm": [float(x) for x in offset * 1e3],
        "shape_err_median_mm": float(np.median(np.linalg.norm(r, axis=1)) * 1e3),
        "shape_err_max_mm": float(np.linalg.norm(r, axis=1).max() * 1e3),
        "along_mean_mm": float(along[moving].mean() * 1e3),
        "cross_median_mm": float(np.median(cross[moving]) * 1e3),
        "delay_ms": delay / hz * 1e3,
        "after_delay_median_mm": float(costs[delay] * 1e3),
        "size_cmd_cm": [float(x) for x in np.ptp(pc[:, 1:], axis=0) * 100],
        "size_meas_cm": [float(x) for x in np.ptp(pm[:, 1:], axis=0) * 100],
        "joint_err_max_deg": dict(
            zip(JOINTS[:5], [float(x) for x in np.degrees(np.abs(qc - qm)[:, :5].max(axis=0))], strict=True)
        ),
        "sag_at_start_deg": dict(
            zip(JOINTS[:5], [float(x) for x in np.asarray(log["sag_deg"])[:5]], strict=True)
        ),
    }
    return m, {"t": t, "pc": pc, "pm": pm, "e": e}


def fit_joint_model(logs: list) -> dict:
    """Per joint, over all runs: error = offset - sign(v) * (deadband + delay * |v|), on moving ticks.

    offset: constant (gravity sag); deadband: lag that does not grow with speed; delay: lag that does.
    Part of the delay is the measurement itself: each tick reads the position right after sending the
    command, before the servo has reacted.
    """
    out = {}
    for j, name in enumerate(JOINTS[:4]):  # wrist_roll barely moves while drawing
        A, y = [], []
        for log in logs:
            qc, qm = log["q_cmd"][:, j], log["q_meas"][:, j]
            v = np.gradient(qc) * float(log["hz"])
            m = np.abs(v) > np.radians(0.5)
            s = np.sign(v[m])
            A.append(np.stack([np.ones(m.sum()), -s, -s * np.abs(v[m])], 1))
            y.append(qm[m] - qc[m])
        A, y = np.vstack(A), np.concatenate(y)
        x = np.linalg.lstsq(A, y, rcond=None)[0]
        out[name] = {
            "offset_deg": float(np.degrees(x[0])),
            "deadband_deg": float(np.degrees(x[1])),
            "delay_ms": float(x[2] * 1e3),
            "r2": float(1 - (y - A @ x).var() / y.var()),
        }
    return out


def style(ax):
    ax.set_facecolor(SURFACE)
    for s in ax.spines.values():
        s.set_color(GRIDC)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(color=GRIDC, lw=0.6)
    ax.set_axisbelow(True)


def plot(name: str, m: dict, d: dict) -> Path:
    fig, (a, b) = plt.subplots(
        1, 2, figsize=(11, 4.6), facecolor=SURFACE, gridspec_kw={"width_ratios": [1, 1.4]}
    )
    for ax in (a, b):
        style(ax)
    pc, pm = d["pc"] * 1000, d["pm"] * 1000
    a.plot(pc[:, 1], pc[:, 2], color=BLUE, lw=2, label="commanded")
    a.plot(pm[:, 1], pm[:, 2], color=ORANGE, lw=2, label="measured")
    a.set_aspect("equal", adjustable="datalim")
    a.set_xlabel("y, left-right (mm)", color=MUTED)
    a.set_ylabel("z, height (mm)", color=MUTED)
    a.legend(frameon=False, fontsize=9, labelcolor=INK)
    a.set_title("Tool path on the plane (FK of the joints)", color=INK, fontsize=10, loc="left")
    err = np.linalg.norm(d["e"], axis=1) * 1000
    b.plot(d["t"], err, color=BLUE, lw=1.8)
    b.set_xlabel("time (s)", color=MUTED)
    b.set_ylabel("|measured - commanded| (mm)", color=MUTED)
    b.set_ylim(0, max(10.0, err.max() * 1.1))
    b.set_title(
        f"Tracking error: median {m['err_median_mm']:.1f} mm, of which a constant offset "
        f"{np.linalg.norm(m['offset_mm']):.1f} mm",
        color=INK,
        fontsize=10,
        loc="left",
    )
    fig.suptitle(f"{name}: {m['status']}", color=INK, fontsize=11, x=0.01, ha="left")
    fig.tight_layout()
    path = RESULTS / f"s3_{name}.png"
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return path


def main() -> None:
    kin = PinArm()
    runs, logs = {}, []
    for f in sorted((RESULTS / "s3").glob("*.npz")):
        log = np.load(f)
        logs.append(log)
        m, d = analyze(kin, log)
        plot(f.stem, m, d)
        runs[f.stem] = m
    model = fit_joint_model(logs)
    (RESULTS / "s3_drawing.json").write_text(json.dumps({"runs": runs, "joint_model": model}, indent=2))
    (RESULTS / "s3_drawing.md").write_text(render(runs, model))
    print((RESULTS / "s3_drawing.md").read_text())


def render(runs: dict, model: dict) -> str:
    lines = [
        "# S3: drawing on the real arm",
        "",
        "Tool positions are the FK of the commanded and the measured joints (servo encoders), so these numbers "
        "are about how the servos follow the commands, not about the absolute accuracy of the model. "
        "Offset = mean of measured - commanded (x is out of the plane, z is up). Shape error = what is left "
        "after removing the offset. Along = mean of the remaining error along the direction of motion "
        "(negative: behind the command). Delay = the time shift that best lines up the two paths.",
        "",
        "| Run | Status | Duration | Error median / max | Offset x, y, z | Shape error median / max | Along | "
        "Delay | Size cmd -> meas (y x z) |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name, m in runs.items():
        o = m["offset_mm"]
        lines.append(
            f"| {name} | {m['status']} | {m['duration_s']:.1f} s | {m['err_median_mm']:.1f} / {m['err_max_mm']:.1f} mm "
            f"| {o[0]:+.1f}, {o[1]:+.1f}, {o[2]:+.1f} mm | {m['shape_err_median_mm']:.1f} / {m['shape_err_max_mm']:.1f} mm "
            f"| {m['along_mean_mm']:+.1f} mm | {m['delay_ms']:.0f} ms "
            f"| {m['size_cmd_cm'][0]:.2f} x {m['size_cmd_cm'][1]:.2f} -> {m['size_meas_cm'][0]:.2f} x "
            f"{m['size_meas_cm'][1]:.2f} cm |"
        )
    lines += ["", "Largest following error per joint (deg) and the sag measured at the start pose (deg):", ""]
    lines += ["| Run | " + " | ".join(JOINTS[:5]) + " |", "|---|" + "---:|" * 5]
    for name, m in runs.items():
        cells = [f"{m['joint_err_max_deg'][j]:.2f} (sag {m['sag_at_start_deg'][j]:+.2f})" for j in JOINTS[:5]]
        lines.append(f"| {name} | " + " | ".join(cells) + " |")
    lines += [
        "",
        f"Joint model fitted over all {len(runs)} runs (moving ticks): error = offset - sign(v) * (deadband + "
        "delay * |v|). The delay includes about one tick (33 ms) from reading right after each command.",
        "",
        "| Joint | Offset (sag) | Deadband | Delay | R2 |",
        "|---|---:|---:|---:|---:|",
    ]
    for name, f in model.items():
        lines.append(
            f"| {name} | {f['offset_deg']:+.2f} deg | {f['deadband_deg']:.2f} deg | {f['delay_ms']:.0f} ms | {f['r2']:.2f} |"
        )
    lines += [""] + [f"![{name}](s3_{name}.png)" for name in runs] + [""]
    return "\n".join(lines)


if __name__ == "__main__":
    main()

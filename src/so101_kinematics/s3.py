"""S3: draw the S2 shapes on the real arm. Every motion goes through so101-core's SafeArm.

    so101-draw readonly                 torque OFF: print the measured joints and the tool position; move the
                                        arm by hand
    so101-draw start  --shape square    torque ON: move_to the start of the shape, hold, measure the sag
    so101-draw draw   --shape square    torque ON: start, then follow the path; log commanded vs measured

The stages that move the arm print the plan and wait for you to type `yes` before the arm is powered and
again before it moves. Run with SO101_CONFIG_DIR pointing at so101-core's config (device ports).
"""

import argparse
import time
from collections.abc import Callable
from datetime import datetime
from pathlib import Path

import numpy as np
from so101_core.arm import SafeArm
from so101_core.config import DeviceConfig
from so101_core.model import JOINTS

from so101_kinematics.drawing import RAMP_SPEED, preflight, shape_ticks
from so101_kinematics.models import PinArm

LOGS = Path("results/s3")


def ask_yes(message: str) -> bool:
    return input(f"\n>>> {message} Type 'yes' to continue: ").strip().lower() == "yes"


def read_only(arm: SafeArm, kin: PinArm, seconds: float = 60.0, out: Callable[[str], None] = print) -> None:
    """Torque off: show the joints and the tool position (FK, base frame) while the arm is moved by hand."""
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < seconds:
        q = arm.read()
        tool = kin.fk(q[:5])[:3, 3] * 100
        out(f"joints (deg) {np.round(np.degrees(q), 1)}   tool (cm) {np.round(tool, 1)}")
        time.sleep(0.2)


def plan_shape(arm: SafeArm, kin: PinArm, shape: str, speed_cm_s: float):
    """Joint commands for the shape and the safety preflight; the gripper stays where it is allowed to."""
    grip = arm.safety.clip(arm.q_cmd)[0][5]
    ticks = shape_ticks(kin, shape, speed_cm_s / 100, arm.safety.cfg.rate_hz, grip)
    return ticks, *preflight(ticks, arm.safety)


def go_to_start(
    arm: SafeArm,
    kin: PinArm,
    q_start: np.ndarray,
    confirm: Callable[[str], bool],
    ramp_speed: float = RAMP_SPEED,
    hold_s: float = 2.0,
    out: Callable[[str], None] = print,
) -> dict | None:
    """move_to the start pose, hold, and measure the steady-state residual (gravity sag) there."""
    travel = np.degrees(np.abs(arm.safety.clip(q_start)[0] - arm.q_cmd)).max()
    out(
        f"start pose (deg): {np.round(np.degrees(q_start[:5]), 1)}; largest joint move {travel:.0f} deg, "
        f"about {travel / np.degrees(ramp_speed):.0f} s at {np.degrees(ramp_speed):.0f} deg/s"
    )
    if not confirm("The arm will MOVE to the start pose."):
        return None
    arm.move_to(q_start, speed_rad_s=ramp_speed)
    samples = []
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < hold_s:
        samples.append(arm.read())
        time.sleep(0.05)
    q_meas = np.mean(samples, axis=0)
    sag = np.degrees(arm.q_cmd - q_meas)  # commanded minus measured, per joint
    tool_err = (kin.fk(arm.q_cmd[:5])[:3, 3] - kin.fk(q_meas[:5])[:3, 3]) * 1000
    out(
        f"sag at the start pose (deg, command - measured): {dict(zip(JOINTS, np.round(sag, 2), strict=True))}"
    )
    out(f"tool offset from the commanded position (mm): {np.round(tool_err, 1)}")
    return {"sag_deg": sag, "tool_offset_mm": tool_err}


def draw(
    arm: SafeArm,
    kin: PinArm,
    shape: str,
    speed_cm_s: float,
    confirm: Callable[[str], bool],
    stall_deg: float = 5.0,
    stall_s: float = 0.3,
    ramp_speed: float = RAMP_SPEED,
    log_dir: Path = LOGS,
    out: Callable[[str], None] = print,
) -> Path | None:
    ticks, clipped, capped = plan_shape(arm, kin, shape, speed_cm_s)
    hz = arm.safety.cfg.rate_hz
    out(
        f"{shape}: {len(ticks)} commands at {hz:.0f} Hz = {len(ticks) / hz:.1f} s at {speed_cm_s:g} cm/s; "
        f"preflight: {clipped} clipped, {capped} step-capped"
    )
    if clipped or capped:
        raise RuntimeError("preflight: the safety layer would change this path; refusing to draw")
    start = go_to_start(arm, kin, ticks[0], confirm, ramp_speed, out=out)
    if start is None or not confirm(f"The arm will DRAW the {shape} at {speed_cm_s:g} cm/s."):
        return None
    # move_to ramped the command onto ticks[0] (to float tolerance); start the path from the exact command
    ticks = np.vstack([arm.q_cmd, ticks[1:]])
    t0, rows = time.perf_counter(), []  # logged every tick, so a run stopped by a stall keeps its data
    status = "complete"
    try:
        arm.follow(
            ticks,
            np.radians(stall_deg),
            stall_s,
            on_tick=lambda i, q, qc: rows.append((time.perf_counter() - t0, qc, q)),
        )
    except BaseException as e:  # StallError, Ctrl-C, bus errors: save what we have, then let SafeArm hold
        status = f"stopped: {type(e).__name__}: {e}"
        raise
    finally:
        path = _save_log(log_dir, shape, speed_cm_s, hz, rows, ticks, start, stall_deg, stall_s, status, out)
    return path


def _save_log(
    log_dir, shape, speed_cm_s, hz, rows, ticks, start, stall_deg, stall_s, status, out
) -> Path | None:
    if not rows:
        return None
    t = np.array([r[0] for r in rows])
    q_cmd, q_meas = np.array([r[1] for r in rows]), np.array([r[2] for r in rows])
    lag = np.degrees(np.abs(q_meas - q_cmd)).max(axis=0)
    worst = dict(zip(JOINTS, np.round(lag, 2), strict=True))
    out(f"{status}. largest following error per joint (deg): {worst}")
    log_dir.mkdir(parents=True, exist_ok=True)
    path = log_dir / f"{shape}_{speed_cm_s:g}cms_{datetime.now():%Y%m%d_%H%M%S}.npz"
    np.savez(
        path,
        t=t,
        q_cmd=q_cmd,
        q_meas=q_meas,
        planned=ticks,
        status=status,
        shape=shape,
        speed_cm_s=speed_cm_s,
        hz=hz,
        sag_deg=start["sag_deg"],
        tool_offset_mm=start["tool_offset_mm"],
        stall_deg=stall_deg,
        stall_s=stall_s,
    )
    out(f"log: {path}")
    return path


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=["readonly", "start", "draw"])
    ap.add_argument("--shape", choices=["circle", "square"], default="square")
    ap.add_argument("--speed", type=float, default=2.0, help="drawing speed, cm/s")
    ap.add_argument("--stall-deg", type=float, default=5.0)
    ap.add_argument("--stall-s", type=float, default=0.3)
    ap.add_argument("--seconds", type=float, default=60.0, help="readonly: how long to print")
    args = ap.parse_args()

    kin = PinArm()
    powered = args.stage != "readonly"
    if powered and not ask_yes(
        "The arm will be POWERED (torque on) and then moved. Is the area in front of it clear, and are you "
        "present with the power switch in reach?"
    ):
        return
    with SafeArm(DeviceConfig.load()).connect(torque=powered) as arm:
        if args.stage == "readonly":
            read_only(arm, kin, args.seconds)
        elif args.stage == "start":
            ticks, clipped, capped = plan_shape(arm, kin, args.shape, args.speed)
            print(f"preflight: {clipped} clipped, {capped} step-capped")
            go_to_start(arm, kin, ticks[0], ask_yes)
        else:
            draw(arm, kin, args.shape, args.speed, ask_yes, args.stall_deg, args.stall_s)


if __name__ == "__main__":
    main()

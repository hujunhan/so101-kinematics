"""Replay the planned drawing in the MuJoCo twin (simulation only, no hardware).

Plays what the real arm would be commanded in S3: a slow ramp from the recorded rest pose to the start of
the circle (like SafeArm.move_to), the circle, a ramp to the square, the square. Every tick goes through
SafetyLayer.filter, as SafeArm.command() does. The tool path is left behind as a trace.

    uv run python benchmarks/s2_replay.py [--speed 2] [--hz 30] [--headless]

The window stays open at the end; close it to exit. Needs results/s2_drawing.json (run s2_drawing.py).
Run from the repo root with SO101_CONFIG_DIR pointing at so101-core's config (for the rest pose).
"""

import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np
from so101_core.arm import Rate
from so101_core.config import JointMap
from so101_core.safety import SafetyLayer
from so101_core.twin import Twin

from so101_kinematics.drawing import make_shapes, ramp, resample, track
from so101_kinematics.models import PinArm

RESULTS = Path(__file__).resolve().parent.parent / "results"


def plan(speed, hz):
    arm, jm, safety = PinArm(), JointMap.load(), SafetyLayer()
    yc, zc, r = json.loads((RESULTS / "s2_drawing.json").read_text())["circle_center_radius_m"]
    # the gripper stays put, but the rest pose is outside the safe range (gripper -9 deg, margin starts at -5),
    # so move_to's goal clipping moves it inward by a few degrees on the first ramp
    grip = safety.clip(jm.rest)[0][5]
    paths = {}
    for name, path in make_shapes(yc, zc, r).items():
        qs, _ = track(arm, path, "activeset")
        paths[name] = [np.append(q, grip) for q in resample(qs, path, speed, hz)]
    ticks, labels, q = [], [], jm.rest.copy()
    for name in ("circle", "square"):
        for kind, seq in (
            ("ramp to the " + name, ramp(q, paths[name][0], safety, hz)),
            ("drawing the " + name, paths[name]),
        ):
            ticks += seq
            labels += [kind] * len(seq)
            q = ticks[-1]
    # what SafeArm.command() would send: every tick filtered against the previous command
    sent, q_from, clipped, capped = [], jm.rest.copy(), 0, 0
    for q in ticks:
        f = safety.filter(q, q_from)
        clipped += f.clipped.any()
        capped += f.step_limited.any()
        sent.append(f.q)
        q_from = f.q
    return jm.rest.copy(), sent, labels, clipped, capped


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=2.0, help="tool speed while drawing, cm/s")
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--headless", action="store_true", help="no window: plan and print the summary only")
    args = ap.parse_args()

    rest, sent, labels, clipped, capped = plan(args.speed / 100, args.hz)
    print(
        f"{len(sent)} ticks = {len(sent) / args.hz:.1f} s at {args.hz:.0f} Hz; "
        f"safety layer: {clipped} ticks clipped, {capped} ticks step-capped"
    )
    if args.headless:
        return

    twin = Twin().open_viewer()
    v = twin.viewer
    v.cam.lookat[:] = [0.2, 0.0, 0.22]
    v.cam.distance, v.cam.azimuth, v.cam.elevation = 0.95, 150, -12
    twin.show(rest)
    time.sleep(1.0)
    rate, last, trace = Rate(args.hz), None, 0
    for q, label in zip(sent, labels, strict=True):
        if label != last:
            print(label)
            last = label
        if not twin.show(q):
            return
        if label.startswith("drawing") and v.user_scn.ngeom < v.user_scn.maxgeom:
            tip = twin.data.site("gripperframe").xpos
            g = v.user_scn.geoms[v.user_scn.ngeom]
            mujoco.mjv_initGeom(
                g,
                mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([0.002, 0, 0]),
                tip,
                np.eye(3).flatten(),
                np.array([0.16, 0.47, 0.84, 1.0], dtype=np.float32),
            )
            v.user_scn.ngeom += 1
            trace += 1
        rate.sleep()
    print(f"done, {trace} trace points. Close the window to exit.")
    while twin.show(sent[-1]):
        time.sleep(0.05)


if __name__ == "__main__":
    main()

"""
End-to-end check for the 3D viewer.

Runs the whole chain off-screen with the simulator -- wire framing, decoding,
displacement solve, and scene update -- then verifies that the magnet actor
actually rendered onto the known circular path it was fed.

Needs a display; on a headless machine run it under xvfb:
    xvfb-run -a python test_viewer_e2e.py
"""

import math
import time

import numpy as np
import pyvista as pv

from motion_model import MotionSolver
from stream import SensorReader, SimulatedSerial
from tmag5273_viz import Viewer

Z0 = -6.75
GAIN = 20.0
AMPLITUDE_MM = SimulatedSerial.AMPLITUDE_MM
PERIOD_S = SimulatedSerial.PERIOD_S

failures = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}   {detail if not ok else ''}".rstrip())
    if not ok:
        failures.append(name)


def main():
    pv.OFF_SCREEN = True

    solver = MotionSolver(z0_mm=Z0, mode="linear")
    reader = SensorReader(None, solver, simulate=True)
    reader.start()

    viewer = Viewer(reader, Z0, GAIN, off_screen=True)

    print("\ngeometry")
    check("PCB built from the STEP file", viewer.board_source == "STEP",
          f"source was {viewer.board_source}")

    # Let the rest baseline capture complete.
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        viewer.update()
        state = reader.snapshot()
        if state and state["calibrated"]:
            break
        time.sleep(0.01)

    state = reader.snapshot()
    check("rest baseline captured", bool(state and state["calibrated"]))
    check("rest Bz near the simulated 44000 uT",
          abs(state["field"][2] - 44000) < 6000, f"got {state['field'][2]:.0f}")

    # Sample the rendered actor over more than a full revolution.
    print(f"\nsampling rendered magnet for {PERIOD_S + 1.5:.1f}s")
    samples = []
    deadline = time.monotonic() + PERIOD_S + 1.5
    while time.monotonic() < deadline:
        viewer.update()
        samples.append(tuple(viewer.magnet_actor.position))
        time.sleep(1.0 / 60.0)

    positions = np.array(samples)
    # Actor position is an offset; undo the visual exaggeration to get mm.
    xs = positions[:, 0] / GAIN
    ys = positions[:, 1] / GAIN
    zs = positions[:, 2] / GAIN

    print("\nmotion")
    check("magnet actually moved", np.ptp(xs) > 0.1, f"x span {np.ptp(xs):.4f} mm")

    radii = np.hypot(xs, ys)
    settled = radii[len(radii) // 3:]  # drop the smoothing ramp-in
    check("traces a circle of the right radius",
          abs(settled.mean() - AMPLITUDE_MM) < 0.05,
          f"mean radius {settled.mean():.4f} mm, expected {AMPLITUDE_MM}")
    check("radius stays constant (a circle, not a line)",
          np.ptp(settled) < 0.05, f"radius spread {np.ptp(settled):.4f} mm")

    check("x sweeps both directions", xs.min() < -0.1 and xs.max() > 0.1,
          f"x range [{xs.min():.3f}, {xs.max():.3f}]")
    check("y sweeps both directions", ys.min() < -0.1 and ys.max() > 0.1,
          f"y range [{ys.min():.3f}, {ys.max():.3f}]")
    check("z stays small (the sim only breathes 50 um)",
          np.abs(zs).max() < 0.15, f"max |z| {np.abs(zs).max():.4f} mm")

    # Phase: x leads y by a quarter turn on this path.
    quarter = int(round((PERIOD_S / 4) * 60))
    if len(xs) > 2 * quarter:
        tail = slice(len(xs) // 3, len(xs) - quarter)
        corr = np.corrcoef(xs[tail], ys[np.arange(tail.start, tail.stop) + quarter])[0, 1]
        check("x leads y by a quarter period (correct rotation sense)",
              corr > 0.8, f"correlation {corr:.3f}")

    print("\nsolver switch")
    reader.toggle_mode()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and solver.mode != "nonlinear":
        viewer.update()
        time.sleep(0.01)
    check("switched to nonlinear", solver.mode == "nonlinear")

    after = []
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        viewer.update()
        after.append(tuple(viewer.magnet_actor.position))
        time.sleep(1.0 / 60.0)
    arr = np.array(after)
    r_after = np.hypot(arr[:, 0], arr[:, 1]).mean() / GAIN
    check("still tracking the circle after the switch",
          abs(r_after - AMPLITUDE_MM) < 0.08, f"radius {r_after:.4f} mm")

    print("\nstream health")
    state = reader.snapshot()
    check("frames decoded", state["frames_decoded"] > 1000,
          f"{state['frames_decoded']} frames")
    check("no dropped frames", state["frames_dropped"] == 0,
          f"{state['frames_dropped']} dropped")
    check("no checksum errors", state["checksum_errors"] == 0,
          f"{state['checksum_errors']} bad")

    print("\nrendering")
    image = viewer.plotter.screenshot(return_img=True)
    check("frame rendered with content", image is not None and image.size > 0)
    # A totally black frame would mean nothing actually drew.
    check("frame is not blank", image.std() > 5.0, f"pixel std {image.std():.2f}")

    reader.running = False
    viewer.plotter.close()

    print()
    if failures:
        print(f"{len(failures)} FAILED: {', '.join(failures)}")
        raise SystemExit(1)
    print("end-to-end viewer check passed")


if __name__ == "__main__":
    main()

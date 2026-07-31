"""
Geometry tests for the PCB mesh.

Checks both sources - the tessellated STEP file and the procedural fallback -
against the dimensions measured out of HE_Sensor_fil.step, and confirms the
board is positioned with the sensor die at the origin.

Run with:  python test_geometry.py
"""

import sys

import numpy as np

from step_loader import BOARD_H, BOARD_T, BOARD_W, load_board

TOL = 0.05  # mm; tessellation of the filleted corners costs a little accuracy

failures = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}   {detail if not ok else ''}".rstrip())
    if not ok:
        failures.append(name)


class _BlockOCP:
    """Import hook that hides OCP, to exercise the procedural fallback."""

    def find_module(self, name, path=None):
        return self if name == "OCP" or name.startswith("OCP.") else None

    def load_module(self, name):
        raise ImportError(f"{name} hidden for this test")


def verify(mesh, source, label):
    print(f"\n{label} (source: {source})")
    bounds = mesh.bounds

    width = bounds[1] - bounds[0]
    height = bounds[3] - bounds[2]
    thickness = bounds[5] - bounds[4]

    check(f"width is {BOARD_W} mm", abs(width - BOARD_W) < TOL, f"got {width:.3f}")
    check(f"height is {BOARD_H} mm", abs(height - BOARD_H) < TOL, f"got {height:.3f}")
    check(f"thickness is {BOARD_T} mm", abs(thickness - BOARD_T) < TOL,
          f"got {thickness:.3f}")

    check("centered on x", abs(bounds[0] + bounds[1]) < TOL,
          f"x bounds {bounds[0]:.3f}..{bounds[1]:.3f}")
    check("centered on y", abs(bounds[2] + bounds[3]) < TOL,
          f"y bounds {bounds[2]:.3f}..{bounds[3]:.3f}")

    # The die is the origin, so the board must hang entirely below z=0.
    check("die face sits at z=0", abs(bounds[5]) < TOL, f"top at {bounds[5]:.3f}")
    check("board lies below the die", bounds[4] < 0 and bounds[5] <= TOL,
          f"z bounds {bounds[4]:.3f}..{bounds[5]:.3f}")

    check("mesh has geometry", mesh.n_points > 100 and mesh.n_cells > 100,
          f"{mesh.n_points} points, {mesh.n_cells} cells")

    # Four mounting holes should leave the material near each corner center empty.
    corners = [(10.25, 6.375), (-10.25, 6.375), (-10.25, -6.375), (10.25, -6.375)]
    points = np.asarray(mesh.points)
    missing = []
    for cx, cy in corners:
        near = np.hypot(points[:, 0] - cx, points[:, 1] - cy) < 1.6
        if not near.any():
            missing.append((cx, cy))
    check("four mounting holes present", not missing, f"no points around {missing}")


def main():
    print("PCB geometry tests")

    mesh, source = load_board()
    verify(mesh, source, "STEP path")
    if source != "STEP":
        print("  NOTE: cadquery-ocp is not installed, so the STEP path was "
              "not actually exercised.")

    # Re-import step_loader with OCP hidden so the fallback really runs.
    blocker = _BlockOCP()
    sys.meta_path.insert(0, blocker)
    try:
        for name in [n for n in sys.modules if n == "OCP" or n.startswith("OCP.")]:
            del sys.modules[name]
        mesh, source = load_board()
        verify(mesh, source, "procedural fallback")
        check("fallback really was procedural", source == "procedural", f"got {source}")
    finally:
        sys.meta_path.remove(blocker)

    print()
    if failures:
        print(f"{len(failures)} FAILED: {', '.join(failures)}")
        raise SystemExit(1)
    print("all tests passed")


if __name__ == "__main__":
    main()

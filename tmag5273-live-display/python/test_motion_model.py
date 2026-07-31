"""
Round-trip tests for the magnet displacement solvers.

Strategy: pick a known displacement, run it through the forward dipole model to
synthesize a field reading, then check that each solver recovers the
displacement it started from. No hardware required.

Run with:  python test_motion_model.py
"""

import math

from motion_model import (
    DEFAULT_Z0_MM,
    MotionSolver,
    dipole_field,
)

Z0 = DEFAULT_Z0_MM
# Arbitrary; every solver bootstraps k from the rest field, so its value only
# has to be self-consistent between the synthetic field and the solver.
K = 2.0e6

failures = []


def check(name, condition, detail=""):
    if condition:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}   {detail}")
        failures.append(name)


def make_solver(mode, displacement_free_rest=True):
    """Build a solver already calibrated at the rest point."""
    solver = MotionSolver(z0_mm=Z0, mode=mode, rest_samples=10)
    rest = dipole_field(0.0, 0.0, Z0, K)
    for _ in range(10):
        solver.observe(*rest)
    assert solver.calibrated
    return solver


def recover(solver, dx, dy, dz):
    """Synthesize the field for a displacement and solve it back."""
    field = dipole_field(dx, dy, Z0 + dz, K)
    return solver.solve(*field)


def test_calibration():
    print("\ncalibration")
    solver = make_solver("linear")
    check(
        "k recovered from rest field",
        math.isclose(solver.k, K, rel_tol=1e-9),
        f"got {solver.k}, want {K}",
    )
    check("no warning for a strong rest field", solver.warning is None)

    weak = MotionSolver(z0_mm=Z0, mode="linear", rest_samples=4)
    for _ in range(4):
        weak.observe(10.0, 20.0, 250.0)  # ambient-only, no magnet attached
    check(
        "warns when the rest field is too weak to be a magnet",
        weak.warning is not None,
        "expected a warning about the missing magnet",
    )


def test_zero_displacement():
    print("\nzero displacement")
    for mode in ("linear", "nonlinear"):
        solver = make_solver(mode)
        dx, dy, dz = recover(solver, 0.0, 0.0, 0.0)
        moved = max(abs(dx), abs(dy), abs(dz))
        check(f"{mode}: rest field yields no motion", moved < 1e-6, f"moved {moved:.2e} mm")


def test_small_displacement_roundtrip():
    print("\nsmall displacement round-trip (slip regime)")
    # Displacements in the range slip detection actually produces.
    cases = [
        (0.05, 0.0, 0.0),
        (0.0, 0.05, 0.0),
        (0.0, 0.0, 0.05),
        (0.04, -0.03, 0.02),
        (-0.06, 0.02, -0.01),
    ]

    for mode, tol in (("linear", 5e-3), ("nonlinear", 1e-6)):
        for dx, dy, dz in cases:
            solver = make_solver(mode)
            gx, gy, gz = recover(solver, dx, dy, dz)
            err = max(abs(gx - dx), abs(gy - dy), abs(gz - dz))
            check(
                f"{mode}: ({dx:+.2f},{dy:+.2f},{dz:+.2f}) mm within {tol} mm",
                err < tol,
                f"error {err:.2e} mm -> got ({gx:+.4f},{gy:+.4f},{gz:+.4f})",
            )


def test_linear_and_nonlinear_agree_when_small():
    print("\nsolvers agree in the small-signal regime")
    for dx, dy, dz in [(0.02, 0.01, 0.0), (0.0, -0.02, 0.01)]:
        lin = recover(make_solver("linear"), dx, dy, dz)
        non = recover(make_solver("nonlinear"), dx, dy, dz)
        spread = max(abs(lin[i] - non[i]) for i in range(3))
        check(
            f"({dx:+.2f},{dy:+.2f},{dz:+.2f}) mm agree within 10 um",
            spread < 0.01,
            f"spread {spread:.2e} mm",
        )


def test_nonlinear_wins_when_large():
    print("\nnonlinear stays exact where linear drifts")
    dx, dy, dz = 2.0, 1.5, 0.8  # well outside the linearization's comfort zone

    lin = recover(make_solver("linear"), dx, dy, dz)
    non = recover(make_solver("nonlinear"), dx, dy, dz)

    lin_err = max(abs(lin[0] - dx), abs(lin[1] - dy), abs(lin[2] - dz))
    non_err = max(abs(non[0] - dx), abs(non[1] - dy), abs(non[2] - dz))

    check(
        "nonlinear recovers a large displacement exactly",
        non_err < 1e-5,
        f"error {non_err:.2e} mm",
    )
    check(
        "linear is visibly worse at this amplitude",
        lin_err > non_err,
        f"linear error {lin_err:.2e} vs nonlinear {non_err:.2e}",
    )


def test_direction_signs():
    print("\nsign conventions")
    solver = make_solver("nonlinear")
    dx, _, _ = recover(solver, 0.1, 0.0, 0.0)
    check("+x displacement reads back as +x", dx > 0, f"got {dx:+.4f}")

    solver = make_solver("nonlinear")
    _, dy, _ = recover(solver, 0.0, 0.1, 0.0)
    check("+y displacement reads back as +y", dy > 0, f"got {dy:+.4f}")

    # +dz means the magnet moved away from the die (stack compressing pushes the
    # other way, but the solver should report the geometry it was handed).
    solver = make_solver("nonlinear")
    _, _, dz = recover(solver, 0.0, 0.0, 0.1)
    check("+z displacement reads back as +z", dz > 0, f"got {dz:+.4f}")


def test_uncalibrated_is_inert():
    print("\nuncalibrated behaviour")
    solver = MotionSolver(z0_mm=Z0, mode="linear", rest_samples=50)
    out = solver.observe(*dipole_field(0.5, 0.5, Z0, K))
    check("reports no motion before calibration completes", out == (0.0, 0.0, 0.0))
    check("still uncalibrated after one sample", not solver.calibrated)


def test_nonlinear_falls_back_gracefully():
    print("\nnonlinear robustness")
    solver = make_solver("nonlinear")
    # A field this wild has no plausible magnet position behind it.
    out = solver.solve(1e12, -1e12, 1e12)
    check(
        "absurd field still returns a finite answer via fallback",
        all(math.isfinite(v) for v in out),
        f"got {out}",
    )


def main():
    print("motion_model round-trip tests")
    test_calibration()
    test_zero_displacement()
    test_small_displacement_roundtrip()
    test_linear_and_nonlinear_agree_when_small()
    test_nonlinear_wins_when_large()
    test_direction_signs()
    test_uncalibrated_is_inert()
    test_nonlinear_falls_back_gracefully()

    print()
    if failures:
        print(f"{len(failures)} FAILED: {', '.join(failures)}")
        raise SystemExit(1)
    print("all tests passed")


if __name__ == "__main__":
    main()

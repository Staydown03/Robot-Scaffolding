"""
Magnet displacement solvers for the TMAG5273 slip sensor.

Geometry convention (all lengths in mm, all fields in microtesla):

    * The sensor die sits at the origin.
    * The magnet is an axially-magnetized disc whose center rests at (0, 0, Z0).
    * Z0 is SIGNED. It is negative when the magnet sits on the opposite face of
      the PCB from the die, which is how the prototype is built.

Dipole field with the moment along z (k absorbs mu0*m/4pi and the unit scaling):

    Bx = 3*k*x*z / r^5
    By = 3*k*y*z / r^5
    Bz = k*(3*z^2 - r^2) / r^5        with r = |(x, y, z)|

Two solvers recover magnet position from a field reading:

    "linear"    Closed-form linearization about the rest point. k cancels out,
                so it needs no magnet characterization at all. Accurate for the
                small displacements that slip detection actually involves.
    "nonlinear" Gauss-Newton on the full dipole model with an analytic Jacobian,
                seeded from the previous frame. Handles larger excursions. Its k
                bootstraps from the same rest capture, so it is also
                calibration-free. Falls back to "linear" if it fails to converge.
"""

import math

# Physical layout of the prototype stack, measured from the die outward:
#   1.75 mm PCB + 4.0 mm double-sided tape + 1.0 mm to the magnet's mid-plane.
# Negative because the stack is on the face of the PCB opposite the die.
DEFAULT_Z0_MM = -6.75

# Below this the magnet is almost certainly not attached and every solve would
# be dividing by noise.
MIN_PLAUSIBLE_REST_FIELD_UT = 1000.0

MODES = ("linear", "nonlinear")


def dipole_field(x, y, z, k):
    """Forward model: field in uT at the origin from a magnet centered at (x,y,z)."""
    r2 = x * x + y * y + z * z
    if r2 <= 0.0:
        raise ValueError("magnet cannot be coincident with the sensor")
    r = math.sqrt(r2)
    r5 = r2 * r2 * r
    return (
        3.0 * k * x * z / r5,
        3.0 * k * y * z / r5,
        k * (3.0 * z * z - r2) / r5,
    )


def _dipole_jacobian(x, y, z, k):
    """d(Bx,By,Bz)/d(x,y,z), returned row-major as a 3x3 nested list."""
    r2 = x * x + y * y + z * z
    r = math.sqrt(r2)
    r5 = r2 * r2 * r
    r7 = r5 * r2
    q = 2.0 * z * z - x * x - y * y  # equals 3z^2 - r^2

    return [
        [
            3.0 * k * z / r5 - 15.0 * k * x * x * z / r7,
            -15.0 * k * x * y * z / r7,
            3.0 * k * x / r5 - 15.0 * k * x * z * z / r7,
        ],
        [
            -15.0 * k * x * y * z / r7,
            3.0 * k * z / r5 - 15.0 * k * y * y * z / r7,
            3.0 * k * y / r5 - 15.0 * k * y * z * z / r7,
        ],
        [
            -2.0 * k * x / r5 - 5.0 * k * q * x / r7,
            -2.0 * k * y / r5 - 5.0 * k * q * y / r7,
            4.0 * k * z / r5 - 5.0 * k * q * z / r7,
        ],
    ]


def _solve3(a, b):
    """Solve a 3x3 system by Gaussian elimination with partial pivoting.

    Returns None when the matrix is too close to singular to trust.
    """
    m = [list(a[i]) + [b[i]] for i in range(3)]

    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-12:
            return None
        m[col], m[pivot] = m[pivot], m[col]

        for row in range(col + 1, 3):
            factor = m[row][col] / m[col][col]
            for c in range(col, 4):
                m[row][c] -= factor * m[col][c]

    out = [0.0, 0.0, 0.0]
    for row in (2, 1, 0):
        acc = m[row][3] - sum(m[row][c] * out[c] for c in range(row + 1, 3))
        out[row] = acc / m[row][row]
    return out


class MotionSolver:
    """Turns (Bx, By, Bz) readings into magnet displacement from rest.

    Feed samples to `observe()`; the first `rest_samples` of them are averaged
    into the rest baseline and the solver reports no displacement until that
    completes. `recalibrate()` restarts the capture.
    """

    def __init__(self, z0_mm=DEFAULT_Z0_MM, mode="linear", rest_samples=100):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, got {mode!r}")
        if z0_mm == 0:
            raise ValueError("z0_mm cannot be zero")

        self.z0 = float(z0_mm)
        self.mode = mode
        self.rest_samples = int(rest_samples)

        self.rest_field = None  # (Bx, By, Bz) once calibrated
        self.k = None
        self.warning = None

        self._accum = [0.0, 0.0, 0.0]
        self._count = 0
        # Seeds the nonlinear solve; tracks the previous solution for continuity.
        self._last_pos = (0.0, 0.0, self.z0)

    @property
    def calibrated(self):
        return self.rest_field is not None

    def recalibrate(self):
        """Discard the current baseline and capture a fresh one."""
        self.rest_field = None
        self.k = None
        self.warning = None
        self._accum = [0.0, 0.0, 0.0]
        self._count = 0
        self._last_pos = (0.0, 0.0, self.z0)

    def set_mode(self, mode):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, got {mode!r}")
        self.mode = mode

    def observe(self, bx, by, bz):
        """Consume one sample. Returns (dx, dy, dz) in mm relative to rest."""
        if not self.calibrated:
            self._accum[0] += bx
            self._accum[1] += by
            self._accum[2] += bz
            self._count += 1
            if self._count >= self.rest_samples:
                self._finish_calibration()
            return (0.0, 0.0, 0.0)

        return self.solve(bx, by, bz)

    def _finish_calibration(self):
        n = float(self._count)
        self.rest_field = (
            self._accum[0] / n,
            self._accum[1] / n,
            self._accum[2] / n,
        )
        bz_rest = self.rest_field[2]

        if abs(bz_rest) < MIN_PLAUSIBLE_REST_FIELD_UT:
            self.warning = (
                f"Rest Bz is only {bz_rest:.0f} uT. Expected tens of thousands "
                "with the magnet attached - check that the magnet stack is on "
                "the sensor. Displacements will be meaningless until it is."
            )

        # k from the on-axis rest relation Bz_rest = 2k / |z0|^3. This absorbs
        # magnet grade, true gap and dipole-model error into a single number.
        self.k = bz_rest * abs(self.z0) ** 3 / 2.0
        self._last_pos = (0.0, 0.0, self.z0)

    def solve(self, bx, by, bz):
        if self.mode == "nonlinear":
            result = self._solve_nonlinear(bx, by, bz)
            if result is not None:
                return result
        return self._solve_linear(bx, by, bz)

    def _solve_linear(self, bx, by, bz):
        """Closed-form linearization about the rest point.

        Off-diagonal Jacobian terms vanish by symmetry at (0, 0, z0), and the
        magnet strength cancels, leaving displacement as a function of field
        ratios and the standoff alone.
        """
        rest_x, rest_y, rest_z = self.rest_field
        bz_rest = rest_z
        if bz_rest == 0.0:
            return (0.0, 0.0, 0.0)

        dbx = bx - rest_x
        dby = by - rest_y
        dbz = bz - rest_z

        dx = (2.0 * self.z0 / 3.0) * (dbx / bz_rest)
        dy = (2.0 * self.z0 / 3.0) * (dby / bz_rest)
        dz = -(self.z0 / 3.0) * (dbz / bz_rest)

        self._last_pos = (dx, dy, self.z0 + dz)
        return (dx, dy, dz)

    def _solve_nonlinear(self, bx, by, bz, max_iter=8, tol=1e-4):
        """Gauss-Newton on the full dipole model, seeded from the last solution.

        Returns None (caller falls back to the linear solver) if the step is
        singular, diverges, or drives the magnet into the sensor.
        """
        if not self.k:
            return None

        target = (bx, by, bz)
        x, y, z = self._last_pos
        # A displacement this large means the model has lost the magnet.
        max_excursion = 4.0 * abs(self.z0)

        for _ in range(max_iter):
            try:
                model = dipole_field(x, y, z, self.k)
            except ValueError:
                return None

            residual = [model[i] - target[i] for i in range(3)]
            jac = _dipole_jacobian(x, y, z, self.k)
            step = _solve3(jac, [-r for r in residual])
            if step is None:
                return None

            # Cap each iteration so a bad Jacobian cannot fling the estimate.
            norm = math.sqrt(sum(s * s for s in step))
            limit = 0.5 * abs(self.z0)
            if norm > limit:
                scale = limit / norm
                step = [s * scale for s in step]
                norm = limit

            x += step[0]
            y += step[1]
            z += step[2]

            if abs(x) > max_excursion or abs(y) > max_excursion or abs(z) > max_excursion:
                return None
            if z == 0.0 or (z > 0) != (self.z0 > 0):
                # Crossed through the sensor plane; physically impossible here.
                return None

            if norm < tol:
                break
        else:
            return None  # never converged

        self._last_pos = (x, y, z)
        return (x, y, z - self.z0)

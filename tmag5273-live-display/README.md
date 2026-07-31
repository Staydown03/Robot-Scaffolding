# TMAG5273 Magnetic Slip Sensor

Tooling for a magnetic tactile slip sensor built around an Adafruit TMAG5273
3D Hall-effect magnetometer on an Arduino Uno Rev3.

A magnet is embedded in a compliant stack on the far face of the sensor PCB.
When the contact surface slips, the stack shears and the magnet translates
relative to the fixed sensor die — and that displacement is what the field
readings encode.

Two front-ends:

| | What it shows | Entry point |
|---|---|---|
| **3D viewer** | PCB + magnet in 3D, moving as the real magnet moves | `python/tmag5273_viz.py` |
| **2D bars** | Raw X/Y/Z field as bar meters | `python/tmag5273_display.py` |

```
TMAG5273  --I2C-->  Arduino Uno R3  --USB-->  Laptop (Python)
```

## The prototype stack

Measured from the PCB's non-component face outward:

| Layer | Thickness |
|-------|-----------|
| PCB | 1.75 mm |
| Double-sided tape ×2 | 4.0 mm |
| Tape with the ⌀8 × 2 mm magnet embedded | 2.0 mm |
| Silicone (bracelet) contact surface | 2.0 mm |

That puts the magnet's center **6.75 mm** from the die — the `z0` used
throughout, negative because the magnet is on the opposite face from the die.

## 1. Wire the sensor

| TMAG5273 | Arduino Uno |
|----------|-------------|
| VIN | 5V |
| GND | GND |
| SDA | **A4** (analog header) |
| SCL | **A5** (analog header) |

I2C only works on A4/A5. Digital pins 4 and 5 are ordinary GPIO and will leave
the sensor undetectable. If the sketch reports `ERR: TMAG5273 not found`, upload
`arduino/i2c_scanner/i2c_scanner.ino` — it reports every address that responds.

## 2. Flash the sketch

Install via **Tools → Manage Libraries**: `Adafruit TMAG5273`, `Adafruit BusIO`,
`Adafruit Unified Sensor`. Then select **Tools → Board → Arduino Uno** and your
**Port**, and upload one of:

- `arduino/tmag5273_binary_stream/` — high-rate binary stream, for the 3D viewer
- `arduino/tmag5273_serial_stream/` — human-readable CSV, for the 2D bar display

Close the IDE's Serial Monitor afterwards: only one program can hold the COM
port at a time.

Both sketches try all four factory I2C addresses, so they work with any of the
A1–A4 variants.

## 3. Run the viewer (Windows)

```
cd python
pip install -r requirements.txt
python tmag5273_viz.py
```

| Key | Action |
|-----|--------|
| `M` | switch motion model (linear ↔ nonlinear) |
| `R` | re-capture the rest baseline |
| `↑` / `↓` | more / less visual exaggeration |
| `Q` | quit |

Useful flags:

```
python tmag5273_viz.py --simulate        # no hardware; drives a known circular path
python tmag5273_viz.py --port COM3       # skip the port prompt
python tmag5273_viz.py --z0 -7.25        # trim the die-to-magnet standoff
python tmag5273_viz.py --gain 50         # visual exaggeration (default 20x)
```

**Real displacements are tens of microns**, far too small to see at true scale
on a 25 mm board, so the viewer exaggerates the motion for display. The exact
factor is shown in the HUD and the true micron values are always displayed
alongside. Start with `--simulate` to confirm everything works before trusting
live readings.

## How displacement is recovered

Modeling the disc as an axial dipole with the die at the origin:

```
Bx = 3kxz/r^5      By = 3kyz/r^5      Bz = k(3z^2 - r^2)/r^5
```

At rest `Bz_rest = 2k/|z0|^3`. Linearizing about that point, the off-diagonal
Jacobian terms vanish by symmetry and the magnet strength `k` cancels:

```
dx = (2*z0/3) * (dBx / Bz_rest)
dy = (2*z0/3) * (dBy / Bz_rest)
dz = -(z0/3)  * (dBz / Bz_rest)
```

Displacement comes from field *ratios* plus the standoff — no magnet grade or
dipole-moment calibration needed. At `z0 = -6.75 mm` that is ~100 µm of shear
per mT; with ~10 µT of post-averaging noise, resolution lands near 1 µm.

The nonlinear model (`M`) instead runs Gauss-Newton on the full dipole with an
analytic Jacobian, seeded from the previous frame. It handles large excursions
that the linearization distorts, and bootstraps `k` from the same rest capture,
so it needs no extra calibration either.

The rest baseline is the average of the first 100 samples after connecting, so
**hold the sensor still at startup**, or press `R` to recapture.

### Limitations worth knowing

- **3 DOF only.** One 3-axis point measurement recovers three unknowns. Magnet
  tilt and rotation are not observable and alias into apparent translation. The
  model is honest for small in-plane shear plus normal press, not general 6-DOF
  motion.
- **`z0` sets the scale.** The Hall elements sit inside the package, so the true
  standoff carries ~0.5–1 mm of uncertainty. Error in `z0` scales every
  displacement linearly — it is the knob to trim against reality (`--z0`).
- **The dipole model is marginal here.** Standoff/radius ≈ 1.7, so expect
  10–30% scale error. The ratio form absorbs much of it. Using the exact
  cylinder-magnet field is the upgrade path if fidelity matters.
- **Expect ~40–45 mT at rest** (~44,000 µT) once the magnet is attached. The
  library forces wide range, giving ±80 mT (x1 part) or ±266 mT (x2), so this
  fits. Readings of a few hundred µT mean the magnet is not attached — the
  viewer warns about this.

## Geometry

`cad/HE_Sensor_fil.step` is the board model. If `cadquery-ocp` is installed the
viewer tessellates that file directly, so a revised CAD export drops straight
in. Otherwise it rebuilds the board from the dimensions measured out of that
same file (25.5 × 17.75 × 1.75 mm, R2.5 corners, four ⌀2.5 mm holes), which for
this part is exact. The HUD shows which source is in use.

The sensor die is assumed to be at the board center on the +Z face of the STEP.
If the real die sits elsewhere, correct it in `step_loader.py`.

## Tests

None require hardware.

```
cd python
python test_motion_model.py     # solver round-trips against the forward model
python test_stream.py           # wire protocol: framing, checksums, resync, drops
python test_geometry.py         # board dimensions, both mesh sources
python test_viewer_e2e.py       # full chain; verifies the rendered magnet
                                # traces the simulated path
```

`test_viewer_e2e.py` needs a display — on a headless machine run it under
`xvfb-run -a python test_viewer_e2e.py`.

## Protocol

`tmag5273_binary_stream.ino` sends one ASCII header, then fixed 10-byte frames:

```
#TMAG5273 v=1 variant=x2 range_xy=266.0 range_z=266.0 addr=0x35

byte 0-1  0xAA 0x55   sync
byte 2    seq         uint8, wraps - lets the host detect dropped frames
byte 3-8  int16 LE    raw x, y, z counts
byte 9    checksum    XOR of bytes 2..8
```

Raw counts keep the stream lossless; the host converts with
`uT = (raw / 32768) * range_mT * 1000` using the ranges from the header.

Baud is **500000**, not 460800 — 500000 divides exactly from the Uno's 16 MHz
clock (UBRR=3, 0% error) while 460800's nearest divisor is 8.5% off, which
corrupts framing. 250000 is also exact if a USB serial driver objects.

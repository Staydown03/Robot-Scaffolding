"""
Live 3D magnet visualization for the TMAG5273 slip sensor.

Reads the binary stream from tmag5273_binary_stream.ino, solves the magnet's
displacement from the field readings, and renders the PCB and magnet with
PyVista (VTK), moving the simulated magnet the way the real one is moving.

Usage (Windows):
    pip install -r requirements.txt
    python tmag5273_viz.py

    python tmag5273_viz.py --simulate      # no Arduino needed; drives a known
                                           # circular path end to end
    python tmag5273_viz.py --port COM3     # skip the port prompt
    python tmag5273_viz.py --z0 -7.25      # trim the die-to-magnet standoff
    python tmag5273_viz.py --gain 50       # visual exaggeration factor

In the window:
    M            switch motion model (linear <-> nonlinear)
    R            re-capture the rest baseline
    Up / Down    more / less visual exaggeration
    Q            quit
"""

import argparse
import time

import numpy as np
import pyvista as pv

from motion_model import DEFAULT_Z0_MM, MotionSolver
from step_loader import BOARD_T, load_board, magnet_mesh
from stream import SensorReader, choose_serial_port

MAGNET_R = 4.0   # 8 mm diameter disc
MAGNET_T = 2.0

BG_TOP = "#131f33"
BG_BOTTOM = "#070c15"
PCB_COLOR = "#12a86a"
MAGNET_COLOR = "#e2e9f0"
GHOST_COLOR = "#4fd7e0"

TARGET_FPS = 60


class Viewer:
    def __init__(self, reader, z0, gain, step_path=None, off_screen=False):
        self.reader = reader
        self.z0 = z0
        self.gain = gain
        self.smoothing = 0.25
        self.shown = np.zeros(3)   # smoothed displacement, mm

        self.plotter = pv.Plotter(
            off_screen=off_screen, window_size=(1280, 800),
            title="TMAG5273 - Live Magnet Motion",
        )
        self.plotter.set_background(BG_BOTTOM, top=BG_TOP)

        board, self.board_source = (
            load_board(step_path) if step_path else load_board()
        )
        self.plotter.add_mesh(
            board, color=PCB_COLOR, opacity=0.45, smooth_shading=True,
            specular=0.3, specular_power=15, name="board",
        )
        # Edges keep the outline legible now that the board is translucent.
        self.plotter.add_mesh(
            board.extract_feature_edges(feature_angle=25),
            color="#5ef2b4", opacity=0.5, line_width=2, name="board_edges",
        )

        magnet = magnet_mesh(MAGNET_R, MAGNET_T, z0)
        self.magnet_actor = self.plotter.add_mesh(
            magnet, color=MAGNET_COLOR, smooth_shading=True,
            specular=0.6, specular_power=30, name="magnet",
        )

        # Rest-position reference: the two rim circles. A full wireframe
        # cylinder reads as noise rather than as a reference outline. (The rims
        # come through as boundary edges, since the caps are separate from the
        # side wall, so the extractor's defaults are what pick them up.)
        self.plotter.add_mesh(
            magnet_mesh(MAGNET_R, MAGNET_T, z0).extract_feature_edges(feature_angle=30),
            color=GHOST_COLOR, line_width=2, opacity=0.8, name="ghost",
        )

        self.link = pv.Line((0, 0, z0), (0, 0, z0))
        self.plotter.add_mesh(
            self.link, color=GHOST_COLOR, line_width=3, name="link"
        )

        self._add_axes()
        self.hud = self.plotter.add_text(
            "waiting for data...", position="upper_left",
            font_size=10, font="courier", color="#e8f1f8", name="hud",
        )
        self.plotter.add_text(
            "M model   R re-zero   Up/Down exaggeration   Q quit",
            position="lower_left", font_size=8, font="courier",
            color="#8fa6bd", name="help",
        )

        self.plotter.enable_lightkit()
        self.plotter.camera_position = [
            (44.0, -46.0, 22.0),   # eye
            (0.0, 0.0, z0 / 2),    # focus
            (0.0, 0.0, 1.0),       # up: z, matching the sensor frame
        ]

        self.running = True
        if not off_screen:
            self.plotter.add_key_event("m", self.reader.toggle_mode)
            self.plotter.add_key_event("r", self.reader.recalibrate)
            self.plotter.add_key_event("Up", lambda: self._nudge_gain(1.25))
            self.plotter.add_key_event("Down", lambda: self._nudge_gain(0.8))

    def _add_axes(self):
        """Sensor-frame axes drawn at the die."""
        for direction, color in (
            ((1, 0, 0), "#ff6a3d"),
            ((0, 1, 0), "#e3d34c"),
            ((0, 0, 1), "#4fd7e0"),
        ):
            self.plotter.add_mesh(
                pv.Arrow(start=(0, 0, 0), direction=direction, scale=6.5,
                         tip_length=0.22, tip_radius=0.06, shaft_radius=0.02),
                color=color, name=f"axis_{color}",
            )
        self.plotter.add_axes(interactive=False)

    def _nudge_gain(self, factor):
        self.gain = float(np.clip(self.gain * factor, 1.0, 500.0))

    def update(self):
        state = self.reader.snapshot()
        if state is None:
            return

        if state["z0"] != self.z0:
            self.z0 = state["z0"]

        target = np.asarray(state["displacement"], dtype=float)
        self.shown += self.smoothing * (target - self.shown)

        offset = self.shown * self.gain
        position = np.array([offset[0], offset[1], offset[2]])
        # VTK actor position is an offset from where the mesh was built, and
        # the magnet mesh was already built centered at z0.
        self.magnet_actor.position = position

        self.link.points = np.array([
            [0.0, 0.0, self.z0],
            [position[0], position[1], self.z0 + position[2]],
        ])

        self._update_hud(state)

    def _update_hud(self, state):
        bx, by, bz = state["field"]
        dx, dy, dz = state["displacement"]
        status = "live" if state["calibrated"] else "capturing rest baseline..."

        lines = [
            f"  status  {status}",
            f"   model  {state['mode']}",
            f"  source  {self.board_source} geometry",
            "",
            "  FIELD (uT)",
            f"       X  {bx:12,.0f}",
            f"       Y  {by:12,.0f}",
            f"       Z  {bz:12,.0f}",
            "",
            "  DISPLACEMENT (um)",
            f"       X  {dx * 1000:12,.1f}",
            f"       Y  {dy * 1000:12,.1f}",
            f"       Z  {dz * 1000:12,.1f}",
            "",
            f"    rate  {state['sample_rate']:,.0f} Hz",
            f"  frames  {state['frames_decoded']:,} "
            f"({state['frames_dropped']} lost, {state['checksum_errors']} bad)",
            f"  exagg.  x{self.gain:.0f}   z0 {self.z0:.2f} mm",
        ]
        if state["warning"]:
            lines += ["", "  !! " + state["warning"][:60]]

        self.hud.set_text("upper_left", "\n".join(lines))

    def run(self):
        self.plotter.show(interactive_update=True, auto_close=False)
        frame_time = 1.0 / TARGET_FPS
        try:
            while self.running:
                start = time.monotonic()
                self.update()
                self.plotter.update()
                if not self.plotter.render_window:
                    break
                elapsed = time.monotonic() - start
                if elapsed < frame_time:
                    time.sleep(frame_time - elapsed)
        except (KeyboardInterrupt, RuntimeError, AttributeError):
            pass
        finally:
            self.reader.running = False
            try:
                self.plotter.close()
            except Exception:
                pass

    def screenshot(self, path, settle_s=6.0):
        """Render one frame off-screen after letting the stream settle."""
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            self.update()
            time.sleep(1.0 / TARGET_FPS)
        self.plotter.screenshot(path)
        self.reader.running = False
        return path


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial port, e.g. COM3")
    parser.add_argument(
        "--simulate", action="store_true",
        help="synthesize a known circular magnet path instead of reading hardware",
    )
    parser.add_argument(
        "--z0", type=float, default=DEFAULT_Z0_MM,
        help=(
            "signed die-to-magnet-center standoff in mm "
            f"(default {DEFAULT_Z0_MM}; negative means the magnet is on the "
            "opposite face of the PCB from the die)"
        ),
    )
    parser.add_argument(
        "--mode", choices=("linear", "nonlinear"), default="linear",
        help="initial motion model (switch in the window with M)",
    )
    parser.add_argument(
        "--gain", type=float, default=20.0,
        help="visual exaggeration of the displacement (default 20x)",
    )
    parser.add_argument("--step", help="path to a STEP file for the PCB")
    parser.add_argument(
        "--screenshot", help="render one frame to this path and exit (headless)"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    solver = MotionSolver(z0_mm=args.z0, mode=args.mode)
    port = None if args.simulate else (args.port or choose_serial_port())

    reader = SensorReader(port, solver, simulate=args.simulate)
    reader.start()

    off_screen = bool(args.screenshot)
    viewer = Viewer(reader, args.z0, args.gain, args.step, off_screen=off_screen)
    print(f"PCB geometry: {viewer.board_source}")

    if off_screen:
        print(f"Wrote {viewer.screenshot(args.screenshot)}")
    else:
        viewer.run()


if __name__ == "__main__":
    main()

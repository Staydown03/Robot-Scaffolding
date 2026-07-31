"""
Serial transport for the TMAG5273 slip sensor.

Decodes the binary stream produced by tmag5273_binary_stream.ino, runs each
sample through the displacement solver, and publishes the latest state. Kept
independent of any visualization front-end.
"""

import math
import queue
import struct
import sys
import threading
import time

import serial
import serial.tools.list_ports

from motion_model import dipole_field

STREAM_BAUD = 500000  # must match STREAM_BAUD in the sketch
FRAME_SYNC = b"\xAA\x55"
FRAME_LEN = 10


# --------------------------------------------------------------------------
# Frame decoding
# --------------------------------------------------------------------------


class StreamDecoder:
    """Turns the raw byte stream into (x, y, z) microtesla readings.

    Consumes the ASCII header first, then resynchronizes on the 0xAA55 sync
    pattern, validating each frame's checksum. Free of I/O so it can be
    exercised directly in tests.
    """

    def __init__(self):
        self.buffer = bytearray()
        self.header = None
        self.range_xy = None
        self.range_z = None
        self.frames_decoded = 0
        self.frames_dropped = 0
        self.checksum_errors = 0
        self.error_line = None
        self._last_seq = None

    @property
    def ready(self):
        return self.header is not None

    def feed(self, data):
        """Add bytes; yields (bx, by, bz) in microtesla for each valid frame."""
        self.buffer.extend(data)

        if not self.ready:
            self._consume_header()
            if not self.ready:
                return

        yield from self._consume_frames()

    def _consume_header(self):
        while True:
            newline = self.buffer.find(b"\n")
            if newline < 0:
                # Don't let junk accumulate forever if the header never arrives.
                if len(self.buffer) > 4096:
                    del self.buffer[:-1024]
                return

            line = bytes(self.buffer[:newline]).strip()
            del self.buffer[: newline + 1]

            if line.startswith(b"#TMAG5273"):
                self._parse_header(line.decode("ascii", errors="replace"))
                return
            if line.startswith(b"ERR"):
                self.error_line = line.decode("ascii", errors="replace")
                return

    def _parse_header(self, line):
        fields = {}
        for token in line.split()[1:]:
            if "=" in token:
                key, _, value = token.partition("=")
                fields[key] = value

        self.header = fields
        try:
            self.range_xy = float(fields["range_xy"])
            self.range_z = float(fields["range_z"])
        except (KeyError, ValueError):
            # Fall back to the widest x2 range rather than refusing to run.
            self.range_xy = self.range_z = 266.0

    def _consume_frames(self):
        while True:
            start = self.buffer.find(FRAME_SYNC)
            if start < 0:
                # Keep one byte in case a sync pattern straddles the boundary.
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                return

            if start:
                del self.buffer[:start]

            if len(self.buffer) < FRAME_LEN:
                return

            frame = bytes(self.buffer[:FRAME_LEN])
            checksum = 0
            for byte in frame[2:9]:
                checksum ^= byte

            if checksum != frame[9]:
                # False sync or corrupted frame: step past it and rescan.
                self.checksum_errors += 1
                del self.buffer[:2]
                continue

            del self.buffer[:FRAME_LEN]

            seq = frame[2]
            if self._last_seq is not None:
                gap = (seq - self._last_seq - 1) & 0xFF
                if gap:
                    self.frames_dropped += gap
            self._last_seq = seq

            raw_x, raw_y, raw_z = struct.unpack("<3h", frame[3:9])
            self.frames_decoded += 1

            yield (
                raw_x / 32768.0 * self.range_xy * 1000.0,
                raw_y / 32768.0 * self.range_xy * 1000.0,
                raw_z / 32768.0 * self.range_z * 1000.0,
            )


def encode_frame(seq, raw_x, raw_y, raw_z):
    """Build one wire frame. Used by the simulator and by the tests."""
    body = struct.pack("<B3h", seq & 0xFF, raw_x, raw_y, raw_z)
    checksum = 0
    for byte in body:
        checksum ^= byte
    return FRAME_SYNC + body + bytes([checksum])


# --------------------------------------------------------------------------
# Sources
# --------------------------------------------------------------------------


def choose_serial_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found. Plug in the Arduino Uno, or use --simulate.")
        sys.exit(1)
    if len(ports) == 1:
        return ports[0].device

    print("Multiple serial ports found:")
    for i, port in enumerate(ports):
        print(f"  [{i}] {port.device} - {port.description}")
    choice = input(f"Select a port [0-{len(ports) - 1}]: ").strip()
    return ports[int(choice)].device


class SimulatedSerial:
    """Stand-in for a serial port that emits a magnet on a known circular path.

    Produces real wire frames through the real encoder, so --simulate exercises
    the whole chain (framing -> decode -> solver -> render) rather than
    injecting displacements directly.
    """

    AMPLITUDE_MM = 0.30
    PERIOD_S = 4.0
    REST_BZ_UT = 44000.0  # roughly what an 8x2mm disc gives at this standoff

    def __init__(self, z0_mm, rate_hz=400.0):
        self.z0 = z0_mm
        self.rate = rate_hz
        self.range_xy = self.range_z = 266.0
        self.k = self.REST_BZ_UT * abs(z0_mm) ** 3 / 2.0
        self._seq = 0
        self._started = time.monotonic()
        self._next_sample = self._started
        self._header_sent = False

    def read(self, _size=1):
        if not self._header_sent:
            self._header_sent = True
            return (
                b"#TMAG5273 v=1 variant=x2 range_xy=266.0 "
                b"range_z=266.0 addr=0x35\n"
            )

        now = time.monotonic()
        if self._next_sample > now:
            time.sleep(self._next_sample - now)
        self._next_sample += 1.0 / self.rate

        elapsed = time.monotonic() - self._started
        # Hold still briefly so the solver can capture a clean rest baseline.
        if elapsed < 1.0:
            dx = dy = dz = 0.0
        else:
            phase = 2.0 * math.pi * (elapsed - 1.0) / self.PERIOD_S
            dx = self.AMPLITUDE_MM * math.cos(phase)
            dy = self.AMPLITUDE_MM * math.sin(phase)
            dz = 0.05 * math.sin(2.0 * phase)

        bx, by, bz = dipole_field(dx, dy, self.z0 + dz, self.k)
        return encode_frame(
            self._seq_next(),
            self._to_counts(bx, self.range_xy),
            self._to_counts(by, self.range_xy),
            self._to_counts(bz, self.range_z),
        )

    def _seq_next(self):
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    @staticmethod
    def _to_counts(microtesla, range_mt):
        counts = int(round(microtesla / (range_mt * 1000.0) * 32768.0))
        return max(-32768, min(32767, counts))

    def close(self):
        pass


# --------------------------------------------------------------------------
# Reader thread
# --------------------------------------------------------------------------


class SensorReader(threading.Thread):
    """Owns the serial port and the solver; publishes the latest solved state."""

    def __init__(self, port, solver, simulate=False):
        super().__init__(daemon=True)
        self.port_name = port
        self.solver = solver
        self.simulate = simulate

        self.decoder = StreamDecoder()
        self.latest = None
        self.lock = threading.Lock()
        self.commands = queue.Queue()
        self.running = True

        self._serial = None
        self._rate_window_start = time.monotonic()
        self._rate_window_count = 0
        self._sample_rate = 0.0

    def _open(self):
        if self.simulate:
            return SimulatedSerial(self.solver.z0)
        return serial.Serial(self.port_name, STREAM_BAUD, timeout=1)

    def run(self):
        self._serial = self._open()
        if self.simulate:
            print("Simulating a magnet on a 0.30 mm circular path (no hardware).")
        else:
            print(f"Serial port {self.port_name} opened at {STREAM_BAUD} baud.")

        announced_header = False
        warned = False

        while self.running:
            self._drain_commands()

            try:
                chunk = self._serial.read(4096 if not self.simulate else 1)
            except serial.SerialException as exc:
                print(f"Serial connection dropped ({exc}). Reconnecting...")
                self._reconnect()
                continue

            if not chunk:
                continue

            for bx, by, bz in self.decoder.feed(chunk):
                if not announced_header and self.decoder.header:
                    announced_header = True
                    print(f"Sensor: {self.decoder.header}")

                dx, dy, dz = self.solver.observe(bx, by, bz)

                if self.solver.warning and not warned:
                    warned = True
                    print(f"WARNING: {self.solver.warning}")

                self._tick_rate()
                with self.lock:
                    self.latest = {
                        "field": (bx, by, bz),
                        "displacement": (dx, dy, dz),
                        "calibrated": self.solver.calibrated,
                        "mode": self.solver.mode,
                        "warning": self.solver.warning,
                        "z0": self.solver.z0,
                        "sample_rate": self._sample_rate,
                        "frames_decoded": self.decoder.frames_decoded,
                        "frames_dropped": self.decoder.frames_dropped,
                        "checksum_errors": self.decoder.checksum_errors,
                    }

            if self.decoder.error_line:
                print(f"Arduino reported: {self.decoder.error_line}")
                self.decoder.error_line = None

    def _tick_rate(self):
        self._rate_window_count += 1
        now = time.monotonic()
        elapsed = now - self._rate_window_start
        if elapsed >= 0.5:
            self._sample_rate = self._rate_window_count / elapsed
            self._rate_window_count = 0
            self._rate_window_start = now

    def _drain_commands(self):
        while True:
            try:
                command = self.commands.get_nowait()
            except queue.Empty:
                return

            action = command.get("cmd")
            if action == "mode":
                value = command.get("value")
                if value in ("linear", "nonlinear"):
                    self.solver.set_mode(value)
                    print(f"Motion model -> {value}")
            elif action == "recalibrate":
                self.solver.recalibrate()
                print("Re-capturing rest baseline; hold the sensor still.")

    def _reconnect(self):
        try:
            self._serial.close()
        except Exception:
            pass
        self.decoder = StreamDecoder()
        while self.running:
            try:
                self._serial = self._open()
                print(f"Reconnected to {self.port_name}.")
                return
            except serial.SerialException:
                time.sleep(1.0)

    def snapshot(self):
        with self.lock:
            return dict(self.latest) if self.latest else None

    def toggle_mode(self):
        nxt = "nonlinear" if self.solver.mode == "linear" else "linear"
        self.commands.put({"cmd": "mode", "value": nxt})

    def recalibrate(self):
        self.commands.put({"cmd": "recalibrate"})

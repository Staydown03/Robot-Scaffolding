"""
TMAG5273 live bar-graph display for Windows.

Reads "X,Y,Z" magnetic field lines (microtesla) streamed by the
tmag5273_serial_stream.ino sketch over USB serial from an Arduino Uno,
and draws a live bar-meter display similar to the Adafruit TFT demo.

Setup (Windows):
    pip install -r requirements.txt
    python tmag5273_display.py

If more than one serial port is found, you'll be prompted to pick one
(check Windows Device Manager for "Arduino Uno (COMx)" if unsure).
"""

import sys
import threading
import queue
import time
import tkinter as tk

import serial
import serial.tools.list_ports

BAUD_RATE = 115200
FULL_SCALE_UT = 133000  # +/- range shown by the bars; matches TMAG5273 x2 variant (133 mT)

BG_COLOR = "#0d1b2a"
PANEL_COLOR = "#122236"
TEXT_COLOR = "#e8f1f8"
AXES = [
    ("X", "#ff6a3d"),
    ("Y", "#d9d94a"),
    ("Z", "#4fd7e0"),
]


def choose_serial_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found. Plug in the Arduino Uno and try again.")
        sys.exit(1)
    if len(ports) == 1:
        return ports[0].device

    print("Multiple serial ports found:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} - {p.description}")
    choice = input(f"Select a port [0-{len(ports) - 1}]: ").strip()
    return ports[int(choice)].device


class SerialReader(threading.Thread):
    def __init__(self, port, data_queue):
        super().__init__(daemon=True)
        self.data_queue = data_queue
        self.ser = serial.Serial(port, BAUD_RATE, timeout=1)
        print(f"Serial port {port} opened. Waiting for data from the Arduino...")

    def run(self):
        last_good = time.monotonic()
        lines_seen = 0
        while True:
            raw = self.ser.readline()
            if not raw:
                if time.monotonic() - last_good > 3:
                    print(
                        "No data received in the last 3s. Check: sketch is uploaded "
                        "and running, Serial Monitor/Plotter is closed in the Arduino "
                        "IDE, and wiring (SDA/SCL/VIN/GND) is correct."
                    )
                    last_good = time.monotonic()
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            if line.startswith("ERR"):
                print(f"Arduino reported: {line}")
                continue

            parts = line.split(",")
            if len(parts) != 3:
                print(f"Ignoring unexpected line: {line!r}")
                continue
            try:
                x, y, z = (float(p) for p in parts)
            except ValueError:
                print(f"Ignoring unparsable line: {line!r}")
                continue

            lines_seen += 1
            if lines_seen <= 3:
                print(f"Received: X={x} Y={y} Z={z}")
            last_good = time.monotonic()
            self.data_queue.put((x, y, z))


class BarDisplay:
    def __init__(self, root, data_queue):
        self.data_queue = data_queue
        self.root = root
        root.title("TMAG5273 Live Field Display")
        root.configure(bg=BG_COLOR)

        self.width, self.height = 480, 420
        self.canvas = tk.Canvas(
            root, width=self.width, height=self.height, bg=BG_COLOR, highlightthickness=0
        )
        self.canvas.pack(padx=10, pady=10)

        self.bar_width = 110
        self.bar_gap = 30
        self.bar_top = 60
        self.bar_bottom = self.height - 50
        self.mid_y = (self.bar_top + self.bar_bottom) // 2

        self.value_texts = []
        self.bars = []
        start_x = 40
        for i, (label, color) in enumerate(AXES):
            x0 = start_x + i * (self.bar_width + self.bar_gap)
            x1 = x0 + self.bar_width
            self.canvas.create_rectangle(
                x0, self.bar_top, x1, self.bar_bottom, outline="#3a5068", width=2
            )
            self.canvas.create_line(
                x0, self.mid_y, x1, self.mid_y, fill="#3a5068", dash=(3, 2)
            )
            bar = self.canvas.create_rectangle(
                x0, self.mid_y, x1, self.mid_y, fill=color, outline=""
            )
            value_text = self.canvas.create_text(
                (x0 + x1) / 2,
                self.bar_top - 25,
                text="0",
                fill=TEXT_COLOR,
                font=("Consolas", 16, "bold"),
            )
            self.canvas.create_text(
                (x0 + x1) / 2,
                self.bar_bottom + 22,
                text=label,
                fill=color,
                font=("Consolas", 18, "bold"),
            )
            self.bars.append((bar, x0, x1))
            self.value_texts.append(value_text)

        self.poll()

    def set_bar(self, index, value):
        bar, x0, x1 = self.bars[index]
        clamped = max(-FULL_SCALE_UT, min(FULL_SCALE_UT, value))
        span = (self.mid_y - self.bar_top)
        offset = (clamped / FULL_SCALE_UT) * span
        y = self.mid_y - offset
        top, bottom = sorted((self.mid_y, y))
        self.canvas.coords(bar, x0, top, x1, bottom)
        self.canvas.itemconfigure(self.value_texts[index], text=f"{value:.0f}")

    def poll(self):
        latest = None
        try:
            while True:
                latest = self.data_queue.get_nowait()
        except queue.Empty:
            pass

        if latest is not None:
            for i, value in enumerate(latest):
                self.set_bar(i, value)

        self.root.after(30, self.poll)


def main():
    port = choose_serial_port()
    print(f"Connecting to {port} @ {BAUD_RATE} baud...")

    data_queue = queue.Queue()
    reader = SerialReader(port, data_queue)
    reader.start()

    print("Opening display window - check your taskbar/Alt-Tab if you don't see it pop up.")
    root = tk.Tk()
    root.lift()
    root.attributes("-topmost", True)
    root.after(500, lambda: root.attributes("-topmost", False))
    BarDisplay(root, data_queue)
    root.mainloop()


if __name__ == "__main__":
    main()

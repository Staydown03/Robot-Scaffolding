# TMAG5273 Live Field Display

Live bar-graph display of X/Y/Z magnetic field readings from an Adafruit
TMAG5273 3D Hall Effect Magnetometer, read over an Arduino Uno Rev3 and
plotted in a small window on your Windows laptop (similar to the on-board
TFT demo Adafruit ships on their Feather boards).

```
TMAG5273  --I2C-->  Arduino Uno R3  --USB-->  Laptop (Python display)
```

## 1. Wire the sensor to the Uno

| TMAG5273 | Arduino Uno |
|----------|-------------|
| VIN      | 5V          |
| GND      | GND         |
| SDA      | A4          |
| SCL      | A5          |

## 2. Flash the Arduino sketch

1. Open `arduino/tmag5273_serial_stream/tmag5273_serial_stream.ino` in the Arduino IDE.
2. Install libraries via **Tools > Manage Libraries**:
   - `Adafruit TMAG5273`
   - `Adafruit BusIO`
   - `Adafruit Unified Sensor`
3. Select **Tools > Board > Arduino Uno** and the Uno's **Port**.
4. Upload.
5. Close the Arduino IDE's Serial Monitor afterwards — only one program can
   hold the COM port open at a time, and the Python display needs it next.

The sketch tries all four factory I2C addresses the TMAG5273 ships in
(A1-A4 variants), so it works regardless of which one you have.

## 3. Run the display on your laptop (Windows)

Requires Python 3 ([python.org](https://www.python.org/downloads/), check
"Add python.exe to PATH" during install).

```
cd python
pip install -r requirements.txt
python tmag5273_display.py
```

If more than one serial port is listed, pick the one matching your Arduino
(check Windows Device Manager under "Ports (COM & LPT)" if unsure). A dark
window opens with three colored bars (X/Y/Z) and live microtesla readings
that update in real time as you move a magnet near the sensor.

## Notes

- The bar scale is set to +/-133000 uT (`FULL_SCALE_UT` in
  `tmag5273_display.py`), matching the TMAG5273's high-range (x2) variant.
  If your board is the standard-range (x1) variant, lower it to `80000` for
  bars that use the full height.
- Update rate is set by the `delay(50)` in the Arduino sketch (~20 Hz).

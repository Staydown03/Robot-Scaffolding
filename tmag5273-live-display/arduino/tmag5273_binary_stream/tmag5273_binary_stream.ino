/*
  TMAG5273 -> high-rate binary serial streamer
  Board: Arduino Uno Rev3
  Sensor: Adafruit TMAG5273 3D Hall Effect Magnetometer breakout

  Feeds tmag5273_3d.py, the live 3D magnet visualization. For the simpler
  human-readable CSV stream (and the 2D bar display), use
  tmag5273_serial_stream.ino instead.

  Wiring (I2C):
    TMAG5273 VIN -> Uno 5V
    TMAG5273 GND -> Uno GND
    TMAG5273 SDA -> Uno A4 (SDA)      <-- ANALOG header, not digital pin 4
    TMAG5273 SCL -> Uno A5 (SCL)      <-- ANALOG header, not digital pin 5

  Arduino IDE setup:
    Tools > Manage Libraries, install:
      - Adafruit TMAG5273
      - Adafruit BusIO
      - Adafruit Unified Sensor
    Tools > Board > Arduino Uno, Tools > Port > (your Uno's COM port)

  Protocol: one ASCII header line at boot, then fixed 10-byte binary frames.

    #TMAG5273 v=1 variant=x2 range_xy=266.0 range_z=266.0 addr=0x35\n

    byte 0-1  0xAA 0x55   sync
    byte 2    seq         uint8, wraps - lets the host detect dropped frames
    byte 3-8  int16 LE    raw x, y, z counts
    byte 9    checksum    XOR of bytes 2..8

  Raw counts are sent rather than microtesla so the stream stays lossless;
  the host converts with uT = (raw / 32768) * range_mT * 1000 using the ranges
  from the header.
*/

#include <Adafruit_TMAG5273.h>
#include <Wire.h>

// 500000 divides exactly from the Uno's 16MHz clock (UBRR=3, 0% error).
// 460800 does NOT - the closest divisor is 8.5% off, which corrupts framing.
// Drop to 250000 (also exact) if your USB serial driver dislikes 500000.
#define STREAM_BAUD 500000

// I2C fast mode. The sensor supports it and it roughly quarters the time
// spent per sample on the bus.
#define I2C_CLOCK_HZ 400000

#define FRAME_SYNC0 0xAA
#define FRAME_SYNC1 0x55

Adafruit_TMAG5273 tmag;

// The sensor ships in four factory I2C address variants (A1-A4).
const uint8_t kCandidateAddresses[] = {0x35, 0x22, 0x78, 0x44};

uint8_t sensorAddress = 0;
uint8_t sequence = 0;

void setup() {
  Serial.begin(STREAM_BAUD);
  while (!Serial) {
    delay(10);
  }

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  bool found = false;
  for (uint8_t i = 0; i < sizeof(kCandidateAddresses); i++) {
    if (tmag.begin(kCandidateAddresses[i])) {
      sensorAddress = kCandidateAddresses[i];
      found = true;
      break;
    }
  }

  if (!found) {
    Serial.println(F("ERR: TMAG5273 not found - check wiring/address"));
    while (1) {
      delay(10);
    }
  }

  // Configure explicitly rather than trusting begin()'s defaults.
  tmag.setMagneticChannels(TMAG5273_MAG_CH_XYZ);
  tmag.setOperatingMode(TMAG5273_MODE_CONTINUOUS);
  // 8x averaging, not 32x: heavier averaging is what caps the sample rate, and
  // micro-vibration work needs the bandwidth. The host smooths for display.
  tmag.setConversionAverage(TMAG5273_CONV_AVG_8X);

  bool isX2 = (tmag.getDeviceID() & 0x03) == 0x02;
  float rangeXY;
  float rangeZ;
  if (isX2) {
    rangeXY = tmag.getXYRangeWide() ? 266.0 : 133.0;
    rangeZ = tmag.getZRangeWide() ? 266.0 : 133.0;
  } else {
    rangeXY = tmag.getXYRangeWide() ? 80.0 : 40.0;
    rangeZ = tmag.getZRangeWide() ? 80.0 : 40.0;
  }

  Serial.print(F("#TMAG5273 v=1 variant="));
  Serial.print(isX2 ? F("x2") : F("x1"));
  Serial.print(F(" range_xy="));
  Serial.print(rangeXY, 1);
  Serial.print(F(" range_z="));
  Serial.print(rangeZ, 1);
  Serial.print(F(" addr=0x"));
  Serial.println(sensorAddress, HEX);
  Serial.flush();
}

// Reads all three axes in one I2C transaction. Results live in contiguous
// registers 0x12..0x17 (MSB first per axis), so a burst read costs a third of
// what three separate readX/readY/readZ calls would.
bool readAxesBurst(int16_t *x, int16_t *y, int16_t *z) {
  Wire.beginTransmission(sensorAddress);
  Wire.write(TMAG5273_REG_X_MSB_RESULT);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(sensorAddress, (uint8_t)6) != 6) {
    return false;
  }

  uint8_t raw[6];
  for (uint8_t i = 0; i < 6; i++) {
    raw[i] = Wire.read();
  }

  *x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
  *y = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
  *z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
  return true;
}

void loop() {
  int16_t x;
  int16_t y;
  int16_t z;

  if (!readAxesBurst(&x, &y, &z)) {
    return;  // transient bus error; just try again next pass
  }

  uint8_t frame[10];
  frame[0] = FRAME_SYNC0;
  frame[1] = FRAME_SYNC1;
  frame[2] = sequence++;
  frame[3] = (uint8_t)(x & 0xFF);
  frame[4] = (uint8_t)((x >> 8) & 0xFF);
  frame[5] = (uint8_t)(y & 0xFF);
  frame[6] = (uint8_t)((y >> 8) & 0xFF);
  frame[7] = (uint8_t)(z & 0xFF);
  frame[8] = (uint8_t)((z >> 8) & 0xFF);

  uint8_t checksum = 0;
  for (uint8_t i = 2; i < 9; i++) {
    checksum ^= frame[i];
  }
  frame[9] = checksum;

  Serial.write(frame, sizeof(frame));
}

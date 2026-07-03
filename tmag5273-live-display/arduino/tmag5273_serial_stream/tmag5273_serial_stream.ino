/*
  TMAG5273 -> Serial CSV streamer
  Board: Arduino Uno Rev3
  Sensor: Adafruit TMAG5273 3D Hall Effect Magnetometer breakout (any A1-A4 addr variant)

  Wiring (I2C):
    TMAG5273 VIN -> Uno 5V
    TMAG5273 GND -> Uno GND
    TMAG5273 SDA -> Uno A4 (SDA)
    TMAG5273 SCL -> Uno A5 (SCL)

  Arduino IDE setup:
    Tools > Manage Libraries, install:
      - Adafruit TMAG5273
      - Adafruit BusIO
      - Adafruit Unified Sensor
    Tools > Board > Arduino Uno, Tools > Port > (your Uno's COM port)

  Streams "X,Y,Z\n" (magnetic field in microtesla, one decimal place)
  over Serial at 115200 baud. Read by tmag5273_display.py on the laptop.
*/

#include <Adafruit_TMAG5273.h>

Adafruit_TMAG5273 tmag;

// The sensor ships in four factory I2C address variants (A1-A4).
// Try each known address until one responds instead of hardcoding one.
const uint8_t kCandidateAddresses[] = {0x35, 0x22, 0x78, 0x44};

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  bool found = false;
  for (uint8_t i = 0; i < sizeof(kCandidateAddresses); i++) {
    if (tmag.begin(kCandidateAddresses[i])) {
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

  // Hardware-average 32 samples per reading instead of the default 1 -
  // trades a bit of update speed for much less sensor noise.
  tmag.setConversionAverage(TMAG5273_CONV_AVG_32X);
}

void loop() {
  float x = tmag.readMagneticX();
  float y = tmag.readMagneticY();
  float z = tmag.readMagneticZ();

  Serial.print(x, 1);
  Serial.print(',');
  Serial.print(y, 1);
  Serial.print(',');
  Serial.println(z, 1);

  delay(50);
}

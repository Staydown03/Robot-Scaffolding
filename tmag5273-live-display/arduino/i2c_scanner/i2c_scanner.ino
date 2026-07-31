/*
  I2C bus scanner - run this BEFORE the TMAG5273 sketch if the sensor
  isn't being found. Reports every I2C address that responds.

  Wiring (same as the TMAG5273 sketch):
    Sensor VIN -> Uno 5V
    Sensor GND -> Uno GND
    Sensor SDA -> Uno A4
    Sensor SCL -> Uno A5

  Open Tools > Serial Monitor at 115200 baud after uploading.
*/

#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println(F("I2C Scanner starting..."));
}

void loop() {
  int found = 0;

  Serial.println(F("Scanning..."));
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("Device found at address 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println(F("No I2C devices found - check wiring/power."));
  } else {
    Serial.print(found);
    Serial.println(F(" device(s) found."));
  }

  delay(3000);
}

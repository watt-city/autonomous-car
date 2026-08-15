#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor

  // Default ESP32-S3 I2C pins: SDA = 8, SCL = 9
  Wire.begin(8, 9);
  Serial.println("\n--- I2C Scanner Running ---");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found. Check wiring (SDA=GPIO8, SCL=GPIO9, 3V3, GND).");
  } else {
    Serial.println("Scan complete.\n");
  }

  delay(4000); 
}
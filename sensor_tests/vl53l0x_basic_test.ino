// vl53l0x_basic_test.ino -- bring up a VL53L0X (VL53L0CXV0DH/1) breakout
// Board: ESP32S3 Dev Module, USB CDC On Boot: Enabled. Serial Monitor 115200.
// Library: "Adafruit_VL53L0X" (Library Manager).
//
// Wiring (WATT City DevKit pins):
//   Module VIN/VCC -> 3V3      Module GND -> GND
//   Module SDA     -> GPIO8    Module SCL -> GPIO9
//   Module XSHUT   -> GPIO12   (optional; leave unconnected if your board pulls it up)
//   Module GPIO1   -> not used

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define I2C_SDA    8
#define I2C_SCL    9
#define XSHUT_PIN  12     // set to -1 if XSHUT is not wired

Adafruit_VL53L0X lox;

// Step 1: is anything answering on the I2C bus?
void i2cScan() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  device at 0x%02X%s\n", addr, addr == 0x29 ? "  <- VL53L0X" : "");
      found++;
    }
  }
  if (!found) Serial.println("  nothing found: check power, GND, and try swapping SDA/SCL");
}

void setup() {
  Serial.begin(115200);
  delay(1500);                         // give USB serial time to connect
  Serial.println("\nVL53L0X basic test");

  if (XSHUT_PIN >= 0) {                // XSHUT low = sensor off; pulse it to reset
    pinMode(XSHUT_PIN, OUTPUT);
    digitalWrite(XSHUT_PIN, LOW);
    delay(10);
    digitalWrite(XSHUT_PIN, HIGH);
    delay(10);
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  i2cScan();

  // Step 2: start the sensor
  if (!lox.begin(0x29, false, &Wire)) {
    Serial.println("VL53L0X did not start. Fix the scan result above first.");
    while (true) delay(100);
  }
  Serial.println("VL53L0X ready. Move your hand in front of it.\n");
}

// Step 3: read distance
void loop() {
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);

  if (m.RangeStatus == 4) {
    Serial.println("out of range");          // nothing close enough, or too dark/shiny
  } else {
    Serial.printf("%4u mm   (status %u)\n", m.RangeMilliMeter-40, m.RangeStatus);
  }
  delay(100);
}

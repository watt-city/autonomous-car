#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define I2C_SDA 8
#define I2C_SCL 9

// CALIBRATION OFFSET (in mm)
const int OFFSET_MM = -17; 

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!lox.begin(0x29, false, &Wire)) {
    Serial.println("❌ Failed to boot VL53L0X!");
    while (1) delay(10);
  }

  // 🎯 Optional: Increase measurement timing budget for higher accuracy (+/- 3%)
  // Default is 33ms. Setting to 200ms gives much higher accuracy/calibration precision!
  lox.setMeasurementTimingBudgetMicroSeconds(200000); 

  Serial.println("✅ VL53L0X Calibrated & Ready!");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); 

  if (measure.RangeStatus != 4) {
    int raw_mm = measure.RangeMilliMeter;
    
    // Apply calibration offset
    int calibrated_mm = raw_mm + OFFSET_MM;
    
    // Clamp readings below physical sensor limit (~30mm)
    if (calibrated_mm < 30 && raw_mm < 50) {
      calibrated_mm = 0; // Treat optical blind spot / dead zone as near 0
    }

    Serial.print("Raw: "); Serial.print(raw_mm);
    Serial.print(" mm  |  Calibrated: "); Serial.print(calibrated_mm);
    Serial.print(" mm  ("); Serial.print(calibrated_mm / 10.0, 1); Serial.println(" cm)");
  } else {
    Serial.println("Out of range (>1.2m)");
  }

  delay(100);
}
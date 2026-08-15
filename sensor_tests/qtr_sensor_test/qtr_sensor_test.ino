#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint8_t sensorPins[SensorCount] = {1, 2, 3, 4, 5, 6, 7, 10};

void setup() {
  Serial.begin(115200);
  delay(1000);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(11); // CTRL Pin

  // 1. Allocate memory for calibration structures
  qtr.calibrate();

  // 2. Define your calibrated baseline arrays
  uint16_t savedMin[8] = {248, 194, 194, 208, 224, 248, 259, 288};
  uint16_t savedMax[8] = {2500, 2500, 2122, 2474, 2405, 2500, 2500, 2500};

  // 3. Overwrite internal library calibration values
  for (uint8_t i = 0; i < SensorCount; i++) {
    qtr.calibrationOn.minimum[i] = savedMin[i];
    qtr.calibrationOn.maximum[i] = savedMax[i];
  }

  Serial.println("✅ Saved Calibration Loaded! Printing individual IR pair values.");
}

void loop() {
  uint16_t sensorValues[SensorCount];

  // Read calibrated values (0 = White, 1000 = Black) for each IR pair
  qtr.readCalibrated(sensorValues);

  // Print each sensor pair side-by-side
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(sensorValues[i]);
    if (i < SensorCount - 1) Serial.print("\t");
  }
  Serial.println();

  delay(100);
}
#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint8_t sensorPins[SensorCount] = {1, 2, 3, 4, 5, 6, 7, 10};

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Configure QTR sensor array for RC mode
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(11); // CTRL Pin

  Serial.println("\n--- CALIBRATING: Sweep all 8 sensors over black line & white paper NOW! ---");

  // Calibrate for 5 seconds (200 iterations * 25ms = 5000ms)
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(25);
  }

  Serial.println("\n=======================================================");
  Serial.println("✅ CALIBRATION COMPLETE! COPY AND PASTE THESE ARRAYS:");
  Serial.println("=======================================================\n");

  // Print minimum (white) calibration array
  Serial.print("uint16_t savedMin[8] = {");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    if (i < SensorCount - 1) Serial.print(", ");
  }
  Serial.println("};");

  // Print maximum (black) calibration array
  Serial.print("uint16_t savedMax[8] = {");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    if (i < SensorCount - 1) Serial.print(", ");
  }
  Serial.println("};\n");

  Serial.println("=======================================================");
  Serial.println("⏸️ Pausing for 10 seconds so you can copy the values...");
  Serial.println("=======================================================\n");
  
  delay(10000); // 10-second pause to copy text easily

  Serial.println("--- Starting Live Sensor Readings ---\n");
}

void loop() {
  uint16_t sensorValues[SensorCount];

  // Read calibrated values (0 = White, 1000 = Black)
  qtr.readCalibrated(sensorValues);

  // Print all 8 IR pair readings side-by-side
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
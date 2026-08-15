#include <QTRSensors.h>

QTRSensors qtr;

// Configure 8 analog channels on ESP32-S3
const uint8_t SensorCount = 8;
uint8_t sensorPins[SensorCount] = {1, 2, 3, 4, 5, 6, 7, 10};

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Configure QTR sensor array
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(11); // CTRL Pin

  Serial.println("\n--- CALIBRATING: Sweep sensor over black line & white paper now! ---");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Calibrate for 5 seconds (200 samples)
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(20);
  }
  
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("✅ Calibration Complete!\n");
}

void loop() {
  uint16_t sensorValues[SensorCount];
  uint16_t position = qtr.readLineBlack(sensorValues);

  // 3. Print calibrated values (0 - 1000)
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  
  // 4. Print calculated line position
  Serial.print(" | Line Position: ");
  Serial.println(position);

  delay(100);
}
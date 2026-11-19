
#include <Arduino.h>
//the pins and the BAUD that are going to be used
#define CAMERA_RX_PIN 44
#define CAMERA_TX_PIN 43
#define CAMERA_BAUD 115200

HardwareSerial CameraUART(1);

int targetTrafficLight = -1;

unsigned long lastDetectionTime = 0;

const unsigned long TARGET_TIMEOUT = 1000;

void readCamera() {

  while (CameraUART.available()) {

    String message = CameraUART.readStringUntil('\n');
    message.trim();
    if (message.startsWith("TAG,")) {

      int commaIndex = message.indexOf(',', 4);

      String idText;

      if (commaIndex == -1) {
        idText = message.substring(4);
      } else {
        idText = message.substring(4, commaIndex);
      }

      int tagID = idText.toInt();
      if (tagID == -1) {
        continue;
      }
      lastDetectionTime = millis();
      if (tagID != targetTrafficLight) {

        targetTrafficLight = tagID;

        Serial.print("Selected traffic light ID: ");
        Serial.println(targetTrafficLight);
      }
      Serial.print("Camera detected TAG ID: ");
      Serial.println(tagID);
    }
  }
}
void setup() {

  Serial.begin(115200);

  CameraUART.begin(
    CAMERA_BAUD,
    SERIAL_8N1,
    CAMERA_RX_PIN,
    CAMERA_TX_PIN
  );

  delay(1000);
  Serial.println("Waiting for detection.");
}

void loop() {
  readCamera();
  if (targetTrafficLight != -1 &&
      millis() - lastDetectionTime > TARGET_TIMEOUT) {
    targetTrafficLight = -1;
    Serial.println("Traffic light target lost");
  }
}

#include <Wire.h>
#include <QTRSensors.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_VL53L0X.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- BLE UUIDs (Standard Nordic UART Service) ---
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// --- Pin Definitions ---
#define I2C_SDA 8
#define I2C_SCL 9
#define TOF_XSHUT 12
#define QTR_CTRL 11

const uint8_t SensorCount = 8;
uint8_t qtrPins[SensorCount] = {1, 2, 3, 4, 5, 6, 7, 10};

// --- Objects ---
QTRSensors qtr;
Adafruit_LSM6DS3TRC lsm;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
bool deviceConnected = false;
unsigned long lastValidToFTime = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      // Restart advertising so you can reconnect without rebooting
      BLEDevice::startAdvertising(); 
    }
};

void resetVL53L0X() {
  pinMode(TOF_XSHUT, OUTPUT);
  digitalWrite(TOF_XSHUT, LOW);
  delay(20);
  digitalWrite(TOF_XSHUT, HIGH);
  delay(20);
}

void btPrint(String msg) {
  Serial.print(msg);
  if (deviceConnected && pTxCharacteristic != NULL) {
    pTxCharacteristic->setValue(msg.c_str());
    pTxCharacteristic->notify();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1. Initialize BLE
  BLEDevice::init("ESP32S3_Car");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("\n--- BLE Started: Connect via BLE Terminal App to 'ESP32S3_Car' ---");

  // 2. Hardware Reset ToF
  resetVL53L0X();

  // 3. Init I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); 
  Wire.setTimeOut(30); 

  // 4. Init Sensors
  if (!lsm.begin_I2C(0x6A, &Wire)) {
    btPrint("❌ IMU Not Found!\n");
  } else {
    btPrint("✅ IMU Ready!\n");
  }

  if (!lox.begin(0x29, false, &Wire)) {
    btPrint("❌ VL53L0X Not Found!\n");
  } else {
    lox.startRangeContinuous(50);
    btPrint("✅ VL53L0X Ready!\n");
  }

  // 5. Init QTR
  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);
  qtr.setEmitterPin(QTR_CTRL);
  qtr.calibrate();

  uint16_t savedMin[8] = {199, 180, 190, 214, 234, 263, 272, 290};
  uint16_t savedMax[8] = {1453, 1259, 1216, 1187, 1138, 1128, 1079, 1259};

  for (uint8_t i = 0; i < SensorCount; i++) {
    qtr.calibrationOn.minimum[i] = savedMin[i];
    qtr.calibrationOn.maximum[i] = savedMax[i];
  }
  btPrint("✅ QTR Calibration Loaded!\n\n");
}

void loop() {
  String telemetry = "";

  // --- A. QTR Read ---
  uint16_t qtrValues[SensorCount];
  qtr.readCalibrated(qtrValues);
  telemetry += "QTR:[ ";
  for (uint8_t i = 0; i < SensorCount; i++) {
    telemetry += String(qtrValues[i]);
    if (i < SensorCount - 1) telemetry += "\t";
  }
  telemetry += " ]";

  // --- B. 6-DoF IMU Read ---
  sensors_event_t accel, gyro, temp;
  lsm.getEvent(&accel, &gyro, &temp);

  telemetry += " | ACCEL[X:" + String(accel.acceleration.x, 1) + 
               " Y:" + String(accel.acceleration.y, 1) + 
               " Z:" + String(accel.acceleration.z, 1) + "]";

  telemetry += " | GYRO[X:" + String(gyro.gyro.x, 2) + 
               " Y:" + String(gyro.gyro.y, 2) + 
               " Z:" + String(gyro.gyro.z, 2) + "]";

  // --- C. ToF Read ---
  telemetry += " | DIST: ";
  if (lox.isRangeComplete()) {
    uint16_t distance = lox.readRange();
    lastValidToFTime = millis();
    
    if (distance < 8190) { 
      telemetry += String(distance) + "mm\n";
    } else {
      telemetry += "Out of range\n";
    }
  } else {
    telemetry += "Waiting...\n";
    
    if (millis() - lastValidToFTime > 500) {
      btPrint("⚠️ ToF Froze! Auto-rebooting...\n");
      resetVL53L0X();
      lox.begin(0x29, false, &Wire);
      lox.startRangeContinuous(50);
      lastValidToFTime = millis();
    }
  }

  btPrint(telemetry);
  delay(50);
}
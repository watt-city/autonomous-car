#include <Arduino.h>
#include <Wire.h>
#include <QTRSensors.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_VL53L0X.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- PIN DEFINITIONS ---
// I2C & Sensors
#define I2C_SDA          8
#define I2C_SCL          9
#define VL53_XSHUT       12

// QTR-8RC Line Array
#define QTR_CTRL         11
const uint8_t SensorCount = 8;
uint8_t qtrPins[SensorCount] = {1, 2, 3, 4, 5, 6, 7, 10};

// Motor A (Left Motor)
#define LEFT_PWMA        15
#define LEFT_AIN1        16
#define LEFT_AIN2        17
#define LEFT_ENC_A       13
#define LEFT_ENC_B       14

// Motor B (Right Motor)
#define RIGHT_PWMB       18
#define RIGHT_BIN1       39
#define RIGHT_BIN2       40
#define RIGHT_ENC_A      21
#define RIGHT_ENC_B      38

// --- BLE NORDIC UART UUIDs ---
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// --- OBJECT INSTANCES ---
QTRSensors qtr;
Adafruit_LSM6DS3TRC lsm;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
bool deviceConnected = false;

// --- ENCODER TICK COUNTERS ---
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// ISR: Left Encoder
void IRAM_ATTR handleLeftEncoder() {
  if (digitalRead(LEFT_ENC_B) == HIGH) {
    leftTicks++;
  } else {
    leftTicks--;
  }
}

// ISR: Right Encoder (Inverted logic to match motor physical mounting)
void IRAM_ATTR handleRightEncoder() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) {
    rightTicks--;
  } else {
    rightTicks++;
  }
}

// --- PID CONTROL VARIABLES ---
float Kp = 0.045;
float Ki = 0.0000;
float Kd = 0.380;

int baseSpeed = 160;  // Default cruising PWM speed (0 - 255)
int maxSpeed  = 220;  // PWM speed ceiling
int lastError = 0;
float integral = 0;

// --- SAFETY PARAMETERS ---
const uint16_t MIN_OBSTACLE_DIST_MM = 80; // Stop if object closer than 80mm
unsigned long lastTelemetryTime = 0;

// --- BLE SERVER CALLBACKS ---
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    // Restart advertising so other central devices can discover
    pServer->getAdvertising()->start();
  }
};

// Function prototypes
void setMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void sendBLETelemetry(uint16_t linePos, uint16_t tofDist, float gyroZ);

void setup() {
  Serial.begin(115200);

  // 1. Initialize Wire (I2C)
  Wire.begin(I2C_SDA, I2C_SCL);

  // 2. Motor Driver Pin Setup
  pinMode(LEFT_PWMA, OUTPUT);
  pinMode(LEFT_AIN1, OUTPUT);
  pinMode(LEFT_AIN2, OUTPUT);

  pinMode(RIGHT_PWMB, OUTPUT);
  pinMode(RIGHT_BIN1, OUTPUT);
  pinMode(RIGHT_BIN2, OUTPUT);
  stopMotors();

  // 3. Encoder Pin Setup & Interrupt Registration
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), handleLeftEncoder, RISING);

  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), handleRightEncoder, RISING);

  // 4. VL53L0X ToF Reset & Initialization
  pinMode(VL53_XSHUT, OUTPUT);
  digitalWrite(VL53_XSHUT, LOW);
  delay(10);
  digitalWrite(VL53_XSHUT, HIGH);
  delay(10);

  if (!lox.begin()) {
    Serial.println("Failed to detect VL53L0X ToF sensor");
  }

  // 5. LSM6DS3TR-C IMU Initialization
  if (!lsm.begin_I2C()) {
    Serial.println("Failed to detect LSM6DS3TR-C IMU");
  }

  // 6. QTR Sensor Array Setup & Calibration Loading
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

  // 7. Initialize BLE Service
  BLEDevice::init("ESP32S3_LineFollower");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("System Initialized! Starting PID Autonomous Mode...");
}

void loop() {
  // 1. Check Distance Sensor Safety Gate
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  uint16_t distanceMm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;
  
  if (distanceMm < MIN_OBSTACLE_DIST_MM) {
    stopMotors(); // Obstacle detected -> Emergency Halt
    return;
  }

  // 2. Read QTR Line Position (0 to 7000)
  uint16_t sensorValues[SensorCount];
  uint16_t linePosition = qtr.readLineBlack(sensorValues);

  // 3. Compute PID Error Signal (-3500 to +3500)
  int error = (int)linePosition - 3500;

  float pTerm = Kp * error;
  integral += error;
  float iTerm = Ki * integral;
  float dTerm = Kd * (error - lastError);

  float motorAdjustment = pTerm + iTerm + dTerm;
  lastError = error;

  // 4. Compute Left/Right Speeds
  int leftMotorSpeed  = baseSpeed + motorAdjustment;
  int rightMotorSpeed = baseSpeed - motorAdjustment;

  leftMotorSpeed  = constrain(leftMotorSpeed, 0, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

  // 5. Drive Motors
  setMotors(leftMotorSpeed, rightMotorSpeed);

  // 6. BLE Telemetry Stream (20 Hz Rate Limit)
  if (millis() - lastTelemetryTime > 50) {
    lastTelemetryTime = millis();

    sensors_event_t accel, gyro, temp;
    lsm.getEvent(&accel, &gyro, &temp);

    sendBLETelemetry(linePosition, distanceMm, gyro.gyro.z);
  }
}

// Set Left & Right motor speed directions (-255 to 255)
void setMotors(int leftSpeed, int rightSpeed) {
  // Left Motor Direction
  if (leftSpeed >= 0) {
    digitalWrite(LEFT_AIN1, HIGH);
    digitalWrite(LEFT_AIN2, LOW);
  } else {
    digitalWrite(LEFT_AIN1, LOW);
    digitalWrite(LEFT_AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(LEFT_PWMA, constrain(leftSpeed, 0, 255));

  // Right Motor Direction
  if (rightSpeed >= 0) {
    digitalWrite(RIGHT_BIN1, HIGH);
    digitalWrite(RIGHT_BIN2, LOW);
  } else {
    digitalWrite(RIGHT_BIN1, LOW);
    digitalWrite(RIGHT_BIN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(RIGHT_PWMB, constrain(rightSpeed, 0, 255));
}

// Emergency stop helper
void stopMotors() {
  analogWrite(LEFT_PWMA, 0);
  analogWrite(RIGHT_PWMB, 0);
  digitalWrite(LEFT_AIN1, LOW);
  digitalWrite(LEFT_AIN2, LOW);
  digitalWrite(RIGHT_BIN1, LOW);
  digitalWrite(RIGHT_BIN2, LOW);
}

// Stream CSV packet over BLE: "Pos,Dist,GyroZ,LTicks,RTicks"
void sendBLETelemetry(uint16_t linePos, uint16_t tofDist, float gyroZ) {
  if (deviceConnected && pTxCharacteristic != NULL) {
    char txBuffer[80];
    snprintf(txBuffer, sizeof(txBuffer), "%u,%u,%.2f,%ld,%ld\n", 
             linePos, tofDist, gyroZ, leftTicks, rightTicks);
    
    pTxCharacteristic->setValue((uint8_t*)txBuffer, strlen(txBuffer));
    pTxCharacteristic->notify();
  }
}
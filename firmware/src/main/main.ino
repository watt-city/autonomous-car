#include <Wire.h>
#include <QTRSensors.h>

// Motor Controller TB6612FNG Pins ---
#define STBY        17
#define PWMA        13
#define AIN1        14
#define AIN2        21
#define PWMB        15
#define BIN1        16
#define BIN2        18

// QTR Sensor Setup ---
#define QTR_CTRL    11
const uint8_t SensorCount = 8;
uint8_t qtrPins[SensorCount] = {1, 2, 3, 4, 5, 6, 7, 10};

QTRSensors qtr;

// --- PID Tuning Parameters ---
float Kp = 0.04;
float Ki = 0.0000;
float Kd = 0.35;

int baseSpeed = 160;  // Cruising PWM speed (0 - 255)
int maxSpeed  = 220;  // Max PWM speed ceiling

int lastError = 0;
float integral = 0;

void setup() {
  Serial.begin(115200);

  // 1. Motor Driver Pins
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Enable Motor Driver
  digitalWrite(STBY, HIGH);

  // 2. QTR Sensor Array Setup
  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);
  qtr.setEmitterPin(QTR_CTRL);

  // Load Saved Calibration Data
  qtr.calibrate();
  uint16_t savedMin[8] = {199, 180, 190, 214, 234, 263, 272, 290};
  uint16_t savedMax[8] = {1453, 1259, 1216, 1187, 1138, 1128, 1079, 1259};
  
  for (uint8_t i = 0; i < SensorCount; i++) {
    qtr.calibrationOn.minimum[i] = savedMin[i];
    qtr.calibrationOn.maximum[i] = savedMax[i];
  }
}

void loop() {
  // 1. Get exact position over line (0 = far left, 3500 = center, 7000 = far right)
  uint16_t sensorValues[SensorCount];
  uint16_t position = qtr.readLineBlack(sensorValues);

  // 2. Compute position error relative to center
  int error = (int)position - 3500;

  // 3. Compute PID loop terms
  float pTerm = Kp * error;
  integral += error;
  float iTerm = Ki * integral;
  float dTerm = Kd * (error - lastError);

  float motorAdjustment = pTerm + iTerm + dTerm;
  lastError = error;

  // 4. Calculate Left and Right motor speeds
  int leftMotorSpeed  = baseSpeed + motorAdjustment;
  int rightMotorSpeed = baseSpeed - motorAdjustment;

  // Clamp values within motor operating range (0 to maxSpeed)
  leftMotorSpeed  = constrain(leftMotorSpeed, 0, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

  // 5. Output signals to TB6612 motor driver
  driveMotors(leftMotorSpeed, rightMotorSpeed);
}

void driveMotors(int leftSpeed, int rightSpeed) {
  // Left Motor Forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, leftSpeed);

  // Right Motor Forward
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, rightSpeed);
}
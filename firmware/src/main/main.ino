#include <Arduino.h>

// --- Motor Pins (Left = Motor A, Right = Motor B) ---
#define LEFT_PWMA   15
#define LEFT_AIN1   16
#define LEFT_AIN2   17

#define RIGHT_PWMB  18
#define RIGHT_BIN1  39  // Changed from 19 to avoid USB D- conflict
#define RIGHT_BIN2  40  // Changed from 20 to avoid USB D+ conflict

// --- Encoder Pins ---
#define LEFT_ENC_A  13
#define LEFT_ENC_B  14

#define RIGHT_ENC_A 21
#define RIGHT_ENC_B 38

// --- Quadrature Tick Counters ---
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// ISR for Left Encoder
void IRAM_ATTR handleLeftEncoder() {
  if (digitalRead(LEFT_ENC_B) == HIGH) {
    leftTicks++;
  } else {
    leftTicks--;
  }
}

// ISR for Right Encoder (Inverted logic to match physical mounting)
void IRAM_ATTR handleRightEncoder() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) {
    rightTicks--;  // Flipped ++ to --
  } else {
    rightTicks++;  // Flipped -- to ++
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give USB serial connection time to establish

  // 1. Setup Motor Output Pins
  pinMode(LEFT_PWMA, OUTPUT);
  pinMode(LEFT_AIN1, OUTPUT);
  pinMode(LEFT_AIN2, OUTPUT);

  pinMode(RIGHT_PWMB, OUTPUT);
  pinMode(RIGHT_BIN1, OUTPUT);
  pinMode(RIGHT_BIN2, OUTPUT);

  // 2. Setup Encoder Inputs
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), handleLeftEncoder, RISING);

  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), handleRightEncoder, RISING);

  Serial.println("\n--- Dual Motor & Encoder USB-Safe Test ---");
}

void loop() {
  // --- Drive Both Forward ---
  Serial.println(">>> Driving BOTH FORWARD...");
  setMotors(180, 180);
  printTicksForDuration(2000);

  stopMotors();
  delay(1000);

  // --- Drive Both Reverse ---
  Serial.println(">>> Driving BOTH REVERSE...");
  setMotors(-180, -180);
  printTicksForDuration(2000);

  stopMotors();
  delay(2000);
}

void setMotors(int leftSpeed, int rightSpeed) {
  // Left Motor
  if (leftSpeed >= 0) {
    digitalWrite(LEFT_AIN1, HIGH);
    digitalWrite(LEFT_AIN2, LOW);
  } else {
    digitalWrite(LEFT_AIN1, LOW);
    digitalWrite(LEFT_AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(LEFT_PWMA, constrain(leftSpeed, 0, 255));

  // Right Motor
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

void stopMotors() {
  analogWrite(LEFT_PWMA, 0);
  analogWrite(RIGHT_PWMB, 0);
  digitalWrite(LEFT_AIN1, LOW);
  digitalWrite(LEFT_AIN2, LOW);
  digitalWrite(RIGHT_BIN1, LOW);
  digitalWrite(RIGHT_BIN2, LOW);
}

void printTicksForDuration(unsigned long durationMs) {
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    Serial.print("LEFT Ticks: ");
    Serial.print(leftTicks);
    Serial.print(" \t| RIGHT Ticks: ");
    Serial.println(rightTicks);
    delay(100);
  }
}
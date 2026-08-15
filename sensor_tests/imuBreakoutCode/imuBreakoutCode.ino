#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>

#define I2C_SDA 8
#define I2C_SCL 9

Adafruit_LSM6DS3TRC lsm6ds;

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to attach

  Serial.println("\n--- ESP32-S3 LSM6DS3TRC Test ---");

  // Initialize Wire on custom pins BEFORE calling the library
  Wire.begin(I2C_SDA, I2C_SCL);

  // Pass 0x6A address and pointer to our initialized Wire instance
  if (!lsm6ds.begin_I2C(0x6A, &Wire)) {
    Serial.println("❌ Failed to find LSM6DS3TRC at 0x6A!");
    while (1) {
      delay(10);
    }
  }

  Serial.println("✅ LSM6DS3TRC Found and Ready!");
}

void loop() {
  sensors_event_t accel, gyro, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);

  Serial.print("Accel X: "); Serial.print(accel.acceleration.x, 2);
  Serial.print(" | Y: "); Serial.print(accel.acceleration.y, 2);
  Serial.print(" | Z: "); Serial.print(accel.acceleration.z, 2);
  
  Serial.print("  ||  Gyro X: "); Serial.print(gyro.gyro.x, 2);
  Serial.print(" | Y: "); Serial.print(gyro.gyro.y, 2);
  Serial.print(" | Z: "); Serial.println(gyro.gyro.z, 2);

  delay(150);
}
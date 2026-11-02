# WATT City: Autonomous Car

Custom PCB hardware, PID line-following firmware, and breakout bring-up tests for the WATT City autonomous car.

> **Current Hardware Status:** All Arduino sketches currently in `firmware/` and `sensor_tests/` are configured for our **ESP32-S3 DevKit + breakout board prototype** (LSM6DS3TR-C I2C breakout, VL53L0X breakout, Pololu QTR-8RC, TB6612FNG breakout). The final custom PCB in `hardware/` uses a bare **ESP32-S3-WROOM-1-N8** module with an onboard **LSM6DSM** IMU over SPI; pin definitions and IMU initialization will be updated when bringing up the custom PCB.

**Owned by:** Car team · **Leads:** Alisa Zhang, Katie Tao · **Questions:** `#watt-city` on WATT Slack

Part of [WATT City](https://github.com/watt-city), IEEE WATT's year-long project at UCLA. Related repos: [`camera`](https://github.com/watt-city/camera), [`autonomous-stoplight`](https://github.com/watt-city/autonomous-stoplight).

---

## Repo layout

```
hardware/                              ← KiCad 10.0 project for the custom Car PCB (ESP32-S3-WROOM-1-N8)
  watt_city_car_pcb.kicad_pro          ← Main KiCad project file
  watt_city_car_pcb.kicad_sch          ← Top-level hierarchical schematic
  watt_city_car_pcb.kicad_pcb          ← PCB layout (in progress)
  microcontroller.kicad_sch            ← ESP32-S3-WROOM-1-N8 + Boot/Reset buttons
  power.kicad_sch                      ← Power architecture (LDO, USB-C, protection)
    ldo.kicad_sch                      ← AP2112K-3.3 regulator
    powerProtection.kicad_sch          ← Reverse-polarity / diode protection
    usb.kicad_sch                      ← USB-C connector + ESD protection
  motorController.kicad_sch            ← Onboard TB6612FNG driver + left/right motor connectors
  distSensor.kicad_sch                 ← VL53L0X ToF distance sensor header
  irArray.kicad_sch                    ← QTR-8RC 8-channel IR line sensor header
  imu.kicad_sch                        ← Onboard LSM6DSM IMU (SPI) schematic sheet
firmware/                              ← Integrated car firmware (currently targeting DevKit prototype)
  src/main/main.ino                    ← CURRENT: PID line-following, ToF obstacle stop, encoders & BLE telemetry
sensor_tests/                          ← Standalone DevKit + breakout board bring-up & calibration sketches
  i2c_scanner/                         ← Pings I2C bus (SDA=8, SCL=9) to verify breakout addresses
  imu_lsm6ds3trc_test/                 ← Reads 3-axis accel + 3-axis gyro from LSM6DS3TR-C breakout (0x6A)
  qtr_sensor_calibration/              ← 5-second sweep calibration; prints savedMin/savedMax arrays
  qtr_sensor_test/                     ← Live calibrated QTR-8RC readout (0 = white, 1000 = black)
  vl53l0x_test/                        ← VL53L0X breakout distance readout (0x29) with -17 mm offset
  sensors_all_test/                    ← Combined QTR + IMU + ToF breakout test streaming over BLE UART
```

## Setup

**Arduino IDE**
1. Install the **esp32 by Espressif** board package.
2. Install the required libraries via *Tools → Manage Libraries*:
   - **QTRSensors** by Pololu
   - **Adafruit LSM6DS** by Adafruit (plus Adafruit Unified Sensor & BusIO dependencies)
   - **Adafruit VL53L0X** by Adafruit
3. Board settings (for both the DevKit and the bare `ESP32-S3-WROOM-1-N8` on the custom PCB):
   - Board: **ESP32S3 Dev Module**
   - **USB CDC On Boot: Enabled** (required on the custom PCB to print over native USB `GPIO 19`/`20`)
   - Flash Size: **8MB (64Mb)**

**KiCad**
- Open `hardware/watt_city_car_pcb.kicad_pro` in **KiCad 10.0+**.
- **Only one person edits the KiCad project at a time** (KiCad files cannot merge). Send a message in `#watt-city` on Slack before opening KiCad, and push as soon as you finish.

## DevKit Prototype Pinout (Current Sketches)

The table below reflects the wiring used in `firmware/src/main/main.ino` and `sensor_tests/` on the **ESP32-S3 DevKit + breakout prototype**:

| Subsystem | Signal | DevKit GPIO | Notes |
|---|---|---|---|
| **I2C Bus** | `I2C_SDA` / `I2C_SCL` | `GPIO 8` / `GPIO 9` | Shared by LSM6DS3TR-C breakout (`0x6A`) and VL53L0X (`0x29`) |
| **VL53L0X ToF** | `VL53_XSHUT` | `GPIO 12` | Active-low hardware reset / shutdown |
| **QTR-8RC Array** | `QTR_CTRL` (Emitter) | `GPIO 11` | IR LED emitter control |
| | Sensors `1..8` | `GPIO 1, 2, 3, 4, 5, 6, 7, 10` | RC reflectance inputs |
| **Left Motor (A)** | `LEFT_PWMA`, `AIN1`, `AIN2` | `GPIO 15`, `GPIO 16`, `GPIO 17` | TB6612FNG Channel A |
| | `LEFT_ENC_A`, `LEFT_ENC_B` | `GPIO 13`, `GPIO 14` | Left quadrature encoder |
| **Right Motor (B)** | `RIGHT_PWMB`, `BIN1`, `BIN2` | `GPIO 18`, `GPIO 39`, `GPIO 40` | TB6612FNG Channel B |
| | `RIGHT_ENC_A`, `RIGHT_ENC_B` | `GPIO 21`, `GPIO 38` | Right quadrature encoder (inverted in ISR) |

## Using the sketches

### `firmware/src/main`: PID autonomous line follower (DevKit prototype)
- Runs the closed-loop PID controller (`Kp = 0.045`, `Ki = 0.0`, `Kd = 0.380`, `baseSpeed = 160`) tracking a black line via `qtr.readLineBlack()` (`0` to `7000`, center at `3500`).
- **Safety gate:** Halts both motors immediately if the VL53L0X detects an obstacle closer than `MIN_OBSTACLE_DIST_MM` (`80 mm`).
- **BLE Telemetry:** Advertises as `ESP32S3_LineFollower` using the Nordic UART Service and streams CSV lines (`Pos,Dist,GyroZ,LTicks,RTicks`) at 20 Hz.

### `sensor_tests/` breakout workflow
1. **`i2c_scanner`**: Run first when wiring breakouts to confirm the IMU (`0x6A`) and VL53L0X (`0x29`) respond on `GPIO 8` / `GPIO 9`.
2. **`qtr_sensor_calibration`**: Sweep all 8 IR sensors across the black tape and white surface for 5 seconds. Copy the printed `savedMin` and `savedMax` arrays into `qtr_sensor_test` and `firmware/src/main/main.ino`.
3. **`vl53l0x_test` & `imu_lsm6ds3trc_test`**: Verify individual distance and gyro/accelerometer breakout readings over USB Serial (`115200` baud).
4. **`sensors_all_test`**: Full breakout suite burn-in over BLE (`ESP32S3_Car`) with automatic VL53L0X watchdog reset if the I2C bus hangs.

## Transitioning from DevKit Breakouts → Custom PCB

When the custom Car PCB (`hardware/watt_city_car_pcb`) arrives from fabrication, update the firmware for the bare `ESP32-S3-WROOM-1-N8` schematic (`microcontroller.kicad_sch`):
- **IMU (I2C → SPI):** The breakout prototype uses an `LSM6DS3TR-C` over I2C (`0x6A`), whereas the custom PCB places an onboard **`LSM6DSM`** connected over **SPI** (`IMU_SPI.SCK`, `MOSI`, `MISO`, `CS`, plus `IMU_INT1`/`INT2`). Switch `lsm.begin_I2C()` to `lsm.begin_SPI(...)` (using `Adafruit_LSM6DSM`).
- **Pin Definitions:** Verify and update the `#define` pin numbers in `main.ino` to match the final routed nets in `microcontroller.kicad_sch` (`QTR_ARR`, `LASER_I2C`, `MOTOR_R`, `pwm_left`, `dir1_left`, `dir2_left`, etc.).
- **Camera & Stoplight Integration:** Add UART parsing for the `camera` co-processor AprilTag packets and BLE/ESP-NOW reception for the `autonomous-stoplight` state.

## Git workflow

Read [CONTRIBUTING.md](CONTRIBUTING.md) for our team Git rules:
1. **Always Pull before you start** and **Push when you finish** in GitHub Desktop.
2. **KiCad:** Only one person edits the PCB at a time—claim it in `#watt-city` on Slack first.
3. **Branches:** Pushing straight to `main` is fine for `sensor_tests/`, `hardware/`, and docs. Use a branch when experimenting on `firmware/src/main/main.ino` so `main` always stays drivable.

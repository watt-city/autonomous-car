# WATT City: Autonomous Car

Custom PCB hardware, PID line-following firmware, and sensor bring-up tests for the WATT City autonomous car: an **ESP32-S3-WROOM-1-N8** vehicle equipped with a **QTR-8RC** reflectance line array, **VL53L0X** time-of-flight obstacle sensor, **LSM6DS3TR-C** 6-DoF IMU, **TB6612FNG** dual motor driver with quadrature encoders, and **BLE** telemetry.

**Owned by:** Car team · **Leads:** Alisa Zhang, Katie Tao · **Questions:** `#watt-city` on WATT Slack

Part of [WATT City](https://github.com/watt-city), IEEE WATT's year-long project at UCLA. Related repos: [`camera`](https://github.com/watt-city/camera), [`autonomous-stoplight`](https://github.com/watt-city/autonomous-stoplight).

---

## Repo layout

```
hardware/                              ← KiCad 10.0 project for the custom Car PCB
  watt_city_car_pcb.kicad_pro          ← Main KiCad project file
  watt_city_car_pcb.kicad_sch          ← Top-level hierarchical schematic
  watt_city_car_pcb.kicad_pcb          ← PCB layout
  microcontroller.kicad_sch            ← ESP32-S3-WROOM-1-N8 + Boot/Reset buttons
  power.kicad_sch                      ← Power architecture (LDO, USB-C, protection)
    ldo.kicad_sch                      ← AP2112K-3.3 regulator
    powerProtection.kicad_sch          ← Reverse-polarity / diode protection
    usb.kicad_sch                      ← USB-C connector + ESD protection
  motorController.kicad_sch            ← TB6612FNG driver + left/right motor connectors
  distSensor.kicad_sch                 ← VL53L0X ToF distance sensor header
  irArray.kicad_sch                    ← QTR-8RC 8-channel IR line sensor header
  imu.kicad_sch                        ← On-board IMU schematic sheet
firmware/                              ← Integrated car firmware
  src/main/main.ino                    ← CURRENT: PID line-following, ToF obstacle stop, encoders & BLE telemetry
sensor_tests/                          ← Standalone Arduino bring-up & calibration sketches
  i2c_scanner/                         ← Pings I2C bus (SDA=8, SCL=9) to verify sensor addresses
  imu_lsm6ds3trc_test/                 ← Reads 3-axis accel + 3-axis gyro from LSM6DS3TR-C (0x6A)
  qtr_sensor_calibration/              ← 5-second sweep calibration; prints savedMin/savedMax arrays
  qtr_sensor_test/                     ← Live calibrated QTR-8RC readout (0 = white, 1000 = black)
  vl53l0x_test/                        ← VL53L0X distance readout (0x29) with -17 mm offset calibration
  sensors_all_test/                    ← Combined QTR + IMU + ToF bench test streaming over BLE UART
```

## Setup

**Arduino IDE**
1. Install the **esp32 by Espressif** board package.
2. Install the required libraries via *Tools → Manage Libraries*:
   - **QTRSensors** by Pololu
   - **Adafruit LSM6DS** by Adafruit (plus Adafruit Unified Sensor & BusIO dependencies)
   - **Adafruit VL53L0X** by Adafruit
3. Board settings: **ESP32S3 Dev Module** (or matching ESP32-S3 board), **USB CDC On Boot: Enabled**, CPU 240 MHz.

**KiCad**
- Open `hardware/watt_city_car_pcb.kicad_pro` in **KiCad 10.0+**.
- **Only one person edits the KiCad project at a time** (KiCad files cannot merge). Send a message in `#watt-city` on Slack before opening KiCad, and push as soon as you finish.

## Pinout (`ESP32-S3`)

| Subsystem | Signal | GPIO | Notes |
|---|---|---|---|
| **I2C Bus** | `I2C_SDA` / `I2C_SCL` | `GPIO 8` / `GPIO 9` | Shared by IMU (`0x6A`) and VL53L0X (`0x29`) |
| **VL53L0X ToF** | `VL53_XSHUT` | `GPIO 12` | Active-low hardware reset / shutdown |
| **QTR-8RC Array** | `QTR_CTRL` (Emitter) | `GPIO 11` | IR LED emitter control |
| | Sensors `1..8` | `GPIO 1, 2, 3, 4, 5, 6, 7, 10` | RC reflectance inputs |
| **Left Motor (A)** | `LEFT_PWMA`, `AIN1`, `AIN2` | `GPIO 15`, `GPIO 16`, `GPIO 17` | TB6612FNG Channel A |
| | `LEFT_ENC_A`, `LEFT_ENC_B` | `GPIO 13`, `GPIO 14` | Left quadrature encoder |
| **Right Motor (B)** | `RIGHT_PWMB`, `BIN1`, `BIN2` | `GPIO 18`, `GPIO 39`, `GPIO 40` | TB6612FNG Channel B |
| | `RIGHT_ENC_A`, `RIGHT_ENC_B` | `GPIO 21`, `GPIO 38` | Right quadrature encoder (inverted in ISR) |

## Using the sketches

### `firmware/src/main`: PID autonomous line follower (current)
- Runs the closed-loop PID controller (`Kp = 0.045`, `Ki = 0.0`, `Kd = 0.380`, `baseSpeed = 160`) tracking a black line via `qtr.readLineBlack()` (`0` to `7000`, center at `3500`).
- **Safety gate:** Halts both motors immediately if the VL53L0X detects an obstacle closer than `MIN_OBSTACLE_DIST_MM` (`80 mm`).
- **BLE Telemetry:** Advertises as `ESP32S3_LineFollower` using the Nordic UART Service and streams CSV lines (`Pos,Dist,GyroZ,LTicks,RTicks`) at 20 Hz.

### `sensor_tests/` workflow
1. **`i2c_scanner`**: Run first on new hardware to confirm the IMU (`0x6A`) and VL53L0X (`0x29`) respond on `GPIO 8` / `GPIO 9`.
2. **`qtr_sensor_calibration`**: Sweep all 8 IR sensors across the black tape and white surface for 5 seconds. Copy the printed `savedMin` and `savedMax` arrays into `qtr_sensor_test` and `firmware/src/main/main.ino`.
3. **`vl53l0x_test` & `imu_lsm6ds3trc_test`**: Verify individual distance and gyro/accelerometer readings over USB Serial (`115200` baud).
4. **`sensors_all_test`**: Full sensor suite burn-in over BLE (`ESP32S3_Car`) with automatic VL53L0X watchdog reset if the I2C bus hangs.

## Known issues / open decisions

- **Camera UART integration pending.** `firmware/src/main/main.ino` does not yet parse incoming AprilTag packets from the `camera` co-processor over UART. As noted in the [`camera` README](https://github.com/watt-city/camera), the Car and Camera teams need to pick between text `TAG,...` lines and the binary `VisionPacket` format.
- **Stoplight wireless link.** Stoplight state reception (BLE / ESP-NOW) needs to be integrated into the main control loop alongside the VL53L0X obstacle stop.
- **Legacy schematic sheets.** `hardware/` contains standalone sheets (`laserSensor*.kicad_sch`, `motorEncoders.kicad_sch`) from earlier design iterations; check `watt_city_car_pcb.kicad_sch` for the active sheet hierarchy.

## Git workflow

Read [CONTRIBUTING.md](CONTRIBUTING.md) for our team Git rules:
1. **Always Pull before you start** and **Push when you finish** in GitHub Desktop.
2. **KiCad:** Only one person edits the PCB at a time—claim it in `#watt-city` on Slack first.
3. **Branches:** Pushing straight to `main` is fine for `sensor_tests/`, `hardware/`, and docs. Use a branch when experimenting on `firmware/src/main/main.ino` so `main` always stays drivable.

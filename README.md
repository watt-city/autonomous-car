# WATT City: Autonomous Car

Firmware, sensor tests, and PCB design for the WATT City self-driving car: an ESP32-S3 robot that follows a black line, slows or stops for obstacles, reads AprilTags at intersections, and obeys traffic-light messages from the stoplight over ESP-NOW.

> **Current hardware status:** every sketch in `firmware/` and `sensor_tests/` targets the **ESP32-S3 DevKit + breakout prototype** (LSM6DS3TR-C I2C breakout, VL53L0X breakout, Pololu QTR-MD-08RC, TB6612FNG breakout). The custom PCB in `hardware/` uses a bare **ESP32-S3-WROOM-1-N8** with an onboard **LSM6DSM** IMU over SPI; pins and IMU setup change when the PCB is brought up (see the end of this file).

**Owned by:** Car team · **Leads:** Alisa Zhang, Katie Tao · **Questions:** `#watt-city` on WATT Slack

Part of [WATT City](https://github.com/watt-city), IEEE WATT's year-long project at UCLA. Related repos: [`camera`](https://github.com/watt-city/camera), [`autonomous-stoplight`](https://github.com/watt-city/autonomous-stoplight).

---

## Repo layout

```
firmware/
  src/main/main.ino  ← PID line follower + ToF obstacle stop + encoder & BLE telemetry (single loop, PWM out)
  pid_line/          ← Cascaded PID line follower: line PD → per-wheel speed PI on the encoders, live serial tuning
  bringup/           ← Staged hardware check: I2C, QTR, each motor, encoders. Run this first on a new build
  main_board/        ← Integration draft: line follow + ToF + IMU + camera UART + ESP-NOW (PLACEHOLDER PINS)
    watt_link.h      ← Message formats + TAG line parser (camera → car UART text, stoplight → car ESP-NOW)
sensor_tests/        ← One small sketch per sensor, for isolating problems
  i2c_scanner/            ← Pings I2C (SDA=8, SCL=9) to confirm the IMU (0x6A) and VL53L0X (0x29)
  imu_lsm6ds3trc_test/    ← 3-axis accel + gyro from the LSM6DS3TR-C breakout
  qtr_sensor_calibration/ ← 5 s sweep calibration; prints savedMin/savedMax arrays
  qtr_sensor_test/        ← Live calibrated QTR readout (0 = white, 1000 = black)
  vl53l0x_test/           ← VL53L0X distance readout with −17 mm offset
  sensors_all_test/       ← QTR + IMU + ToF together, streamed over BLE UART
  motor_encoder_test/     ← Motors + quadrature encoders
hardware/            ← KiCad 10 project for the car PCB (hierarchical schematic + layout)
docs/
  test_track_letter_tiled.pdf      ← Printable line-follow track, letter pages
  apriltags_36h11_ids0-5_50mm.pdf  ← Printable AprilTags, IDs 0–5 (37 mm black square)
```

**Two line-follower sketches exist on purpose for now.** `src/main/main.ino` has the ToF stop and BLE telemetry; `pid_line` has the fixed-rate cascaded controller. The team will pick one to build on and port the other's features in. Until then, say which one you tested in the hardware log.

## Setup

**Arduino IDE**
1. Install the **esp32 by Espressif** board package, **v3.x** (the code uses the v3 `ledcAttach` API).
2. Libraries (*Tools → Manage Libraries*): **QTRSensors** (Pololu), **Adafruit LSM6DS** (+ Unified Sensor & BusIO), **Adafruit VL53L0X**.
3. Board settings: **ESP32S3 Dev Module**, **USB CDC On Boot: Enabled**, Flash Size **8MB (64Mb)**.
4. **Put the car on a stand (wheels off the table)** before any motor test.

**KiCad**
- Open `hardware/watt_city_car_pcb.kicad_pro` in **KiCad 10.0+**.
- **Only one person edits the KiCad project at a time** (KiCad files can't merge). Claim it in `#watt-city` before opening KiCad, and push as soon as you finish.

## DevKit prototype pinout

| Subsystem | Signal | DevKit GPIO | Notes |
|---|---|---|---|
| **I2C bus** | `SDA` / `SCL` | `8` / `9` | LSM6DS3TR-C (`0x6A`) and VL53L0X (`0x29`). `bringup.ino` notes the log book once had these swapped: if the I2C scan finds nothing, swap them |
| **VL53L0X** | `XSHUT` | `12` | Must be HIGH or the sensor stays off |
| **QTR array** | `CTRL` (emitters) | `11` | |
| | Sensors 1–8 | `1, 2, 3, 4, 5, 6, 7, 10` | |
| **Left motor (A)** | `PWMA`, `AIN1`, `AIN2` | `15`, `16`, `17` | TB6612FNG channel A |
| | Encoder A / B | `13` / `14` | |
| **Right motor (B)** | `PWMB`, `BIN1`, `BIN2` | `18`, `39`, `40` | Not 19/20: those are USB D−/D+ |
| | Encoder | `21`, `38` | `main.ino` uses A=21, B=38 and inverts in the ISR; `pid_line`/`bringup` use A=38, B=21. Both make forward count positive |

> ⚠️ `main_board/main_board.ino` still uses **placeholder pins** that don't match this table. Update it before flashing.

## Using the sketches

### `firmware/bringup`: check the hardware first
Serial Monitor at **115200**: `i` I2C scan · `q` QTR raw · `c` QTR calibrate · `l`/`r` left/right motor · `e` encoders · `z` zero · `s` stop

### `firmware/src/main`: PID line follower with obstacle stop
- Single-loop PID (`Kp = 0.045`, `Ki = 0`, `Kd = 0.380`, `baseSpeed = 160` PWM, max 220) on `qtr.readLineBlack()` (0–7000, center 3500).
- **Safety gate:** stops both motors if the VL53L0X sees something closer than `MIN_OBSTACLE_DIST_MM` (80 mm).
- **BLE telemetry:** advertises as `ESP32S3_LineFollower` (Nordic UART Service), CSV `Pos,Dist,GyroZ,LTicks,RTicks` at 20 Hz.
- Calibration values are hard-coded in `setup()`: re-run `sensor_tests/qtr_sensor_calibration` and paste new `savedMin`/`savedMax` if the track or lighting changes.

### `firmware/pid_line`: cascaded PID with live tuning
Serial Monitor at 115200, line ending **New Line**:
- `cal`: sweep the array across the line for 5 s · `go`: start · `s`: stop
- `v <cps>`: both wheels to a speed (wheels off the ground) to tune the speed loop
- `base <cps>`, `lkp <x>`, `lkd <x>`, `kff`/`skp`/`ski <x>`: tune live · `p`: print gains · `sign`: flip steering
- Output is `label:value` pairs, so **Tools → Serial Plotter** graphs it.

**Tuning status:** the Sep 27 commit notes `base 2000` and `lkp 0.5` worked on the track; the defaults in code are still `base 600` and `lkp 0.30`. Set them over Serial, and update the defaults once confirmed.

### `sensor_tests/` workflow
1. `i2c_scanner` first when wiring breakouts.
2. `qtr_sensor_calibration`: sweep for 5 s, then copy the printed arrays into `qtr_sensor_test` and `firmware/src/main/main.ino`.
3. `vl53l0x_test`, `imu_lsm6ds3trc_test`, `motor_encoder_test`: one subsystem at a time over USB Serial (115200).
4. `sensors_all_test`: full breakout burn-in over BLE (`ESP32S3_Car`), with a VL53L0X watchdog reset if I2C hangs.

## How the car talks to everything else

`firmware/main_board/watt_link.h` defines both links:

- **Camera → car (UART text, 115200):** one line per frame from `camera/firmware/xiao_apriltag_largest`:
  `TAG,<id>,<center_x>,<center_y>,<side_px>,<decision_margin>` or `TAG,-1` (no tag). `watt_parse_tag_line()` parses it. Because a line arrives every frame, no line for 300 ms means the camera link is down.
- **Stoplight → car (ESP-NOW):** 3-byte `V2XPacket` (intersection ID, light state, seconds to change). The intersection ID **must match the AprilTag ID** at that intersection.

**Red-light stop** (`main_board`): the car stops once the matching tag's `side_px` reaches `TAG_STOP_SIDE_PX`. The current value (26 px) is a pinhole **estimate** for ~0.35 m at QVGA; measure it by holding a tag at the stop distance and reading `side=` from the camera's USB bench summary.

## PCB (`hardware/`)

KiCad **10** project `watt_city_car_pcb`, hierarchical: power, power protection, LDO (AP2112K-3.3), USB-C, microcontroller (ESP32-S3-WROOM-1-N8), motor controller (TB6612FNG), motor encoders, IR array connector, VL53L0X laser-distance sheets, and IMU (LSM6DSM).

- Don't commit fab outputs on every change. Tag the commit you send to fab and attach the Gerbers to a GitHub Release.

### Moving from DevKit breakouts to the custom PCB
- **IMU (I2C → SPI):** the breakout is an `LSM6DS3TR-C` on I2C (`0x6A`); the PCB has an **`LSM6DSM`** on **SPI** (`IMU_SPI` SCK/MOSI/MISO/CS + `IMU_INT1`/`INT2`). Switch `lsm.begin_I2C()` to `begin_SPI(...)` with the matching driver class.
- **Pins:** update every `#define` to the routed nets in `microcontroller.kicad_sch`.
- **Integration:** add UART parsing for the camera and ESP-NOW reception for the stoplight (see `main_board/` for a starting point).

## Contributing

Read [CONTRIBUTING.md](CONTRIBUTING.md) before your first push. In short: pull before you start, push when you finish; push straight to `main` for `sensor_tests/`, `hardware/` (after claiming it) and docs; **use a branch for anything in `firmware/`**.

## History

`firmware/bringup`, `pid_line`, `main_board`, and the `docs/` PDFs came from the former `watt-city/car-code` repo (now archived) and were merged here with their commit history (`git log --follow firmware/pid_line/pid_line.ino`).

# WATT City: Autonomous Car

Firmware, sensor tests, and PCB design for the WATT City self-driving car: an ESP32-S3 robot that follows a black line, slows or stops for obstacles, reads AprilTags at intersections, and obeys traffic-light messages from the stoplight over ESP-NOW.

**Owned by:** Car team · **Leads:** Alisa Zhang, Katie Tao · **Questions:** `#watt-city` on WATT Slack

Part of [WATT City](https://github.com/watt-city), IEEE WATT's year-long project at UCLA. Related repos: [`autonomous-stoplight`](https://github.com/watt-city/autonomous-stoplight), [`camera`](https://github.com/watt-city/camera).

---

## Repo layout

```
firmware/
  pid_line/          ← CURRENT: cascaded PID line follower (encoders + QTR). Start here.
  bringup/           ← staged hardware check: I2C, QTR, each motor, encoders
  main_board/        ← integrated firmware draft: line follow + ToF + IMU + camera UART + ESP-NOW
    watt_link.h      ← shared message format (camera → car UART, stoplight → car ESP-NOW)
sensor_tests/        ← one small sketch per sensor, for isolating problems
  i2c_scanner/  imu_lsm6ds3trc_test/  vl53l0x_test/
  qtr_sensor_calibration/  qtr_sensor_test/  sensors_all_test/  motor_encoder_test/
hardware/            ← KiCad 10 project for the car PCB (hierarchical schematic + layout)
docs/
  test_track_letter_tiled.pdf      ← printable line-follow track, 12 letter pages
  apriltags_36h11_ids0-5_50mm.pdf  ← printable AprilTags, IDs 0–5, 50 mm
```

## Hardware (breadboard prototype)

| Part | What it does |
|---|---|
| ESP32-S3-DevKitC-1 | Main controller |
| Pololu 3pi+ chassis, 100:1 HPCB gearmotors, 12 CPR encoders | Drive + wheel speed |
| TB6612FNG | Dual motor driver |
| QTR-MD-08RC | 8-sensor reflectance array for line following |
| VL53L0X | Forward time-of-flight distance (obstacles) |
| Adafruit 4503 (LSM6DS3TR-C) | IMU breakout. **Note:** the PCB uses an LSM6DSM, which needs a different driver |
| Seeed XIAO ESP32S3 Sense | Camera co-processor (code lives in the [`camera`](https://github.com/watt-city/camera) repo) |

### Pin map (verified in bring-up)

These are the pins used by `bringup`, `pid_line`, and `sensor_tests`. Change them here **and** in the hardware log if the wiring changes.

| Signal | GPIO | | Signal | GPIO |
|---|---|---|---|---|
| PWMA (left) | 15 | | ENC_L_A / ENC_L_B | 13 / 14 |
| AIN1 / AIN2 | 16 / 17 | | ENC_R_A / ENC_R_B | 38 / 21 (swapped so forward counts +) |
| PWMB (right) | 18 | | QTR OUT (8 sensors) | 1, 2, 3, 4, 5, 6, 7, 10 |
| BIN1 / BIN2 | 39 / 40 (not 19/20: those are USB D−/D+) | | QTR CTRL (emitters) | 11 |
| I2C SDA / SCL | 8 / 9 | | VL53L0X XSHUT | 12 (must be HIGH) |

> ⚠️ `main_board/main_board.ino` still uses **placeholder pins** that don't match this table. Update it before flashing.
> `bringup.ino` notes the log book disagrees on SDA/SCL (8/9 vs 9/8). If the I2C scan finds nothing, swap them.

## Getting started

1. **Install** the [Arduino IDE](https://www.arduino.cc/en/software) and the **esp32 by Espressif** board package, **v3.x** (the code uses the v3 `ledcAttach` API).
2. **Libraries** (Library Manager): `QTRSensors` (Pololu), `Adafruit VL53L0X`, `Adafruit LSM6DS`.
3. **Board settings:** `ESP32S3 Dev Module`, **USB CDC On Boot: Enabled**.
4. **Put the car on a stand (wheels off the table)** before any motor test.
5. Flash `firmware/bringup`, open Serial Monitor at **115200**, and check each subsystem:
   `i` I2C scan · `q` QTR raw · `c` QTR calibrate · `l`/`r` left/right motor · `e` encoders · `z` zero · `s` stop
6. Flash `firmware/pid_line`. Serial Monitor at 115200, line ending **New Line**:
   - `cal`: sweep the array across the line for 5 s
   - `go`: start line following · `s`: stop
   - `base <cps>`, `lkp <x>`, `lkd <x>`, `kff`/`skp`/`ski <x>`: tune live · `p`: print gains · `sign`: flip steering
   - Output is `label:value` pairs, so **Tools → Serial Plotter** graphs it.

**Tuning status:** the last commit notes `base 2000` and `lkp 0.5` worked on the track. The defaults in code are still `base 600` and `lkp 0.30`, so set them over Serial (or update the defaults in a PR once confirmed).

## How the car talks to everything else

`firmware/main_board/watt_link.h` defines both message formats:

- **Camera → car (UART, 115200):** binary frame `[0xAA][0x55][TYPE][LEN][payload][XOR checksum]` carrying a `VisionPacket` (tag ID, range, bearing, obstacle flag, frame counter).
- **Stoplight → car (ESP-NOW):** 3-byte `V2XPacket` (intersection ID, light state, seconds to change). The intersection ID **must match the AprilTag ID** at that intersection; that's how the car knows which light applies to it.

> ⚠️ **Known mismatch:** the newest camera sketch (`camera/firmware/xiao_apriltag_largest`) sends **text** lines (`TAG,<id>,...`), not this binary format. The camera repo's `vision_node_watt_link` sketch does speak this format. The Camera and Car teams need to agree on one before integration.

## PCB (`hardware/`)

KiCad **10** project `watt_city_car_pcb`, hierarchical: power, power protection, LDO (AP2112K-3.3), USB-C, microcontroller (ESP32-S3-WROOM-1-N8), motor controller (TB6612FNG), motor encoders, IR array connector, VL53L0X laser-distance sheets, and IMU (LSM6DSM).

- Opening it requires KiCad 10 or newer.
- **Only one person edits the PCB at a time.** Claim it with a GitHub issue first; KiCad files don't merge.
- Don't commit fab outputs on every change. Tag the commit you send to fab and attach the Gerbers to a GitHub Release.

## Contributing

Read [CONTRIBUTING.md](CONTRIBUTING.md) before your first push. In short: branch off `main`, open a pull request, get one review, and log every hardware test in the hardware log.

## History

The car firmware in `firmware/bringup`, `pid_line`, and `main_board` was developed in the former `watt-city/car-code` repo and merged here with its commit history (`git log --follow firmware/pid_line/pid_line.ino`).

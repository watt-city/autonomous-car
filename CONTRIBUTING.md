# WATT City Git Guide

A quick, practical guide so our team can share code and KiCad files without overwriting each other's work. If you get stuck with Git, ask in `#watt-city` on Slack!

---

## The 3 Golden Rules

### 1. Always Pull Before You Start, Push When You Finish
90% of Git conflicts happen because someone started editing old files.
- **Before opening KiCad or Arduino IDE:** Open **GitHub Desktop** and click **Fetch origin** (and **Pull origin** if there are updates).
- **As soon as you finish a session:** Write a clear summary (e.g., *"Tune PID Kd and add VL53L0X stop threshold"*), click **Commit**, and click **Push origin**.

### 2. Only One Person Edits a KiCad PCB at a Time
KiCad schematic (`.kicad_sch`) and board (`.kicad_pcb`) files **cannot be merged** if two people edit them at the same time.
- Before opening KiCad to make changes, send a quick message in `#watt-city` on Slack: *"Editing the Car PCB right now!"*
- Pull the latest changes, do your KiCad work, commit, push, and let the channel know you're done: *"Pushed Car PCB!"*

### 3. Pushing to `main` vs. Making a Branch
- **Just push straight to `main`** when you are:
  - Adding or editing standalone sketches in `sensor_tests/`
  - Updating `hardware/` (after claiming the PCB on Slack)
  - Updating READMEs or documentation
- **Make a branch** when you are experimenting on the shared `firmware/src/main/main.ino`:
  - Treat `main.ino` on the `main` branch as **working code that drives the car**.
  - If you are testing a big change to `main.ino` (like adding Bluetooth/ESP-NOW or rewriting the PID loop), click **Current Branch → New Branch** in GitHub Desktop (e.g., `feat/espnow`).
  - Once you've tested it on the physical car and confirmed it works, merge your branch back into `main` (or open a quick Pull Request on GitHub).

---

## Housekeeping Rules

- **Arduino folder rule:** Every `.ino` file must live inside a folder with the **exact same name** (e.g., `vl53l0x_test/vl53l0x_test.ino`), otherwise Arduino IDE won't open it.
- **No WiFi passwords or keys:** These repos are public! If a sketch uses WiFi (like camera calibration), put credentials in `secrets.h` (which is ignored by `.gitignore`).
- **Keep large files in Google Drive:** Put photos, videos, and datasheets in the shared WATT folder rather than committing them to Git.

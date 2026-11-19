/*
 * ble_stoplight_rx.ino  —  ESP32 BLE receiver for stoplight broadcasts
 *
 * Passively scans for advertisements from ble_stoplight_tx
 * (autonomous-stoplight repo), prints each state
 * change, and mirrors the colour on the onboard RGB LED (if it has one).
 * Handles multiple stoplights (IDs 0–15) at once.
 *
 * Works on any ESP32 with BLE (S3, C3, classic ESP32...).
 * Builds on Arduino-ESP32 core 3.x or 4.x.
 *
 * NOTE: BLE prototype for the DevKitC-1 onboard RGB LED. This is NOT the team's
 * ESP-NOW V2XPacket link defined in autonomous-car/firmware/main_board/watt_link.h.
 * Not yet compiled or tested on hardware.
 */

#include <Arduino.h>
#if ESP_ARDUINO_VERSION_MAJOR >= 4
  #include <BLE.h>
#else
  #include <BLEDevice.h>
  #include <BLEScan.h>
  #include <BLEAdvertisedDevice.h>
#endif

// ---------------- user settings ----------------
#define HAS_RGB_LED  1        // set 0 if your receiver board has no WS2812
#define RGB_PIN      38       // board silkscreen "RGB@IO38" = 38; some DevKitC-1 v1.0 boards use 48
#define BRIGHTNESS   40
#define MAX_LIGHTS   16
#define COMPANY_ID   0xFFFF   // must match the transmitter
// ------------------------------------------------

enum LightState { UNKNOWN, YELLOW, RED, GREEN };
LightState lastState[MAX_LIGHTS];   // de-dupes: we hear each state ~25x/second

// ======= put your own logic here (e.g. tell the car to stop on red) =======
void onStoplightChange(int id, LightState s, int rssi) {
#if HAS_RGB_LED
  switch (s) {
    case GREEN:  rgbLedWrite(RGB_PIN, 0, BRIGHTNESS, 0); break;
    case YELLOW: rgbLedWrite(RGB_PIN, BRIGHTNESS, BRIGHTNESS * 55 / 100, 0); break;
    case RED:    rgbLedWrite(RGB_PIN, BRIGHTNESS, 0, 0); break;
    default: break;
  }
#endif
}
// ==========================================================================

LightState parseColour(const char* c) {
  if (!strcmp(c, "green"))  return GREEN;
  if (!strcmp(c, "yellow")) return YELLOW;
  if (!strcmp(c, "red"))    return RED;
  return UNKNOWN;
}

// Called for every advertisement that carries our company ID.
// text/len = the bytes after the company ID, e.g. "stoplight 1 - green"
void handlePayload(const uint8_t* text, size_t len, const String& addr, int rssi) {
  char msg[32];
  if (len == 0 || len >= sizeof(msg)) return;
  memcpy(msg, text, len);
  msg[len] = '\0';

  int id = -1;
  char colour[12] = {0};
  if (sscanf(msg, "stoplight %d - %11s", &id, colour) != 2) return;
  if (id < 0 || id >= MAX_LIGHTS) return;

  LightState s = parseColour(colour);
  if (s == UNKNOWN || s == lastState[id]) return;   // only act on changes
  lastState[id] = s;

  Serial.printf("[RX] %s   (from %s, RSSI %d dBm)\n", msg, addr.c_str(), rssi);
  onStoplightChange(id, s, rssi);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 4
// ---------------- core 4.x ----------------
BLEScan scan;

void onAdv(const BLEAdvertisedDevice& dev) {
  if (!dev.haveManufacturerData()) return;
  if (dev.getManufacturerCompanyId() != COMPANY_ID) return;
  size_t len = 0;
  const uint8_t* mfg = dev.getManufacturerData(&len);   // includes 2-byte company ID
  if (!mfg || len < 3) return;
  handlePayload(mfg + 2, len - 2, dev.getAddress().toString(), dev.getRSSI());
}

void setupBLE() {
  BTStatus st = BLE.begin("Stoplight-RX");
  if (!st) {
    Serial.printf("BLE init failed: %s\n", st.toString());
    while (true) delay(1000);
  }
  scan = BLE.getScan();
  scan.setActiveScan(false);        // passive: the advert packet is all we need
  scan.setInterval(100);
  scan.setWindow(99);               // ≈ continuous listening
  scan.setFilterDuplicates(false);  // we de-dupe ourselves
  scan.onResult(onAdv);
}

void scanOnce() {
  scan.startBlocking(1000);         // callbacks fire during the scan
  scan.clearResults();              // don't let every nearby phone pile up in RAM
}

#else
// ---------------- core 3.x ----------------
BLEScan* scan = nullptr;

class StoplightCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    if (!dev.haveManufacturerData()) return;
    String mfg = dev.getManufacturerData();             // includes 2-byte company ID
    if (mfg.length() < 3) return;
    uint16_t cid = (uint8_t)mfg[0] | ((uint8_t)mfg[1] << 8);
    if (cid != COMPANY_ID) return;
    handlePayload((const uint8_t*)mfg.c_str() + 2, mfg.length() - 2,
                  dev.getAddress().toString(), dev.getRSSI());
  }
};

void setupBLE() {
  BLEDevice::init("Stoplight-RX");
  scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new StoplightCallbacks(), true); // true = report repeats
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(99);
}

void scanOnce() {
  scan->start(1, false);            // 1 s blocking scan
  scan->clearResults();
}
#endif

void setup() {
  Serial.begin(115200);
  delay(300);
  for (int i = 0; i < MAX_LIGHTS; i++) lastState[i] = UNKNOWN;
  setupBLE();
  Serial.printf("Listening for stoplights... (core %s)\n", ESP_ARDUINO_VERSION_STR);
}

void loop() {
  scanOnce();
}

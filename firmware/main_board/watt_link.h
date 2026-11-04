// watt_link.h  --  shared message definitions for WATT City
//
// Two links reach the car's main board:
//   1) XIAO camera --UART text--> main board   (one TAG line per frame)
//   2) stoplight   --ESP-NOW-->   main board   (V2XPacket, traffic-light state)
//
// Copy this file into any sketch that needs it (Arduino only includes headers
// from the sketch's own folder). Format changes need Car + Camera + Stoplight
// agreement.

#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// =====================================================================
// 1) Camera -> car: text over UART, 115200 8N1, one line per frame.
//    Sent by camera/firmware/xiao_apriltag_largest (XIAO TX = D6/GPIO43).
//
//      TAG,<id>,<center_x>,<center_y>,<side_px>,<decision_margin>\n
//      TAG,-1\n                                   <- no tag this frame
//
//    id       : AprilTag 36h11 ID of the LARGEST (nearest) tag in view
//    center   : tag center in pixels (QVGA: 0..319, 0..239)
//    side_px  : sqrt(tag area) in pixels; bigger = closer
//    margin   : decode confidence; the camera already drops < 30
//    An optional 7th field (z_m) appears only if the camera enables pose;
//    it is ignored here.
// =====================================================================
struct VisionReading {
  int16_t tagId;      // -1 = no tag
  float   centerX;    // px
  float   centerY;    // px
  float   sidePx;     // px, larger = closer
  float   margin;     // decision margin
};

// Parse one line (without the trailing newline). Returns true and fills
// `out` on a valid TAG line; returns false and leaves `out` untouched otherwise.
static inline bool watt_parse_tag_line(const char* s, VisionReading& out) {
  if (strncmp(s, "TAG,", 4) != 0) return false;
  const char* p = s + 4;
  char* end;

  long id = strtol(p, &end, 10);
  if (end == p) return false;
  if (id == -1 && (*end == '\0' || *end == '\r')) {
    out = VisionReading{-1, 0.0f, 0.0f, 0.0f, 0.0f};
    return true;
  }
  if (id < 0 || id > 586 || *end != ',') return false;

  float v[4];
  p = end + 1;
  for (int i = 0; i < 4; i++) {
    v[i] = strtof(p, &end);
    if (end == p) return false;
    if (i < 3) {
      if (*end != ',') return false;
      p = end + 1;
    }
  }
  if (!(*end == '\0' || *end == '\r' || *end == ',')) return false;  // ',' = optional z_m

  out = VisionReading{(int16_t)id, v[0], v[1], v[2], v[3]};
  return true;
}

// =====================================================================
// 2) Stoplight -> car: ESP-NOW, 3-byte packet.
// =====================================================================
enum LightState : uint8_t { LIGHT_RED = 0, LIGHT_YELLOW = 1, LIGHT_GREEN = 2, LIGHT_UNKNOWN = 3 };

#pragma pack(push, 1)
struct V2XPacket {
  uint8_t  intersectionId;   // must match the AprilTag id at that intersection
  uint8_t  lightState;       // LightState
  uint8_t  secondsToChange;  // countdown hint, 0 if not provided
};
#pragma pack(pop)

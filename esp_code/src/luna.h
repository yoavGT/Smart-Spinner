#pragma once
#include <Arduino.h>

// ================= USER CONFIG (pins / defaults) =================
#ifndef ESP_LUNA_RX
#define ESP_LUNA_RX 22
#endif

#ifndef ESP_LUNA_TX
#define ESP_LUNA_TX 21
#endif

#ifndef LUNA_DEFAULT_BAUD
#define LUNA_DEFAULT_BAUD 115200
#endif

#ifndef LUNA_FRAME_RATE_HZ
#define LUNA_FRAME_RATE_HZ 250
#endif

#ifndef LUNA_NEAR_THRESHOLD_MM
#define LUNA_NEAR_THRESHOLD_MM 200
#endif

#ifndef LUNA_FAR_THRESHOLD_MM
#define LUNA_FAR_THRESHOLD_MM 900
#endif
// ================================================================

bool luna_init_mm(uint32_t baud = LUNA_DEFAULT_BAUD);
bool luna_read_mm_latest(uint16_t &distance_mm);

// ---------- Zero reference ("compass replacement") ----------
struct LunaZeroRef {
  uint16_t near_th_mm = LUNA_NEAR_THRESHOLD_MM;
  uint16_t far_th_mm  = LUNA_FAR_THRESHOLD_MM;

  uint8_t  hits_needed = 2;
  uint8_t  fars_needed = 2;

  bool armed = true;
  uint8_t near_hits = 0;
  uint8_t far_hits  = 0;

  float zero_offset_deg = 0.0f;
  bool  has_zero = false;
};

bool  luna_update_zero(LunaZeroRef &z, uint16_t dist_mm, float theta_deg);
float luna_angle_relative0(const LunaZeroRef &z, float theta_deg);
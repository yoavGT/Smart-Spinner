#pragma once
#include <Arduino.h>

// init POV renderer (uses leds.* internally)
void pov_init(const char* text,
              uint8_t fg_r, uint8_t fg_g, uint8_t fg_b,
              uint8_t bg_r, uint8_t bg_g, uint8_t bg_b,
              uint8_t global_brightness = 8);

// Update POV display.
// Call this ONLY when you have new luna frame (same place you print telemetry).
// heading_valid: false => show waiting indicator
void pov_update(bool heading_valid, float heading_deg);

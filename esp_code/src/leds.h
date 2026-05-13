#pragma once
#include <Arduino.h>

// Your wiring
#ifndef LED_CLK_PIN
#define LED_CLK_PIN 13
#endif

#ifndef LED_SDI1_PIN
#define LED_SDI1_PIN 25
#endif

#ifndef LED_SDI2_PIN
#define LED_SDI2_PIN 26
#endif

#ifndef LED_SDI3_PIN
#define LED_SDI3_PIN 27
#endif

#ifndef LED_SDI4_PIN
#define LED_SDI4_PIN 14
#endif

// Parallel LED arms (shared clock, per-arm data)
#ifndef LED_ARM_COUNT
#define LED_ARM_COUNT 4
#endif

#ifndef LEDS_PER_ARM
#define LEDS_PER_ARM 10
#endif

// LED strip configuration
#ifndef LED_COUNT
#define LED_COUNT (LED_ARM_COUNT * LEDS_PER_ARM)
#endif

// Init APA102 driver (DotStar).
// spi_hz: 1-12MHz usually safe for short wires; start with 8MHz.
// global_brightness: 0..31 (APA102 "global" brightness field). 31 = max.
void leds_init(uint32_t spi_hz = 8000000, uint8_t global_brightness = 31);

// Set one pixel RGB (0..255 each). Does NOT send; call leds_show().
void leds_set_rgb(uint16_t i, uint8_t r, uint8_t g, uint8_t b);

// Set all pixels RGB (0..255). Does NOT send; call leds_show().
void leds_set_all(uint8_t r, uint8_t g, uint8_t b);

// Clear buffer (sets all LEDs off). Does NOT send; call leds_show().
void leds_clear();

// Push current buffer to LEDs (fast).
void leds_show();

// Set global brightness for all LEDs (0..31). Does NOT send; call leds_show().
void leds_set_global_brightness(uint8_t b);

// Optional helpers
uint16_t leds_count();

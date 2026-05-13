#ifndef STATUS_LEDS_H
#define STATUS_LEDS_H

#include <Arduino.h>

// =============================================================================
// Pin Definitions (from schematic)
// =============================================================================
#define STATUS_LED1_PIN 32   // ESP_ST_LED1
#define STATUS_LED2_PIN 33   // ESP_ST_LED2

// =============================================================================
// LED Identifiers
// =============================================================================
enum StatusLED {
    LED1 = 0,
    LED2 = 1
};

// =============================================================================
// Function Declarations
// =============================================================================

// Initialize status LED pins
void statusLeds_init();

// Turn a specific LED on
void statusLed_on(StatusLED led);

// Turn a specific LED off
void statusLed_off(StatusLED led);

// Toggle a specific LED
void statusLed_toggle(StatusLED led);

// Set LED state (true = on, false = off)
void statusLed_set(StatusLED led, bool state);

// Turn both LEDs on
void statusLeds_allOn();

// Turn both LEDs off
void statusLeds_allOff();

// Start blinking a LED at specified interval (call statusLeds_update() in loop)
// intervalMs: blink period in milliseconds (on for half, off for half)
void statusLed_startBlink(StatusLED led, uint32_t intervalMs);

// Stop blinking a LED (leaves it in current state)
void statusLed_stopBlink(StatusLED led);

// Update function - call this in loop() to handle blinking
// Returns immediately if no LEDs are blinking
void statusLeds_update();

#endif // STATUS_LEDS_H


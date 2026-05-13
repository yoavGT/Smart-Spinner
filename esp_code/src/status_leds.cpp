#include "status_leds.h"

// =============================================================================
// Internal State
// =============================================================================
static const uint8_t ledPins[2] = {STATUS_LED1_PIN, STATUS_LED2_PIN};
static bool ledStates[2] = {false, false};
static bool blinking[2] = {false, false};
static uint32_t blinkIntervals[2] = {0, 0};
static uint32_t lastBlinkTimes[2] = {0, 0};

// =============================================================================
// Initialization
// =============================================================================
void statusLeds_init() {
    // Configure pins as outputs
    pinMode(STATUS_LED1_PIN, OUTPUT);
    pinMode(STATUS_LED2_PIN, OUTPUT);
    
    // Start with LEDs off
    statusLeds_allOff();
}

// =============================================================================
// Basic LED Control (Sink LEDs - active LOW, turn on when GPIO is LOW)
// =============================================================================
void statusLed_on(StatusLED led) {
    if (led > LED2) return;
    ledStates[led] = true;
    digitalWrite(ledPins[led], LOW);   // Sink LED: LOW = ON
}

void statusLed_off(StatusLED led) {
    if (led > LED2) return;
    ledStates[led] = false;
    digitalWrite(ledPins[led], HIGH);  // Sink LED: HIGH = OFF
}

void statusLed_toggle(StatusLED led) {
    if (led > LED2) return;
    ledStates[led] = !ledStates[led];
    digitalWrite(ledPins[led], ledStates[led] ? LOW : HIGH);  // Sink LED
}

void statusLed_set(StatusLED led, bool state) {
    if (state) {
        statusLed_on(led);
    } else {
        statusLed_off(led);
    }
}

void statusLeds_allOn() {
    statusLed_on(LED1);
    statusLed_on(LED2);
}

void statusLeds_allOff() {
    statusLed_off(LED1);
    statusLed_off(LED2);
}

// =============================================================================
// Blinking Control
// =============================================================================
void statusLed_startBlink(StatusLED led, uint32_t intervalMs) {
    if (led > LED2) return;
    blinking[led] = true;
    blinkIntervals[led] = intervalMs;
    lastBlinkTimes[led] = millis();
}

void statusLed_stopBlink(StatusLED led) {
    if (led > LED2) return;
    blinking[led] = false;
}

void statusLeds_update() {
    uint32_t now = millis();
    
    for (int i = 0; i < 2; i++) {
        if (blinking[i]) {
            // Check if it's time to toggle
            if (now - lastBlinkTimes[i] >= blinkIntervals[i] / 2) {
                statusLed_toggle((StatusLED)i);
                lastBlinkTimes[i] = now;
            }
        }
    }
}

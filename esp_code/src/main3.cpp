#include "leds.h"
#include "pov.h"
#include "status_leds.h"   // ===== ADDED =====

#include <Arduino.h>
#include "luna.h"
#include "jyro.h"

// ================= WIFI DEBUG SWITCH =================
#define WIFI_DEBUG true
// =====================================================

#if WIFI_DEBUG
#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h> // strlen
#endif

/****************************************************
 * ===== ADDED: WIFI / UDP TELEMETRY (Laptop <-> ESP32) =====
 * ESP32 creates a WiFi Access Point (AP). Connect your laptop to it.
 * Telemetry is sent via UDP broadcast on UDP_PORT.
 *
 * Laptop side: listen on UDP_PORT (e.g. Python/Netcat/Wireshark).
 ****************************************************/
#if WIFI_DEBUG
static const char* AP_SSID = "SpinTop";
static const char* AP_PASS = "12345678";     // >= 8 chars for WPA2
static constexpr uint16_t UDP_PORT = 4210;

static WiFiUDP udp;
static IPAddress udp_broadcast_ip(255, 255, 255, 255);

static void wifi_debug_init()
{
  WiFi.mode(WIFI_AP);

  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);

  Serial.println();
  Serial.println("=== WiFi Debug (AP mode) ===");
  Serial.print("softAP start: ");
  Serial.println(ok ? "OK" : "FAIL");

  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP IP:   ");
  Serial.println(ip);
  Serial.print("UDP port:");
  Serial.println(UDP_PORT);

  udp.begin(UDP_PORT);
}

static void wifi_debug_send(const char* msg)
{
  if (!msg) return;

  // UDP broadcast: any listener on the WiFi network gets it
  udp.beginPacket(udp_broadcast_ip, UDP_PORT);
  udp.write((const uint8_t*)msg, (int)strlen(msg));
  udp.endPacket();
}
#endif

/****************************************************
 * ===== ADDED: debug_print() =====
 * Prints to Serial always, and (if WIFI_DEBUG) also sends via UDP.
 ****************************************************/
static void debug_print(const char* msg)
{
  if (!msg) return;
  Serial.print(msg);
#if WIFI_DEBUG
  wifi_debug_send(msg);
#endif
}

// ================= LEDS & POV CONFIG =================

static constexpr uint32_t LED_SPI_HZ = 8000000;
static constexpr uint8_t  LED_GLOBAL_BRIGHTNESS = 8;

static const char* POV_TEXT = "HELLO";

static constexpr uint8_t POV_FG_R = 255;
static constexpr uint8_t POV_FG_G = 255;
static constexpr uint8_t POV_FG_B = 255;

static constexpr uint8_t POV_BG_R = 0;
static constexpr uint8_t POV_BG_G = 0;
static constexpr uint8_t POV_BG_B = 0;

// ================= SPIN CALIB CONFIG =================
#define CALIB_SPIN_NUM 3   // ===== ADDED =====
static constexpr uint32_t SPIN_CALIB_TIMEOUT_MS = 3000;
// ====================================================

// ================= STATUS LED BLINK =================
#define STATUS_LED_BLINK_ENABLE true
static constexpr uint32_t STATUS_LED_BLINK_MS = 500;
// ====================================================

// ================= LED CONNECTIVITY DEBUG =================
#define LED_DEBUG_TEST_ENABLE true
static constexpr uint16_t LUNA_NEAR_TEST_MM = 50;
static constexpr uint32_t LUNA_HOLD_MS = 2000;
static constexpr uint8_t LED_DEBUG_TOTAL = LEDS_PER_ARM;
// =========================================================

// ===== Debug print control =====
static constexpr uint32_t PRINT_PERIOD_MS = 100;
static uint32_t last_print_ms = 0;

// ===== Theta integration =====
static float theta_deg = 0.0f;
static uint32_t last_theta_us = 0;

// ===== Gyro bias calibration =====
static float gz_bias_dps = 0.0f;

// ===== Zero reference =====
static LunaZeroRef zref;

// ===== LED debug test state =====
static uint32_t luna_below_start_ms = 0;
static bool luna_below_latched = false;
static uint8_t debug_led_index = 0;

static void debug_led_color(uint8_t idx, uint8_t &r, uint8_t &g, uint8_t &b)
{
  // Simple repeating palette so each step is visibly different.
  static const uint8_t palette[][3] = {
    {255,   0,   0},
    {  0, 255,   0},
    {  0,   0, 255},
    {255, 255,   0},
    {  0, 255, 255},
    {255,   0, 255},
    {255, 127,   0},
    {127,   0, 255}
  };
  const uint8_t p = idx % (sizeof(palette) / sizeof(palette[0]));
  r = palette[p][0];
  g = palette[p][1];
  b = palette[p][2];
}

// ---- helper: update theta from gz (deg/s) ----
static void update_theta_from_gz(float gz_dps)
{
  uint32_t now = micros();
  if (last_theta_us == 0) {
    last_theta_us = now;
    return;
  }
  float dt = (now - last_theta_us) * 1e-6f;
  last_theta_us = now;
  theta_deg += gz_dps * dt;
}

// ---- helper: still calibration ----
static float calibrate_gz_bias(uint32_t duration_ms = 1500)
{
  const uint32_t start = millis();
  uint32_t count = 0;
  double acc = 0.0;

  while (millis() - start < duration_ms) {
    float gx, gy, gz;
    jyro_read_xyz_dps(gx, gy, gz);
    acc += gz;
    count++;
    delay(5);
  }
  return (count > 0) ? (float)(acc / count) : 0.0f;
}

/****************************************************
 * ===== ADDED: SPIN-BASED BIAS CALIBRATION =====
 * - Uses ZERO SET events (block detection)
 * - Prints spin count
 * - Yellow status LED ON during calibration
 ****************************************************/
static float calibrate_gz_bias_over_spins(uint8_t spins_needed)
{
  uint8_t spins_done = 0;
  double sum_gz = 0.0;
  uint32_t samples = 0;
  const uint32_t start_ms = millis();

  Serial.println("Spin calibration started");
  statusLeds_allOn();   // YELLOW ON

  while (spins_done < spins_needed) {
    if (SPIN_CALIB_TIMEOUT_MS > 0 && (millis() - start_ms) > SPIN_CALIB_TIMEOUT_MS) {
      Serial.println("Spin calibration timed out (no Luna data)");
      break;
    }

    float gx, gy, gz;
    jyro_read_xyz_dps(gx, gy, gz);
    update_theta_from_gz(gz);

    uint16_t d_mm;
    if (!luna_read_mm_latest(d_mm)) continue;

    if (luna_update_zero(zref, d_mm, theta_deg)) {
      spins_done++;
      Serial.print("Spin ");
      Serial.print(spins_done);
      Serial.print(" / ");
      Serial.println(spins_needed);
    }

    sum_gz += gz;
    samples++;
  }

  statusLeds_allOff();  // YELLOW OFF
  Serial.println("Spin calibration done");

  return (samples > 0) ? (float)(sum_gz / samples) : 0.0f;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  /*************** ADDED: WIFI INIT (no other changes) ***************/
#if WIFI_DEBUG
  wifi_debug_init();
#endif
  /*******************************************************************/

  // ===== ADDED =====
  statusLeds_init();
  statusLeds_allOff();
  // =================

#if STATUS_LED_BLINK_ENABLE
  // Blink both status LEDs to show main loop is alive.
  statusLed_startBlink(LED1, STATUS_LED_BLINK_MS);
  statusLed_startBlink(LED2, STATUS_LED_BLINK_MS);
#endif

  jyro_init();

  Serial.println("Hold still: calibrating gyro Z bias...");
  gz_bias_dps = calibrate_gz_bias(1500);

  Serial.print("gz_bias_dps (still) = ");
  Serial.println(gz_bias_dps, 4);

  luna_init_mm();

  zref.hits_needed = 2;
  zref.fars_needed = 2;

  leds_init(LED_SPI_HZ, LED_GLOBAL_BRIGHTNESS);
  pov_init(POV_TEXT,
           POV_FG_R, POV_FG_G, POV_FG_B,
           POV_BG_R, POV_BG_G, POV_BG_B,
           LED_GLOBAL_BRIGHTNESS);

  // ===== ADDED =====
  if (CALIB_SPIN_NUM > 0) {
    Serial.print("Spin the top for ");
    Serial.print(CALIB_SPIN_NUM);
    Serial.println(" revolutions...");
    gz_bias_dps = calibrate_gz_bias_over_spins(CALIB_SPIN_NUM);
    Serial.print("gz_bias_dps (FINAL) = ");
    Serial.println(gz_bias_dps, 4);
  }
  // =================

  Serial.println("System running");
}

/* ===================================================
 * =================== LOOP (UNCHANGED) ===================
 * =================================================== */
void loop() {
  // --- Read gyro ---
  float gx, gy, gz;
  jyro_read_xyz_dps(gx, gy, gz);

  // Correct bias
  float gz_corr = gz - gz_bias_dps;

  // Integrate to get theta
  update_theta_from_gz(gz_corr);

  // --- Read Luna (latest) ---
  uint16_t d_mm;
  if (luna_read_mm_latest(d_mm)) {

#if LED_DEBUG_TEST_ENABLE
    // LED connectivity test:
    // - Wait for Luna distance < LUNA_NEAR_TEST_MM continuously for LUNA_HOLD_MS.
    // - Each successful hold lights the next LED (up to LED_DEBUG_TOTAL).
    if (d_mm < LUNA_NEAR_TEST_MM) {
      if (luna_below_start_ms == 0) {
        luna_below_start_ms = millis();
        debug_print("Luna below test threshold (hold)...\n");
      }

      if (!luna_below_latched && (millis() - luna_below_start_ms) >= LUNA_HOLD_MS) {
        uint8_t r, g, b;
        debug_led_color(debug_led_index, r, g, b);

        // Turn off the previous LED across all arms.
        uint8_t prev_index = (debug_led_index == 0) ? (LEDS_PER_ARM - 1) : (debug_led_index - 1);
        for (uint8_t arm = 0; arm < LED_ARM_COUNT; arm++) {
          uint16_t off = arm * LEDS_PER_ARM + prev_index;
          leds_set_rgb(off, 0, 0, 0);
        }

        // Turn on the current LED index across all arms.
        for (uint8_t arm = 0; arm < LED_ARM_COUNT; arm++) {
          uint16_t off = arm * LEDS_PER_ARM + debug_led_index;
          leds_set_rgb(off, r, g, b);
        }
        leds_show();

        char buf[96];
        snprintf(buf, sizeof(buf),
           "LED test: index=%u across %u arms  color=(%u,%u,%u)\n",
           debug_led_index, LED_ARM_COUNT, r, g, b);
        debug_print(buf);

        debug_led_index++;
        if (debug_led_index >= LED_DEBUG_TOTAL) {
          debug_led_index = 0;
          debug_print("LED test: wrap to index 0 (cleared on next trigger)\n");
        }

        luna_below_latched = true;
      }
    } else {
      luna_below_start_ms = 0;
      luna_below_latched = false;
    }
#endif

    // Detect block pass → defines 0°
    if (luna_update_zero(zref, d_mm, theta_deg)) {
      debug_print("ZERO SET (block detected)\n");
    }

    /****************************************************
     * ================== ADDED: POV UPDATE (APA102) ==================
     * - Use the same heading you already compute from the zero-reference
     * - Update on EVERY fresh Luna frame (fast enough for POV)
     ****************************************************/
#if !LED_DEBUG_TEST_ENABLE
    float heading = luna_angle_relative0(zref, theta_deg);
    bool heading_valid = !isnan(heading);
    pov_update(heading_valid, heading);
#else
    float heading = NAN;
#endif

    // Debug print at low rate
    uint32_t now_ms = millis();
    if (now_ms - last_print_ms >= PRINT_PERIOD_MS) {
      last_print_ms = now_ms;

      float spin_dps = fabsf(gz_corr);
      float rpm = jyro_spin_dps_to_rpm(spin_dps);
      float wobble = sqrtf(gx*gx + gy*gy);

      char buf[160];
      if (isnan(heading)) {
        snprintf(buf, sizeof(buf),
                 "d_mm=%u  gz=%.1f dps  rpm=%.1f  wobble=%.1f  theta=%.1f  heading=N/A\n",
                 d_mm, gz_corr, rpm, wobble, theta_deg);
      } else {
        snprintf(buf, sizeof(buf),
                 "d_mm=%u  gz=%.1f dps  rpm=%.1f  wobble=%.1f  theta=%.1f  heading=%.1f\n",
                 d_mm, gz_corr, rpm, wobble, theta_deg, heading);
      }

      debug_print(buf);
    }
  }

#if STATUS_LED_BLINK_ENABLE
  statusLeds_update();
#endif

  // No delay: keep loop fast.
}
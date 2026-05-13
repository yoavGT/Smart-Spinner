#include "leds.h"
#include "pov.h"
#include "status_leds.h"

#include <Arduino.h>
#include "luna.h"
#include "jyro.h"

// ================= WIFI DEBUG SWITCH =================
#define WIFI_DEBUG true
// =====================================================

#if WIFI_DEBUG
#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>
#endif

/****************************************************
 * ===== WIFI / UDP TELEMETRY (Laptop <-> ESP32) =====
 * ESP32 creates a WiFi Access Point (AP). Connect your laptop to it.
 * Telemetry is sent via UDP broadcast on UDP_PORT.
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

  udp.beginPacket(udp_broadcast_ip, UDP_PORT);
  udp.write((const uint8_t*)msg, (int)strlen(msg));
  udp.endPacket();
}
#endif

/****************************************************
 * ===== debug_print() =====
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
#define CALIB_SPIN_NUM 3
static constexpr uint32_t SPIN_CALIB_TIMEOUT_MS = 3000;
// ====================================================

// ================= STATUS LED BLINK =================
#define STATUS_LED_BLINK_ENABLE true
static constexpr uint32_t STATUS_LED_BLINK_MS = 500;
// ====================================================

// ================= SIMPLE POV TEST =================
static constexpr float TARGET_ANGLE_DEG = 0.0f;   // Light location
static constexpr float ANGLE_WINDOW_DEG = 3.0f;   // +/- window around target
static constexpr uint8_t TARGET_LED_INDEX = 0;    // One LED per arm
static constexpr uint8_t TARGET_R = 255;
static constexpr uint8_t TARGET_G = 255;
static constexpr uint8_t TARGET_B = 255;
// ====================================================

// ===== Debug print control =====
static constexpr uint32_t PRINT_PERIOD_MS = 100;
static uint32_t last_print_ms = 0;

// ===== Theta integration =====
static float theta_deg = 0.0f;
static uint32_t last_theta_us = 0;

// ===== Phase lock using Luna zero events =====
static uint32_t last_zero_us = 0;
static uint32_t zero_period_us = 0;
static float omega_period_dps = 0.0f;
static bool have_period = false;
static constexpr float OMEGA_BLEND_GYRO = 0.7f; // 0..1, higher favors gyro

// ===== Gyro bias calibration =====
static float gz_bias_dps = 0.0f;

// ===== Zero reference =====
static LunaZeroRef zref;

// ===== Simple POV state =====
static bool target_led_on = false;
static uint8_t target_arm_index = 0;

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

  if (dt <= 0.0f || dt > 0.1f) {
    return;
  }

  float omega_dps = gz_dps;
  if (have_period && omega_period_dps > 0.0f) {
    float sign = (gz_dps >= 0.0f) ? 1.0f : -1.0f;
    float omega_period_signed = omega_period_dps * sign;
    omega_dps = OMEGA_BLEND_GYRO * gz_dps + (1.0f - OMEGA_BLEND_GYRO) * omega_period_signed;
  }

  theta_deg += omega_dps * dt;
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
 * ===== SPIN-BASED BIAS CALIBRATION =====
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
  statusLeds_allOn();

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

  statusLeds_allOff();
  Serial.println("Spin calibration done");

  return (samples > 0) ? (float)(sum_gz / samples) : 0.0f;
}

static void set_target_leds(bool on, uint8_t arm_index)
{
  for (uint8_t arm = 0; arm < LED_ARM_COUNT; arm++) {
    for (uint8_t i = 0; i < LEDS_PER_ARM; i++) {
      uint16_t off = arm * LEDS_PER_ARM + i;
      if (on && (arm == arm_index) && i == TARGET_LED_INDEX) {
        leds_set_rgb(off, TARGET_R, TARGET_G, TARGET_B);
      } else {
        leds_set_rgb(off, 0, 0, 0);
      }
    }
  }
  leds_show();
}

static float angle_delta_deg(float a, float b)
{
  float d = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
  return d;
}

void setup() {
  Serial.begin(115200);
  delay(200);

#if WIFI_DEBUG
  wifi_debug_init();
#endif

  statusLeds_init();
  statusLeds_allOff();

#if STATUS_LED_BLINK_ENABLE
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

  if (CALIB_SPIN_NUM > 0) {
    Serial.print("Spin the top for ");
    Serial.print(CALIB_SPIN_NUM);
    Serial.println(" revolutions...");
    gz_bias_dps = calibrate_gz_bias_over_spins(CALIB_SPIN_NUM);
    Serial.print("gz_bias_dps (FINAL) = ");
    Serial.println(gz_bias_dps, 4);
  }

  Serial.println("System running (simple POV test)");
}

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
    // Detect block pass -> defines 0 deg
    if (luna_update_zero(zref, d_mm, theta_deg)) {
      uint32_t now_us = micros();
      if (last_zero_us != 0) {
        zero_period_us = now_us - last_zero_us;
        if (zero_period_us > 0) {
          omega_period_dps = 360.0f * (1e6f / (float)zero_period_us);
          have_period = true;
        }
      }
      last_zero_us = now_us;

      theta_deg = 0.0f;
      last_theta_us = now_us;
      zref.zero_offset_deg = 0.0f;
      zref.has_zero = true;

      debug_print("ZERO SET (block detected)\n");
    }

    // Simple POV: light one LED on the arm closest to the target angle
    float heading = luna_angle_relative0(zref, theta_deg);
    bool heading_valid = !isnan(heading);

    if (heading_valid) {
      float best_delta = 360.0f;
      uint8_t best_arm = 0;

      for (uint8_t arm = 0; arm < LED_ARM_COUNT; arm++) {
        float arm_angle = fmodf(heading + (float)arm * 90.0f, 360.0f);
        float delta = fabsf(angle_delta_deg(arm_angle, TARGET_ANGLE_DEG));
        if (delta < best_delta) {
          best_delta = delta;
          best_arm = arm;
        }
      }

      bool in_window = best_delta <= ANGLE_WINDOW_DEG;

      if (in_window && (!target_led_on || best_arm != target_arm_index)) {
        target_led_on = true;
        target_arm_index = best_arm;
        set_target_leds(true, target_arm_index);
        debug_print("POV target hit: LED on\n");
      } else if (!in_window && target_led_on) {
        target_led_on = false;
        set_target_leds(false, target_arm_index);
      }
    } else if (target_led_on) {
      target_led_on = false;
      set_target_leds(false, target_arm_index);
    }

    // Debug print at low rate
    uint32_t now_ms = millis();
    if (now_ms - last_print_ms >= PRINT_PERIOD_MS) {
      last_print_ms = now_ms;

      float spin_dps = fabsf(gz_corr);
      float rpm = jyro_spin_dps_to_rpm(spin_dps);
      float wobble = sqrtf(gx*gx + gy*gy);

      char buf[160];
      if (heading_valid) {
        snprintf(buf, sizeof(buf),
                 "d_mm=%u  gz=%.1f dps  rpm=%.1f  wobble=%.1f  theta=%.1f  heading=%.1f\n",
                 d_mm, gz_corr, rpm, wobble, theta_deg, heading);
      } else {
        snprintf(buf, sizeof(buf),
                 "d_mm=%u  gz=%.1f dps  rpm=%.1f  wobble=%.1f  theta=%.1f  heading=N/A\n",
                 d_mm, gz_corr, rpm, wobble, theta_deg);
      }

      debug_print(buf);
    }
  }

#if STATUS_LED_BLINK_ENABLE
  statusLeds_update();
#endif

  // No delay: keep loop fast.
}

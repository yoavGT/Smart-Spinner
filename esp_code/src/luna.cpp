#include "luna.h"

static constexpr uint8_t DATA_HEAD = 0x59;
static constexpr uint8_t CMD_HEAD  = 0x5A;

static constexpr int FRAME_RATE_CONFIG_TRIES = 3;
static constexpr int SAVE_SETTINGS_TRIES     = 3;

static HardwareSerial Luna(2);

static inline uint8_t sum8(const uint8_t *b, size_t n) {
  uint16_t s = 0;
  for (size_t i = 0; i < n; i++) s += b[i];
  return (uint8_t)(s & 0xFF);
}

static inline void drain_input() {
  while (Luna.available()) (void)Luna.read();
}

static bool send_cmd_echo(const uint8_t *cmd, size_t len, uint32_t timeout_ms = 50) {
  drain_input();
  Luna.write(cmd, len);
  Luna.flush();

  uint8_t resp[16];
  if (len > sizeof(resp)) return false;

  uint32_t t0 = millis();
  size_t got = 0;
  while ((millis() - t0) < timeout_ms && got < len) {
    int c = Luna.read();
    if (c < 0) continue;
    resp[got++] = (uint8_t)c;
  }
  if (got != len) return false;

  if (resp[0] != CMD_HEAD) return false;
  if (resp[len - 1] != sum8(resp, len - 1)) return false;

  for (size_t i = 0; i < len; i++) {
    if (resp[i] != cmd[i]) return false;
  }
  return true;
}

static bool set_output_format_mm() {
  // 5A 05 05 06 6A
  uint8_t cmd[5] = { CMD_HEAD, 0x05, 0x05, 0x06, 0x00 };
  cmd[4] = sum8(cmd, 4);
  return send_cmd_echo(cmd, sizeof(cmd));
}

static bool set_frame_rate(uint16_t hz) {
  if (hz > 250) hz = 250;
  // 5A 06 03 LL HH SU
  uint8_t cmd[6] = { CMD_HEAD, 0x06, 0x03, (uint8_t)(hz & 0xFF), (uint8_t)(hz >> 8), 0x00 };
  cmd[5] = sum8(cmd, 5);
  return send_cmd_echo(cmd, sizeof(cmd));
}

static bool save_settings() {
  // 5A 04 11 6F
  uint8_t cmd[4] = { CMD_HEAD, 0x04, 0x11, 0x00 };
  cmd[3] = sum8(cmd, 3);
  return send_cmd_echo(cmd, sizeof(cmd));
}

bool luna_init_mm(uint32_t baud)
{
  Luna.setRxBufferSize(512);
  Luna.setTimeout(2);

  Luna.begin(baud, SERIAL_8N1, ESP_LUNA_RX, ESP_LUNA_TX);
  delay(30);
  drain_input();

  (void)set_output_format_mm();

  bool ok_fr = false;
  for (int i = 0; i < FRAME_RATE_CONFIG_TRIES && !ok_fr; i++) {
    ok_fr = set_frame_rate(LUNA_FRAME_RATE_HZ);
    if (!ok_fr) delay(10);
  }

  bool ok_save = false;
  for (int i = 0; i < SAVE_SETTINGS_TRIES && !ok_save; i++) {
    ok_save = save_settings();
    if (!ok_save) delay(10);
  }

  return true;
}

bool luna_read_mm_latest(uint16_t &distance_mm)
{
  bool got_any = false;

  while (Luna.available() >= 9) {
    int b0 = Luna.peek();
    if (b0 < 0) break;

    if ((uint8_t)b0 != DATA_HEAD) {
      (void)Luna.read();
      continue;
    }

    (void)Luna.read(); // first 0x59

    int b1 = Luna.read();
    if (b1 < 0) break;
    if ((uint8_t)b1 != DATA_HEAD) continue;

    uint8_t tail[7];
    size_t n = Luna.readBytes(tail, 7);
    if (n != 7) break;

    uint8_t chk = (uint8_t)(DATA_HEAD + DATA_HEAD);
    for (int i = 0; i < 6; i++) chk += tail[i];
    if (chk != tail[6]) continue;

    distance_mm = (uint16_t)(tail[0] | (uint16_t(tail[1]) << 8));
    got_any = true;
  }

  return got_any;
}

// -------- zero reference ----------
static inline float wrap360(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a < 0.0f)    a += 360.0f;
  return a;
}

bool luna_update_zero(LunaZeroRef &z, uint16_t dist_mm, float theta_deg)
{
  if (dist_mm <= z.near_th_mm) {
    z.near_hits++;
    z.far_hits = 0;

    if (z.armed && z.near_hits >= z.hits_needed) {
      z.zero_offset_deg = theta_deg;
      z.has_zero = true;

      z.armed = false;
      z.near_hits = 0;
      return true;
    }
  } else if (dist_mm >= z.far_th_mm) {
    z.far_hits++;
    z.near_hits = 0;

    if (!z.armed && z.far_hits >= z.fars_needed) {
      z.armed = true;
      z.far_hits = 0;
    }
  }
  return false;
}

float luna_angle_relative0(const LunaZeroRef &z, float theta_deg)
{
  if (!z.has_zero) return NAN;
  return wrap360(theta_deg - z.zero_offset_deg);
}
#include "pov.h"
#include "leds.h"
#include <string.h>
#include <math.h>

// POV params
static constexpr int POV_COLS_PER_REV = 360;  // 1 col/degree
static constexpr int FONT_H = 7;
static constexpr int FONT_W = 5;
static constexpr int LED_ROW_OFFSET = 1;      // font row0 -> LED1

static const char* gText = " ";
static uint8_t FG_R=255, FG_G=255, FG_B=255;
static uint8_t BG_R=0,   BG_G=0,   BG_B=0;
static int last_pov_col = -1;

// 5x7 font column fetch
static uint8_t font5x7_col(char c, int col)
{
  if (col < 0 || col >= 5) return 0;
  if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');

  if (c == ' ') {
    static const uint8_t cols[5] = {0,0,0,0,0};
    return cols[col];
  }

  if (c >= '0' && c <= '9') {
    static const uint8_t dig[10][5] = {
      {0x3E,0x51,0x49,0x45,0x3E}, // 0
      {0x00,0x42,0x7F,0x40,0x00}, // 1
      {0x42,0x61,0x51,0x49,0x46}, // 2
      {0x21,0x41,0x45,0x4B,0x31}, // 3
      {0x18,0x14,0x12,0x7F,0x10}, // 4
      {0x27,0x45,0x45,0x45,0x39}, // 5
      {0x3C,0x4A,0x49,0x49,0x30}, // 6
      {0x01,0x71,0x09,0x05,0x03}, // 7
      {0x36,0x49,0x49,0x49,0x36}, // 8
      {0x06,0x49,0x49,0x29,0x1E}, // 9
    };
    return dig[c - '0'][col];
  }

  if (c >= 'A' && c <= 'Z') {
    static const uint8_t let[26][5] = {
      {0x7E,0x11,0x11,0x11,0x7E}, // A
      {0x7F,0x49,0x49,0x49,0x36}, // B
      {0x3E,0x41,0x41,0x41,0x22}, // C
      {0x7F,0x41,0x41,0x22,0x1C}, // D
      {0x7F,0x49,0x49,0x49,0x41}, // E
      {0x7F,0x09,0x09,0x09,0x01}, // F
      {0x3E,0x41,0x49,0x49,0x7A}, // G
      {0x7F,0x08,0x08,0x08,0x7F}, // H
      {0x00,0x41,0x7F,0x41,0x00}, // I
      {0x20,0x40,0x41,0x3F,0x01}, // J
      {0x7F,0x08,0x14,0x22,0x41}, // K
      {0x7F,0x40,0x40,0x40,0x40}, // L
      {0x7F,0x02,0x0C,0x02,0x7F}, // M
      {0x7F,0x04,0x08,0x10,0x7F}, // N
      {0x3E,0x41,0x41,0x41,0x3E}, // O
      {0x7F,0x09,0x09,0x09,0x06}, // P
      {0x3E,0x41,0x51,0x21,0x5E}, // Q
      {0x7F,0x09,0x19,0x29,0x46}, // R
      {0x46,0x49,0x49,0x49,0x31}, // S
      {0x01,0x01,0x7F,0x01,0x01}, // T
      {0x3F,0x40,0x40,0x40,0x3F}, // U
      {0x1F,0x20,0x40,0x20,0x1F}, // V
      {0x3F,0x40,0x38,0x40,0x3F}, // W
      {0x63,0x14,0x08,0x14,0x63}, // X
      {0x07,0x08,0x70,0x08,0x07}, // Y
      {0x61,0x51,0x49,0x45,0x43}, // Z
    };
    return let[c - 'A'][col];
  }

  return 0;
}

static int message_total_cols()
{
  int len = (int)strlen(gText);
  return len * (FONT_W + 1); // 5 + spacing
}

static uint8_t message_col_bitmap(int msg_col)
{
  const int perChar = FONT_W + 1;
  int len = (int)strlen(gText);
  int total = len * perChar;
  if (total <= 0) return 0;

  msg_col %= total;
  if (msg_col < 0) msg_col += total;

  int ci = msg_col / perChar;
  int within = msg_col % perChar;
  if (within == FONT_W) return 0; // spacing

  return font5x7_col(gText[ci], within);
}

static void render_column(uint8_t colBits)
{
  leds_set_all(BG_R, BG_G, BG_B);

  for (int row = 0; row < FONT_H; row++) {
    if ((colBits >> row) & 0x01) {
      int led = LED_ROW_OFFSET + row;
      if (led >= 0 && led < (int)leds_count()) {
        leds_set_rgb((uint16_t)led, FG_R, FG_G, FG_B);
      }
    }
  }

  leds_show();
}

void pov_init(const char* text,
              uint8_t fg_r, uint8_t fg_g, uint8_t fg_b,
              uint8_t bg_r, uint8_t bg_g, uint8_t bg_b,
              uint8_t global_brightness)
{
  gText = (text && text[0]) ? text : " ";
  FG_R = fg_r; FG_G = fg_g; FG_B = fg_b;
  BG_R = bg_r; BG_G = bg_g; BG_B = bg_b;

  leds_set_global_brightness(global_brightness);
  last_pov_col = -1;

  // waiting indicator
  leds_set_all(0,0,0);
  if (leds_count() > 0) leds_set_rgb(0, 0, 0, 32);
  leds_show();
}

void pov_update(bool heading_valid, float heading_deg)
{
  if (!heading_valid || isnan(heading_deg)) {
    leds_set_all(0,0,0);
    if (leds_count() > 0) leds_set_rgb(0, 0, 0, 32);
    leds_show();
    last_pov_col = -1;
    return;
  }

  int pov_col = (int)(heading_deg * (POV_COLS_PER_REV / 360.0f));
  if (pov_col >= POV_COLS_PER_REV) pov_col = POV_COLS_PER_REV - 1;
  if (pov_col < 0) pov_col = 0;

  if (pov_col == last_pov_col) return;
  last_pov_col = pov_col;

  int msgCols = message_total_cols();
  int msg_col = (pov_col * msgCols) / POV_COLS_PER_REV;

  render_column(message_col_bitmap(msg_col));
}

#include "leds.h"
#include <SPI.h>

// APA102 frame format:
// START: 4 bytes of 0x00
// Per LED: [0b111xxxxx][B][G][R] where xxxxx = global brightness (0..31)
// END: at least (n/2) bits of 1 -> commonly (n+15)/16 bytes of 0xFF (works well)

static SPIClass spi(VSPI);                 // use VSPI hardware peripheral
static SPISettings spiSettings(8000000, MSBFIRST, SPI_MODE0);

static uint8_t gBrightness = 31;

// Frame sizes
static constexpr uint16_t N = LED_COUNT;
static constexpr uint16_t N_PER_ARM = LEDS_PER_ARM;
static constexpr uint16_t START_BYTES = 4;
static constexpr uint16_t LED_BYTES   = 4 * N_PER_ARM;
static constexpr uint16_t END_BYTES   = (N_PER_ARM + 15) / 16;   // recommended end-frame bytes

// Full frame buffer(s): start + led frames + end
#if LED_ARM_COUNT == 1
static uint8_t frame[START_BYTES + LED_BYTES + END_BYTES];
#else
static uint8_t frame[LED_ARM_COUNT][START_BYTES + LED_BYTES + END_BYTES];
static const uint8_t dataPins[LED_ARM_COUNT] = {
  LED_SDI1_PIN, LED_SDI2_PIN, LED_SDI3_PIN, LED_SDI4_PIN
};
#endif

static inline uint16_t led_offset(uint16_t i) {
  // LED i starts after START_BYTES, 4 bytes each
  return START_BYTES + 4 * i;
}

uint16_t leds_count() { return N; }

void leds_set_global_brightness(uint8_t b)
{
  if (b > 31) b = 31;
  gBrightness = b;

  // Update brightness field for all LEDs in the frame buffer
#if LED_ARM_COUNT == 1
  for (uint16_t i = 0; i < N_PER_ARM; i++) {
    frame[led_offset(i) + 0] = (uint8_t)(0xE0 | gBrightness);
  }
#else
  for (uint16_t arm = 0; arm < LED_ARM_COUNT; arm++) {
    for (uint16_t i = 0; i < N_PER_ARM; i++) {
      frame[arm][led_offset(i) + 0] = (uint8_t)(0xE0 | gBrightness);
    }
  }
#endif
}

void leds_init(uint32_t spi_hz, uint8_t global_brightness)
{
#if LED_ARM_COUNT == 1
  // Setup SPI pins: (sck, miso, mosi, ss)
  // APA102 uses only SCK + MOSI (data in). MISO/SS unused.
  spi.begin(LED_CLK_PIN, -1, LED_SDI1_PIN, -1);

  // Build SPI settings
  spiSettings = SPISettings(spi_hz, MSBFIRST, SPI_MODE0);
#else
  // Parallel arms: manual bit-bang across multiple data pins.
  pinMode(LED_CLK_PIN, OUTPUT);
  digitalWrite(LED_CLK_PIN, LOW);
  for (uint16_t arm = 0; arm < LED_ARM_COUNT; arm++) {
    pinMode(dataPins[arm], OUTPUT);
    digitalWrite(dataPins[arm], LOW);
  }
#endif

  // Start frame
#if LED_ARM_COUNT == 1
  for (uint16_t i = 0; i < START_BYTES; i++) frame[i] = 0x00;
#else
  for (uint16_t arm = 0; arm < LED_ARM_COUNT; arm++) {
    for (uint16_t i = 0; i < START_BYTES; i++) frame[arm][i] = 0x00;
  }
#endif

  // End frame
#if LED_ARM_COUNT == 1
  for (uint16_t i = 0; i < END_BYTES; i++) frame[START_BYTES + LED_BYTES + i] = 0xFF;
#else
  for (uint16_t arm = 0; arm < LED_ARM_COUNT; arm++) {
    for (uint16_t i = 0; i < END_BYTES; i++) frame[arm][START_BYTES + LED_BYTES + i] = 0xFF;
  }
#endif

  // Default brightness + clear colors
  leds_set_global_brightness(global_brightness);

#if LED_ARM_COUNT == 1
  for (uint16_t i = 0; i < N_PER_ARM; i++) {
    uint16_t off = led_offset(i);
    // frame[off+0] already set by brightness setter
    frame[off + 1] = 0; // B
    frame[off + 2] = 0; // G
    frame[off + 3] = 0; // R
  }
#else
  for (uint16_t arm = 0; arm < LED_ARM_COUNT; arm++) {
    for (uint16_t i = 0; i < N_PER_ARM; i++) {
      uint16_t off = led_offset(i);
      // frame[off+0] already set by brightness setter
      frame[arm][off + 1] = 0; // B
      frame[arm][off + 2] = 0; // G
      frame[arm][off + 3] = 0; // R
    }
  }
#endif

  // Push once so you know it’s alive
  leds_show();
}

void leds_set_rgb(uint16_t i, uint8_t r, uint8_t g, uint8_t b)
{
  if (i >= N) return;

  uint16_t arm = i / N_PER_ARM;
  uint16_t idx = i % N_PER_ARM;
  uint16_t off = led_offset(idx);

#if LED_ARM_COUNT == 1
  frame[off + 0] = (uint8_t)(0xE0 | gBrightness);
  frame[off + 1] = b;  // APA102 order: B,G,R
  frame[off + 2] = g;
  frame[off + 3] = r;
#else
  frame[arm][off + 0] = (uint8_t)(0xE0 | gBrightness);
  frame[arm][off + 1] = b;  // APA102 order: B,G,R
  frame[arm][off + 2] = g;
  frame[arm][off + 3] = r;
#endif
}

void leds_set_all(uint8_t r, uint8_t g, uint8_t b)
{
  for (uint16_t i = 0; i < N; i++) {
    leds_set_rgb(i, r, g, b);
  }
}

void leds_clear()
{
  leds_set_all(0, 0, 0);
}

void leds_show()
{
#if LED_ARM_COUNT == 1
  spi.beginTransaction(spiSettings);
  spi.writeBytes(frame, sizeof(frame));
  spi.endTransaction();
#else
  const uint16_t frame_size = START_BYTES + LED_BYTES + END_BYTES;

  // Ensure SDI lines are idle low between frames.
  for (uint16_t arm = 0; arm < LED_ARM_COUNT; arm++) {
    digitalWrite(dataPins[arm], LOW);
  }
  digitalWrite(LED_CLK_PIN, LOW);

  for (uint16_t byte = 0; byte < frame_size; byte++) {
    for (int bit = 7; bit >= 0; bit--) {
      for (uint16_t arm = 0; arm < LED_ARM_COUNT; arm++) {
        bool high = (frame[arm][byte] & (1 << bit)) != 0;
        digitalWrite(dataPins[arm], high ? HIGH : LOW);
      }
      digitalWrite(LED_CLK_PIN, HIGH);
      digitalWrite(LED_CLK_PIN, LOW);
    }
  }

  // Leave SDI lines low so they are not stuck high after the transfer.
  for (uint16_t arm = 0; arm < LED_ARM_COUNT; arm++) {
    digitalWrite(dataPins[arm], LOW);
  }
#endif
}

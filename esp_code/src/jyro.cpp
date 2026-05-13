#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "jyro.h"

// ===== Pins (your choice) =====
#define MISO 19
#define MOSI 18
#define SCK  5
#define CS   17

// ===== SPI =====
#define SPI_CLK 10000000  // 10 MHz

SPIClass spi(HSPI);
SPISettings spiCfg(SPI_CLK, MSBFIRST, SPI_MODE3);

// ===== LSM6DSV registers =====
#define CTRL2_G      0x11
#define CTRL3_C      0x12
#define CTRL6_G      0x15
#define OUTX_L_G     0x22

static constexpr float GYRO_SENS_DPS_PER_LSB = 0.07f;

static uint8_t read_reg(uint8_t reg) {
  spi.beginTransaction(spiCfg);
  digitalWrite(CS, LOW);

  spi.transfer(reg | 0x80);
  uint8_t v = spi.transfer(0x00);

  digitalWrite(CS, HIGH);
  spi.endTransaction();
  return v;
}

static void write_reg(uint8_t reg, uint8_t val) {
  spi.beginTransaction(spiCfg);
  digitalWrite(CS, LOW);

  spi.transfer(reg & 0x7F);
  spi.transfer(val);

  digitalWrite(CS, HIGH);
  spi.endTransaction();
}

void jyro_init() {
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  spi.begin(SCK, MISO, MOSI, CS);

  write_reg(CTRL3_C, 0x04); // IF_INC=1
  write_reg(CTRL2_G, 0x05); // ODR=60 Hz (your working value)
  write_reg(CTRL6_G, 0x04); // FS=±2000 dps

  (void)read_reg(CTRL3_C);
}

void jyro_read_xyz_dps(float &gx_dps, float &gy_dps, float &gz_dps) {
  uint8_t b[6];

  spi.beginTransaction(spiCfg);
  digitalWrite(CS, LOW);

  spi.transfer(OUTX_L_G | 0x80);
  for (int i = 0; i < 6; i++) b[i] = spi.transfer(0);

  digitalWrite(CS, HIGH);
  spi.endTransaction();

  int16_t gx_raw = (int16_t)((b[1] << 8) | b[0]);
  int16_t gy_raw = (int16_t)((b[3] << 8) | b[2]);
  int16_t gz_raw = (int16_t)((b[5] << 8) | b[4]);

  gx_dps = gx_raw * GYRO_SENS_DPS_PER_LSB;
  gy_dps = gy_raw * GYRO_SENS_DPS_PER_LSB;
  gz_dps = gz_raw * GYRO_SENS_DPS_PER_LSB;
}

float jyro_get_spin_dps() {
  float gx, gy, gz;
  jyro_read_xyz_dps(gx, gy, gz);
  return fabsf(gz);
}

float jyro_get_wobble_dps() {
  float gx, gy, gz;
  jyro_read_xyz_dps(gx, gy, gz);
  (void)gz;
  return sqrtf(gx*gx + gy*gy);
}

float jyro_spin_dps_to_rpm(float spin_dps) {
  return (spin_dps / 360.0f) * 60.0f;
}
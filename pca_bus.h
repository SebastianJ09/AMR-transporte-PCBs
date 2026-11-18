#pragma once
#include "amr_pins.h"
#include <math.h>

inline bool &amr_pca_ready() {
  static bool ready = false;
  return ready;
}

inline void amr_pca_write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(AMR_PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

inline uint8_t amr_pca_read_reg(uint8_t reg) {
  Wire.beginTransmission(AMR_PCA9685_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)AMR_PCA9685_ADDR, (size_t)1, true);
  return Wire.available() ? Wire.read() : 0;
}

inline bool amr_pca_init(float freq_hz = 50.0f) {
  amr_i2c_begin();
  bool &ready = amr_pca_ready();
  if (ready) return true;

  Wire.beginTransmission(AMR_PCA9685_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("[PCA9685] No detectado en 0x40 (revisar SDA=13, SCL=14, VCC=3.3V)");
    return false;
  }

  amr_pca_write_reg(0x00, 0x00);
  float prescaleval = 25000000.0f / (4096.0f * freq_hz) - 1.0f;
  uint8_t prescale = (uint8_t)floorf(prescaleval + 0.5f);

  uint8_t oldmode = amr_pca_read_reg(0x00);
  uint8_t newmode = (oldmode & 0x7F) | 0x10;
  amr_pca_write_reg(0x00, newmode);
  amr_pca_write_reg(0xFE, prescale);
  amr_pca_write_reg(0x00, oldmode);
  delay(5);
  amr_pca_write_reg(0x00, oldmode | 0xA0);

  ready = true;
  Serial.printf("[PCA9685] Inicializado en 0x40 (frecuencia PWM = %.1f Hz)\n", freq_hz);
  return true;
}

inline void amr_pca_set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
  if (!amr_pca_ready()) amr_pca_init();
  Wire.beginTransmission(AMR_PCA9685_ADDR);
  Wire.write(0x06 + 4 * channel);
  Wire.write(on & 0xFF);
  Wire.write(on >> 8);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  Wire.endTransmission();
}

inline void amr_pca_set_duty8(uint8_t channel, uint8_t duty8) {
  if (duty8 == 0) {
    amr_pca_set_pwm(channel, 0, 4096);
  } else if (duty8 >= 255) {
    amr_pca_set_pwm(channel, 4096, 0);
  } else {
    uint16_t off = (uint16_t)((uint32_t)duty8 * 4095u / 255u);
    amr_pca_set_pwm(channel, 0, off);
  }
}

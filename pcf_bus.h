#pragma once
#include "amr_pins.h"

inline uint8_t &amr_pcf_addr() {
  static uint8_t addr = 0;
  return addr;
}

inline uint8_t &amr_pcf_shadow() {
  static uint8_t s = 0x00;
  return s;
}

inline uint8_t amr_pcf_detect() {
  amr_i2c_begin();
  uint8_t &addr = amr_pcf_addr();
  if (addr != 0) return addr;

  const uint8_t candidates[] = { 0x20, 0x27, 0x3F, 0x38, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26 };
  for (uint8_t c : candidates) {
    Wire.beginTransmission(c);
    if (Wire.endTransmission() == 0) {
      addr = c;
      Serial.printf("[PCF8574] Detectado en direccion I2C 0x%02X\n", addr);
      return addr;
    }
  }
  addr = AMR_PCF8574_DEFAULT_ADDR;
  return addr;
}

inline bool amr_pcf_write_bits(uint8_t mask, uint8_t value) {
  uint8_t addr = amr_pcf_detect();
  uint8_t &sh = amr_pcf_shadow();
  sh = (uint8_t)((sh & ~mask) | (value & mask));
  Wire.beginTransmission(addr);
  Wire.write(sh);
  return Wire.endTransmission() == 0;
}

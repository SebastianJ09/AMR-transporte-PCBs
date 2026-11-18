#pragma once
#include <Arduino.h>
#include <Wire.h>

// ---- Bus I2C unificado (3.3 V) ----
constexpr uint8_t  AMR_I2C_SDA      = 13;
constexpr uint8_t  AMR_I2C_SCL      = 14;
constexpr uint32_t AMR_I2C_FREQ_HZ  = 100000;
constexpr uint8_t  AMR_PCA9685_ADDR = 0x40;
constexpr uint8_t  AMR_MPU6050_ADDR = 0x68;
constexpr uint8_t  AMR_PCF8574_DEFAULT_ADDR = 0x20;

// ---- Bits en el Expansor PCF8574 ----
constexpr uint8_t AMR_PCF_AIN1 = 0;   // P0: Motor izquierdo AIN1
constexpr uint8_t AMR_PCF_AIN2 = 1;   // P1: Motor izquierdo AIN2
constexpr uint8_t AMR_PCF_BIN1 = 2;   // P2: Motor derecho BIN1
constexpr uint8_t AMR_PCF_BIN2 = 3;   // P3: Motor derecho BIN2

enum UsId : uint8_t { US_FRONT = 0, US_LEFT = 1, US_RIGHT = 2, US_COUNT = 3 };
constexpr uint8_t AMR_PCF_TRIG[US_COUNT]    = { 4, 5, 6 };     // P4 (Frontal), P5 (Izq), P6 (Der)
constexpr uint8_t AMR_PCF_TRIG_MASK         = (1u << 4) | (1u << 5) | (1u << 6); // 0x70
constexpr uint8_t AMR_PCF_MOTOR_MASK        = 0x0F;            // P0..P3

// ---- Entradas ECHO en el ESP32 (con divisor 1k / 2k) ----
constexpr uint8_t AMR_US_ECHO_PIN[US_COUNT] = { 32, 33, 15 };  // GPIO 32 (F), 33 (I), 15 (D)

// ---- Canales del Controlador PWM PCA9685 (0x40) ----
constexpr uint8_t AMR_PCA_CH_PWMA  = 0;   // Canal 0 -> PWMA (Motor Izquierdo)
constexpr uint8_t AMR_PCA_CH_PWMB  = 1;   // Canal 1 -> PWMB (Motor Derecho)
constexpr uint8_t AMR_PCA_CH_SERVO = 2;   // Canal 2 -> Servomotor SG90

// ---- Pines para pruebas aisladas opcionales ----
constexpr uint8_t AMR_FC16_DIN  = 23;
constexpr uint8_t AMR_FC16_CLK  = 18;
constexpr uint8_t AMR_FC16_CS   = 5;
constexpr uint8_t AMR_SERVO_PIN = 12;

inline void amr_i2c_begin() {
  static bool started = false;
  if (started) return;
  Wire.begin((int)AMR_I2C_SDA, (int)AMR_I2C_SCL, AMR_I2C_FREQ_HZ);
  started = true;
}

#include "motor_driver.h"
#include "amr_pins.h"
#include "pcf_bus.h"
#include "pca_bus.h"
#include <Wire.h>

static bool pcf_ok = true;

static bool pcf8574_write_directions(uint8_t mask) {
  bool ok = amr_pcf_write_bits(AMR_PCF_MOTOR_MASK, mask);
  if (ok != pcf_ok) {
    Serial.printf("[TB6612FNG] PCF8574 (0x%02X) %s\n", amr_pcf_addr(),
                  ok ? "conectado" : "NO responde (revisar SDA=13/SCL=14/3V3)");
    pcf_ok = ok;
  }
  return ok;
}

void motor_driver_init() {
  amr_i2c_begin();
  amr_pcf_detect();
  amr_pca_init(50.0f);
  motor_driver_stop();
  Serial.printf("[TB6612FNG] Inicializado: Dir por PCF8574 (0x%02X P0..P3) | PWM por PCA9685 (0x40 Ch0/Ch1)\n",
                amr_pcf_addr());
}

void motor_driver_set(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  uint8_t mask = 0x00;
  if (left > 0)       mask |= (1 << AMR_PCF_AIN1);
  else if (left < 0)  mask |= (1 << AMR_PCF_AIN2);

  if (right > 0)      mask |= (1 << AMR_PCF_BIN1);
  else if (right < 0) mask |= (1 << AMR_PCF_BIN2);

  pcf8574_write_directions(mask);
  amr_pca_set_duty8(AMR_PCA_CH_PWMA, (uint8_t)abs(left));
  amr_pca_set_duty8(AMR_PCA_CH_PWMB, (uint8_t)abs(right));
}

void motor_driver_stop() {
  pcf8574_write_directions(0x00);
  amr_pca_set_duty8(AMR_PCA_CH_PWMA, 0);
  amr_pca_set_duty8(AMR_PCA_CH_PWMB, 0);
}

void motor_driver_test_loop() {
  struct Paso { const char *nombre; int l; int r; uint16_t ms; };
  static const Paso pasos[] = {
    {"Adelante",              160,  160, 1500},
    {"Parada",                  0,    0,  700},
    {"Reversa",              -160, -160, 1500},
    {"Parada",                  0,    0,  700},
    {"Giro izquierda (eje)", -160,  160, 1000},
    {"Giro derecha (eje)",    160, -160, 1000},
    {"Solo izquierda",        160,    0, 1000},
    {"Solo derecha",            0,  160, 1000},
    {"Parada",                  0,    0, 2000},
  };
  for (const Paso &p : pasos) {
    Serial.printf("[MOTOR] %-22s L=%4d R=%4d\n", p.nombre, p.l, p.r);
    motor_driver_set(p.l, p.r);
    delay(p.ms);
  }
  motor_driver_stop();
}

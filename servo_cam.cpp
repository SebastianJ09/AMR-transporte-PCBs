#include "servo_cam.h"
#include "amr_pins.h"
#include "pca_bus.h"

static uint8_t current_angle = 90;

static uint16_t angle_to_ticks(uint8_t deg) {
  if (deg > 180) deg = 180;
  return (uint16_t)(102 + ((uint32_t)deg * (491 - 102)) / 180);
}

void servo_cam_init() {
  amr_pca_init(50.0f);
  servo_cam_set_angle(90);
  Serial.printf("[SG90] Servo de paneo inicializado en PCA9685 Canal %u (50 Hz).\n", AMR_PCA_CH_SERVO);
}

void servo_cam_set_angle(uint8_t deg) {
  if (deg > 180) deg = 180;
  current_angle = deg;
  amr_pca_set_pwm(AMR_PCA_CH_SERVO, 0, angle_to_ticks(deg));
}

uint8_t servo_cam_get_angle() { return current_angle; }

void servo_cam_test_loop() {
  for (int a = 0; a <= 180; a += 5) { servo_cam_set_angle(a); delay(30); }
  for (int a = 180; a >= 0; a -= 5) { servo_cam_set_angle(a); delay(30); }
  servo_cam_set_angle(90);
  Serial.printf("[SG90] Barrido completo en Canal 2 PCA9685, angulo=%u\n", servo_cam_get_angle());
  delay(1000);
}

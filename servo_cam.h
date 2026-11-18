#pragma once
#include <Arduino.h>

void servo_cam_init();
void servo_cam_set_angle(uint8_t deg);  // 0..180
uint8_t servo_cam_get_angle();
void servo_cam_test_loop();

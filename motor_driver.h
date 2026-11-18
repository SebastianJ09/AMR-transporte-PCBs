#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

void motor_driver_init();
void motor_driver_set(int left, int right);   // -255..255 (negativo = reversa)
void motor_driver_stop();
void motor_driver_test_loop();                // prueba unitaria (ruedas en el aire)

#endif // MOTOR_DRIVER_H

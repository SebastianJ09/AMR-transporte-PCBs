#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>
#include "amr_pins.h"

void  ultrasonic_init();
void  ultrasonic_update();
float ultrasonic_get_cm(UsId id);
bool  ultrasonic_ok(UsId id);
float ultrasonic_read_cm();
void  ultrasonic_test_loop();

#endif // ULTRASONIC_H

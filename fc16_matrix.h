#pragma once
#include <Arduino.h>

void fc16_matrix_init();
void fc16_matrix_clear();
void fc16_matrix_all_on();                               // 64 LEDs por datos
void fc16_matrix_all_off();                              // apaga todos los LEDs
void fc16_matrix_set_brightness(uint8_t level);          // 0..15
void fc16_matrix_show(const uint8_t rows[8]);            // fila 0 arriba, bit7 = columna izq.
void fc16_matrix_test_loop();

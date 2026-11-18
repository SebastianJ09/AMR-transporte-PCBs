#pragma once
#include <Arduino.h>

// ============================================================================
//  MODO INTEGRADO FINAL - Robot AMR (ESP32-WROVER)
// ----------------------------------------------------------------------------
//  - Bus I2C unificado (SDA=13, SCL=14):
//      * PCF8574  : P0..P3 -> AIN1, AIN2, BIN1, BIN2 (TB6612FNG)
//                   P4..P6 -> TRIG Frontal, Izquierdo, Derecho (HC-SR04 x3)
//      * PCA9685  : Ch0=PWMA, Ch1=PWMB (TB6612FNG), Ch2=Servo SG90 (50 Hz)
//      * MPU-6050 : 0x68
//  - ECHO HC-SR04 (divisor 1k/2k): Frontal=GPIO32, Izq=GPIO33, Der=GPIO15
//  - Wi-Fi AP "AMR_ROBOT_CAM" (clave "12345678", IP 192.168.4.1):
//      * Puerto 80 : Dashboard Web (vehiculo-preview-3sensores.html) + WS (/ws)
//      * Puerto 81 : Stream MJPEG para el Dashboard Web (/stream)
//      * Puerto 82 : Stream MJPEG para Python / OpenCV (/stream)
//      * Puerto 1883 (cliente): Broker Mosquitto en PC (192.168.4.2)
// ============================================================================

void amr_integrado_setup();
void amr_integrado_loop();

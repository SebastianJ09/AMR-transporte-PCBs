#pragma once
#include <Arduino.h>

// Camara OV2640 integrada en la Freenove ESP32-WROVER (conector FPC).
// Este modulo controla el SENSOR DE IMAGEN. El movimiento (pan) de la camara
// es responsabilidad de servo_cam.h / servo_cam.cpp.

// Inicializa unicamente el sensor OV2640 (sin iniciar Wi-Fi ni servidor HTTP).
// Util cuando el AP y los servidores web/stream se gestionan desde amr_integrado.
bool camera_ov2640_init_sensor();

// Video por Wi-Fi (MJPEG) autonomo para pruebas aisladas: inicia el sensor,
// crea el AP "AMR_ROBOT_CAM" (clave "12345678", IP 192.168.4.1) y el servidor HTTP (80).
void camera_ov2640_init();
bool camera_ov2640_is_ready();

// Captura un frame JPEG. Devuelve tamano en bytes (0 si falla).
// Si ancho/alto no son nullptr, devuelve la resolucion del frame.
size_t camera_ov2640_capture(uint16_t *ancho = nullptr, uint16_t *alto = nullptr);
void camera_ov2640_set_quality(uint8_t calidad);   // 10 (mejor) .. 63 (peor)
void camera_ov2640_set_framesize(int framesize);   // FRAMESIZE_QVGA, FRAMESIZE_VGA, etc.
void camera_ov2640_set_flip(bool vflip, bool hmirror);
void camera_ov2640_test_loop();

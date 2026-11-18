#pragma once
#include <Arduino.h>

// Inicia servidores HTTP independientes (esp_http_server) para streaming MJPEG
// simultaneo en cada puerto indicado (ej. 81 para Web y 82 para Python/OpenCV).
// Ambos servidores atienden tanto "/stream" como "/".
bool camera_stream_start(const uint16_t *ports, uint8_t count);
bool camera_stream_is_running();

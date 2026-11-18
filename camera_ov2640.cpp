#include "camera_ov2640.h"
#include "esp_camera.h"
#include <WiFi.h>

// ============================================================================
//  CAMARA OV2640 - Freenove ESP32-WROVER (pinout fijo de la placa, no se cablea)
// ----------------------------------------------------------------------------
//  | Senal  | GPIO |   | Senal | GPIO |
//  |--------|------|   |-------|------|
//  | XCLK   | 21   |   | Y9    | 35   |
//  | SIOD   | 26   |   | Y8    | 34   |
//  | SIOC   | 27   |   | Y7    | 39   |
//  | VSYNC  | 25   |   | Y6    | 36   |
//  | HREF   | 23   |   | Y5    | 19   |
//  | PCLK   | 22   |   | Y4    | 18   |
//  | PWDN   | --   |   | Y3    | 5    |
//  | RESET  | --   |   | Y2    | 4    |
//
//  CONFLICTOS con otros modulos del proyecto (solo se pueden usar juntos si
//  se reasignan pines; en el harness de pruebas se prueban por separado):
//    - fc16_matrix : DIN 23, CLK 18, CS 5
//    - ultrasonic  : TRIG 4 y 19, ECHO 34, 35 y 39
//    - imu_mpu6050 : SDA 21, SCL 22
//    - motor_driver: 25, 26, 27
//  Libres con la camara activa: GPIO 0*, 2, 12*, 13, 14, 15, 32, 33
//  (* pines de arranque, usarlos con cuidado).
//
//  Requiere PSRAM (-DBOARD_HAS_PSRAM ya esta en platformio.ini).
// ============================================================================

#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   21
#define CAM_PIN_SIOD   26
#define CAM_PIN_SIOC   27
#define CAM_PIN_Y9     35
#define CAM_PIN_Y8     34
#define CAM_PIN_Y7     39
#define CAM_PIN_Y6     36
#define CAM_PIN_Y5     19
#define CAM_PIN_Y4     18
#define CAM_PIN_Y3     5
#define CAM_PIN_Y2     4
#define CAM_PIN_VSYNC  25
#define CAM_PIN_HREF   23
#define CAM_PIN_PCLK   22

static bool cam_ok = false;

// Wi-Fi AP + servidor HTTP (puerto 80)
static WiFiServer http_server(80);
static WiFiClient stream_client;
static bool wifi_started = false;
static bool streaming = false;
static uint32_t stream_frames = 0;
static uint32_t stream_t0 = 0;

bool camera_ov2640_init_sensor() {
  if (cam_ok) return true;

  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = CAM_PIN_Y2;  c.pin_d1 = CAM_PIN_Y3;
  c.pin_d2 = CAM_PIN_Y4;  c.pin_d3 = CAM_PIN_Y5;
  c.pin_d4 = CAM_PIN_Y6;  c.pin_d5 = CAM_PIN_Y7;
  c.pin_d6 = CAM_PIN_Y8;  c.pin_d7 = CAM_PIN_Y9;
  c.pin_xclk  = CAM_PIN_XCLK;
  c.pin_pclk  = CAM_PIN_PCLK;
  c.pin_vsync = CAM_PIN_VSYNC;
  c.pin_href  = CAM_PIN_HREF;
  c.pin_sccb_sda = CAM_PIN_SIOD;
  c.pin_sccb_scl = CAM_PIN_SIOC;
  c.pin_pwdn  = CAM_PIN_PWDN;
  c.pin_reset = CAM_PIN_RESET;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.grab_mode    = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    c.frame_size   = FRAMESIZE_VGA;    // 640x480
    c.jpeg_quality = 12;
    c.fb_count     = 2;
    c.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    Serial.println("[CAM] Aviso: sin PSRAM, se usa QVGA en DRAM");
    c.frame_size   = FRAMESIZE_QVGA;   // 320x240
    c.jpeg_quality = 15;
    c.fb_count     = 1;
    c.fb_location  = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&c);
  cam_ok = (err == ESP_OK);
  if (!cam_ok) {
    Serial.printf("[CAM] esp_camera_init fallo: 0x%x (revisar cable FPC)\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("[CAM] Sensor PID=0x%02X (OV2640 = 0x26)\n", s->id.PID);
  camera_ov2640_set_flip(false, false);
  return true;
}

void camera_ov2640_init() {
  if (!camera_ov2640_init_sensor()) {
    return;
  }

  // ---- Wi-Fi Access Point + servidor HTTP ----
  if (!wifi_started) {
    WiFi.mode(WIFI_AP);
    const IPAddress ip(192, 168, 4, 1);
    WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
    if (WiFi.softAP("AMR_ROBOT_CAM", "12345678")) {
      WiFi.setSleep(false);   // menor latencia en el stream
      http_server.begin();
      wifi_started = true;
      Serial.printf("[CAM] AP 'AMR_ROBOT_CAM' activo | IP %s | http://%s/stream\n",
                    WiFi.softAPIP().toString().c_str(), WiFi.softAPIP().toString().c_str());
    } else {
      Serial.println("[CAM] Error al iniciar el Access Point");
    }
  }
}

bool camera_ov2640_is_ready() { return cam_ok; }

size_t camera_ov2640_capture(uint16_t *ancho, uint16_t *alto) {
  if (!cam_ok) return 0;
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return 0;

  // Un JPEG valido empieza con 0xFF 0xD8
  size_t len = (fb->len > 2 && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8) ? fb->len : 0;
  if (ancho) *ancho = fb->width;
  if (alto)  *alto  = fb->height;
  esp_camera_fb_return(fb);   // liberar siempre el buffer
  return len;
}

void camera_ov2640_set_quality(uint8_t calidad) {
  if (!cam_ok) return;
  esp_camera_sensor_get()->set_quality(esp_camera_sensor_get(), constrain(calidad, 10, 63));
}

void camera_ov2640_set_framesize(int framesize) {
  if (!cam_ok) return;
  sensor_t *s = esp_camera_sensor_get();
  if (s) s->set_framesize(s, (framesize_t)framesize);
}

void camera_ov2640_set_flip(bool vflip, bool hmirror) {
  if (!cam_ok) return;
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, vflip);
  s->set_hmirror(s, hmirror);
}

// --------------------------------------------------------------------------
//  Streaming HTTP (MJPEG) - se atiende desde camera_ov2640_test_loop()
//  Un solo cliente de video a la vez. Sin tareas ni bloqueos largos: cada
//  llamada a test_loop() acepta clientes nuevos y envia, como maximo, UN frame.
// --------------------------------------------------------------------------
static const char *HDR_STREAM =
  "HTTP/1.1 200 OK\r\n"
  "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
  "Cache-Control: no-cache\r\n"
  "Access-Control-Allow-Origin: *\r\n"
  "Connection: close\r\n\r\n";

// Lee la cabecera HTTP (hasta "\r\n\r\n") con un timeout corto y devuelve
// la primera linea ("GET /stream HTTP/1.1").
static String leer_peticion(WiFiClient &cl) {
  String hdr;
  uint32_t t0 = millis();
  while (cl.connected() && millis() - t0 < 500 && hdr.length() < 1024) {
    while (cl.available()) {
      hdr += (char)cl.read();
      if (hdr.endsWith("\r\n\r\n")) {
        int fin = hdr.indexOf("\r\n");
        return hdr.substring(0, fin);
      }
    }
    delay(1);
  }
  int fin = hdr.indexOf("\r\n");
  return fin > 0 ? hdr.substring(0, fin) : hdr;
}

static void cerrar_cliente() {
  if (stream_client) stream_client.stop();
  streaming = false;
}

// Atiende clientes nuevos (no bloquea si no hay ninguno)
static void atender_nuevos_clientes() {
  WiFiClient nuevo = http_server.available();   // devuelve de inmediato
  if (!nuevo) return;

  String linea = leer_peticion(nuevo);
  Serial.printf("[CAM] Peticion: '%s'\n", linea.c_str());

  bool es_stream = linea.startsWith("GET /stream") ||
                   linea.startsWith("GET / ") || linea.startsWith("GET /?");

  if (!es_stream) {
    nuevo.print("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n");
    nuevo.stop();
    return;
  }
  if (streaming && stream_client.connected()) {
    nuevo.print("HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\nCamara ocupada");
    nuevo.stop();
    return;
  }

  cerrar_cliente();
  stream_client = nuevo;
  stream_client.setNoDelay(true);
  stream_client.print(HDR_STREAM);
  streaming = true;
  stream_frames = 0;
  stream_t0 = millis();
  Serial.println("[CAM] Cliente conectado: inicia streaming");
}

// Envia un frame al cliente. Siempre devuelve el buffer con fb_return().
static bool enviar_frame() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[CAM] esp_camera_fb_get fallo");
    return true;   // fallo transitorio de captura: se reintenta
  }

  bool ok = false;
  if (fb->len > 2 && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8) {
    char cab[96];
    int n = snprintf(cab, sizeof(cab),
                     "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                     (unsigned)fb->len);
    ok = stream_client.write((const uint8_t *)cab, n) == (size_t)n &&
         stream_client.write(fb->buf, fb->len) == fb->len &&
         stream_client.write((const uint8_t *)"\r\n", 2) == 2;
  } else {
    ok = true;   // frame corrupto: se descarta, sin cortar el stream
  }

  esp_camera_fb_return(fb);   // liberar SIEMPRE el buffer (evita fuga)
  if (ok) stream_frames++;
  return ok;
}

void camera_ov2640_test_loop() {
  if (!cam_ok) {
    Serial.println("[CAM] Camara no inicializada. Reintentando...");
    camera_ov2640_init();
    delay(2000);
    return;
  }

  atender_nuevos_clientes();

  if (streaming) {
    if (!stream_client.connected() || !enviar_frame()) {
      uint32_t dt = millis() - stream_t0;
      Serial.printf("[CAM] Cliente desconectado | %u frames | %.1f FPS\n",
                    (unsigned)stream_frames, dt ? stream_frames * 1000.0f / dt : 0.0f);
      cerrar_cliente();
    }
    return;   // sin delay: maxima tasa de frames
  }

  // Sin cliente: no bloquea; solo informa por Serial cada 2 s
  static uint32_t ultimo_msg = 0;
  if (millis() - ultimo_msg >= 2000) {
    ultimo_msg = millis();
    Serial.printf("[CAM] Esperando cliente en http://192.168.4.1/ | WiFi clientes: %d | PSRAM libre %u\n",
                  WiFi.softAPgetStationNum(), ESP.getFreePsram());
  }
  delay(5); 
  //esotilinwowtilin
}


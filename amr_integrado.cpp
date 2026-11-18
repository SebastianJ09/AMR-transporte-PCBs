#include "amr_integrado.h"
#include "amr_pins.h"
#include "pcf_bus.h"
#include "pca_bus.h"
#include "motor_driver.h"
#include "ultrasonic.h"
#include "servo_cam.h"
#include "imu_mpu6050.h"
#include "camera_ov2640.h"
#include "camera_stream.h"
#include "web_dashboard.h"

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

// ---- Configuracion de Red y Servicios ----
static const char *AP_SSID = "AMR_ROBOT_CAM";
static const char *AP_PASS = "12345678";
static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_GW(192, 168, 4, 1);
static const IPAddress AP_MASK(255, 255, 255, 0);

static const IPAddress MQTT_BROKER_IP(192, 168, 4, 2);
static const uint16_t  MQTT_PORT = 1883;
static const char     *MQTT_TOPIC_SUB = "amr/comandos/movimiento";

static const uint16_t STREAM_PORTS[2] = { 81, 82 };

// ---- Servidores Web / WebSocket / MQTT ----
static AsyncWebServer webServer(80);
static AsyncWebSocket ws("/ws");
static WiFiClient     mqttWifiClient;
static PubSubClient   mqttClient(mqttWifiClient);

// ---- Estado compartido de Control y Navegacion ----
static volatile bool     g_auto_mode    = false;
static volatile char     g_manual_dir   = 'S';
static volatile uint32_t g_last_move_ms = 0;
static volatile uint32_t g_last_hb_ms   = 0;
static volatile bool     g_estop_latched = false;
static volatile bool     g_punto_b_halt  = false;

static char              g_last_qr[64]  = "";
static portMUX_TYPE      g_state_mux    = portMUX_INITIALIZER_UNLOCKED;

// Velocidades (0..255) y umbrales de distancia (cm)
static constexpr int   SPEED_FWD         = 165;
static constexpr int   SPEED_BACK        = 155;
static constexpr int   SPEED_TURN        = 175;
static constexpr float DIST_DANGER_CM    = 20.0f;
static constexpr float DIST_CAUTION_CM   = 40.0f;
static constexpr float DIST_CLEAR_CM     = 55.0f;

// Estado interno de navegacion autonoma no bloqueante
enum AutoPhase : uint8_t { AUTO_FWD = 0, AUTO_BRAKE = 1, AUTO_TURN = 2 };
static AutoPhase g_auto_phase   = AUTO_FWD;
static char      g_auto_turn_dir = 'R';
static uint32_t  g_phase_t0     = 0;

// Estado reportado a la interfaz web
static char        g_active_mot = 'S';
static const char *g_status_txt = "Detenido";
static bool        g_safety_ovr = false;
static MPUData     g_imu_data   = {};
static bool        g_imu_ok     = false;

// ----------------------------------------------------------------------------
//  Callback y tarea FreeRTOS para MQTT (Mosquitto en 192.168.4.2:1883)
// ----------------------------------------------------------------------------
static void mqtt_callback(char *topic, byte *payload, unsigned int length) {
  char msg[64];
  unsigned int copy_len = (length < sizeof(msg) - 1) ? length : (sizeof(msg) - 1);
  memcpy(msg, payload, copy_len);
  msg[copy_len] = '\0';

  Serial.println("\n========================================");
  Serial.printf("[MQTT RECIBIDO] Canal: %s\n", topic);
  Serial.printf("[MQTT RECIBIDO] Carga util: %s\n", msg);

  portENTER_CRITICAL(&g_state_mux);
  strncpy(g_last_qr, msg, sizeof(g_last_qr) - 1);
  g_last_qr[sizeof(g_last_qr) - 1] = '\0';
  if (strstr(msg, "PUNTO_B") != nullptr) {
    g_punto_b_halt = true;
    g_auto_mode    = false;
    g_manual_dir   = 'S';
  }
  portEXIT_CRITICAL(&g_state_mux);

  if (strstr(msg, "PUNTO_B") != nullptr) {
    Serial.println("[ESTADO AMR] Meta alcanzada (PUNTO B). Freno y espera de ruta.");
  } else {
    Serial.println("[ESTADO AMR] Codigo registrado. Fuera de secuencia critica.");
  }
  Serial.println("========================================\n");
}

static void mqtt_task(void * /*param*/) {
  mqttClient.setServer(MQTT_BROKER_IP, MQTT_PORT);
  mqttClient.setCallback(mqtt_callback);
  mqttClient.setSocketTimeout(1);

  uint32_t last_try = 0;
  for (;;) {
    if (WiFi.softAPgetStationNum() > 0) {
      if (!mqttClient.connected()) {
        uint32_t now = millis();
        if (now - last_try >= 3000) {
          last_try = now;
          if (mqttClient.connect("ESP32_AMR_Chassis")) {
            Serial.printf("[MQTT] Conectado a 192.168.4.2:1883 | Suscrito a '%s'\n", MQTT_TOPIC_SUB);
            mqttClient.subscribe(MQTT_TOPIC_SUB);
          }
        }
      } else {
        mqttClient.loop();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

// ----------------------------------------------------------------------------
//  Recepcion de comandos WebSocket desde vehiculo-preview-3sensores.html
// ----------------------------------------------------------------------------
static void handle_ws_json(const uint8_t *data, size_t len) {
  JsonDocument doc;
  if (deserializeJson(doc, data, len)) return;

  const char *cmd = doc["cmd"] | "";
  uint32_t now = millis();

  if (strcmp(cmd, "hb") == 0) {
    g_last_hb_ms = now;
  } else if (strcmp(cmd, "mode") == 0) {
    const char *v = doc["v"] | "manual";
    bool to_auto = (strcmp(v, "auto") == 0);
    portENTER_CRITICAL(&g_state_mux);
    g_auto_mode     = to_auto;
    g_manual_dir    = 'S';
    g_estop_latched = false;
    g_punto_b_halt  = false;
    g_auto_phase    = AUTO_FWD;
    portEXIT_CRITICAL(&g_state_mux);
    Serial.printf("[WS] Modo cambiado a: %s\n", to_auto ? "AUTONOMO" : "MANUAL");
  } else if (strcmp(cmd, "estop") == 0) {
    portENTER_CRITICAL(&g_state_mux);
    g_auto_mode     = false;
    g_manual_dir    = 'S';
    g_estop_latched = true;
    portEXIT_CRITICAL(&g_state_mux);
    Serial.println("[WS] PARADA DE EMERGENCIA activada");
  } else if (strcmp(cmd, "move") == 0) {
    const char *dir = doc["dir"] | "S";
    char d = dir[0] ? dir[0] : 'S';
    portENTER_CRITICAL(&g_state_mux);
    if (!g_auto_mode) {
      g_manual_dir    = d;
      g_last_move_ms  = now;
      g_last_hb_ms    = now;
      if (d != 'S') {
        g_estop_latched = false;
        g_punto_b_halt  = false;
      }
    }
    portEXIT_CRITICAL(&g_state_mux);
  }
}

static void on_ws_event(AsyncWebSocket * /*server*/, AsyncWebSocketClient * /*client*/,
                        AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    g_last_hb_ms = millis();
  } else if (type == WS_EVT_DISCONNECT) {
    if (ws.count() == 0 && !g_auto_mode) {
      g_manual_dir = 'S';
    }
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      handle_ws_json(data, len);
    }
  }
}

// ----------------------------------------------------------------------------
//  Logica de Traccion y Seguridad (ejecutada en el hilo principal junto al I2C)
// ----------------------------------------------------------------------------
static void apply_motion_char(char mot) {
  g_active_mot = mot;
  switch (mot) {
    case 'F': motor_driver_set(SPEED_FWD,   SPEED_FWD);   break;
    case 'B': motor_driver_set(-SPEED_BACK, -SPEED_BACK); break;
    case 'L': motor_driver_set(-SPEED_TURN, SPEED_TURN);  break;
    case 'R': motor_driver_set(SPEED_TURN,  -SPEED_TURN); break;
    default:  motor_driver_stop(); g_active_mot = 'S';    break;
  }
}

static void update_navigation() {
  float df = ultrasonic_get_cm(US_FRONT);
  float dl = ultrasonic_get_cm(US_LEFT);
  float dr = ultrasonic_get_cm(US_RIGHT);
  bool  sf = ultrasonic_ok(US_FRONT);

  bool front_danger  = sf && (df > 0.0f && df < DIST_DANGER_CM);
  bool front_caution = sf && (df > 0.0f && df < DIST_CAUTION_CM);

  uint32_t now = millis();
  bool auto_mode, estop, punto_b;
  char man_dir;
  uint32_t t_move, t_hb;

  portENTER_CRITICAL(&g_state_mux);
  auto_mode = g_auto_mode;
  estop     = g_estop_latched;
  punto_b   = g_punto_b_halt;
  man_dir   = g_manual_dir;
  t_move    = g_last_move_ms;
  t_hb      = g_last_hb_ms;
  portEXIT_CRITICAL(&g_state_mux);

  // 1. Parada por meta QR (PUNTO_B) o parada de emergencia
  if (punto_b) {
    apply_motion_char('S');
    g_safety_ovr = true;
    g_status_txt = "Meta PUNTO_B";
    return;
  }
  if (estop) {
    apply_motion_char('S');
    g_safety_ovr = true;
    g_status_txt = "Parada de emergencia";
    return;
  }

  // 2. Modo Autonomo (evasion con 3 sensores HC-SR04, sin delays bloqueantes)
  if (auto_mode) {
    g_safety_ovr = false;
    if (g_auto_phase == AUTO_FWD) {
      if (front_danger || front_caution) {
        apply_motion_char('S');
        g_auto_phase = AUTO_BRAKE;
        g_phase_t0   = now;
        // Elegir el flanco mas despejado
        float left_eval  = (dl > 0.0f) ? dl : 120.0f;
        float right_eval = (dr > 0.0f) ? dr : 120.0f;
        g_auto_turn_dir  = (left_eval >= right_eval) ? 'L' : 'R';
        g_status_txt     = "Obstáculo detectado";
      } else {
        apply_motion_char('F');
        g_status_txt = "Avanzando";
      }
    } else if (g_auto_phase == AUTO_BRAKE) {
      apply_motion_char('S');
      g_status_txt = "Obstáculo detectado";
      if (now - g_phase_t0 >= 150) {
        g_auto_phase = AUTO_TURN;
        g_phase_t0   = now;
      }
    } else if (g_auto_phase == AUTO_TURN) {
      apply_motion_char(g_auto_turn_dir);
      g_status_txt = "Esquivando";
      bool clear = (!sf || df >= DIST_CLEAR_CM) && (now - g_phase_t0 >= 380);
      bool timeout = (now - g_phase_t0 >= 950);
      if (clear || timeout) {
        g_auto_phase = AUTO_FWD;
      }
    }
    return;
  }

  // 3. Modo Manual (con watchdog de enlace y bloqueo de colision frontal)
  if (man_dir != 'S' && (now - t_move > 600 || (ws.count() > 0 && now - t_hb > 1500))) {
    man_dir = 'S';
    portENTER_CRITICAL(&g_state_mux);
    g_manual_dir = 'S';
    portEXIT_CRITICAL(&g_state_mux);
  }

  if (man_dir == 'F' && front_danger) {
    apply_motion_char('S');
    g_safety_ovr = true;
    g_status_txt = "Avance bloqueado";
  } else {
    apply_motion_char(man_dir);
    g_safety_ovr = false;
    g_status_txt = (man_dir == 'S') ? "Detenido" : "Manual";
  }
}

// ----------------------------------------------------------------------------
//  Envio de telemetria JSON por WebSocket (/ws) cada 200 ms
// ----------------------------------------------------------------------------
static void send_ws_telemetry() {
  if (ws.count() == 0) return;

  float df = ultrasonic_get_cm(US_FRONT);
  float dl = ultrasonic_get_cm(US_LEFT);
  float dr = ultrasonic_get_cm(US_RIGHT);
  bool  sf = ultrasonic_ok(US_FRONT);
  bool  sl = ultrasonic_ok(US_LEFT);
  bool  sr = ultrasonic_ok(US_RIGHT);

  int lvl = 0;
  if (df < 0.0f || df < DIST_DANGER_CM)       lvl = 2;
  else if (df < DIST_CAUTION_CM)              lvl = 1;

  char qr_copy[64];
  bool auto_mode;
  portENTER_CRITICAL(&g_state_mux);
  strncpy(qr_copy, g_last_qr, sizeof(qr_copy));
  qr_copy[sizeof(qr_copy) - 1] = '\0';
  auto_mode = g_auto_mode;
  portEXIT_CRITICAL(&g_state_mux);

  char mot_str[2] = { g_active_mot, '\0' };

  JsonDocument doc;
  doc["t"]    = "tel";
  doc["mode"] = auto_mode ? "auto" : "manual";
  doc["st"]   = g_status_txt;
  doc["dist"] = df;
  doc["dl"]   = dl;
  doc["dr"]   = dr;
  doc["sens"] = sf;
  doc["sl"]   = sl;
  doc["sr"]   = sr;
  doc["lvl"]  = lvl;
  doc["mot"]  = mot_str;
  doc["cam"]  = camera_ov2640_is_ready();
  doc["ovr"]  = g_safety_ovr;
  doc["up"]   = (uint32_t)(millis() / 1000);
  doc["qr"]   = qr_copy;
  if (g_imu_ok) {
    doc["temp"] = g_imu_data.temp_c;
  }

  char buf[384];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  ws.textAll(buf, n);
}

// ----------------------------------------------------------------------------
//  Inicializacion y Bucle Principal del Modo Integrado
// ----------------------------------------------------------------------------
void amr_integrado_setup() {
  Serial.println("\n============================================================");
  Serial.println("  AMR SISTEMA INTEGRADO: DASHBOARD + 2x STREAM + 3x US + MQTT");
  Serial.println("============================================================");

  // 1. Bus I2C unificado (SDA=13, SCL=14) y actuadores/sensores
  amr_i2c_begin();
  motor_driver_init();
  servo_cam_init();
  ultrasonic_init();
  g_imu_ok = imu_init();

  // 2. Camara OV2640 (solo sensor; el AP y los servidores se levantan aqui)
  camera_ov2640_init_sensor();

  // 3. Wi-Fi Access Point (192.168.4.1)
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  if (WiFi.softAP(AP_SSID, AP_PASS)) {
    WiFi.setSleep(false);
    Serial.printf("[WIFI] AP '%s' activo | IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("[WIFI] Error al iniciar Access Point");
  }

  // 4. Montaje opcional de LittleFS (respaldo) y servidor AsyncWebServer (puerto 80)
  LittleFS.begin(false);

  ws.onEvent(on_ws_event);
  webServer.addHandler(&ws);

  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html; charset=utf-8", WEB_DASHBOARD_HTML);
  });
  webServer.on("/vehiculo-preview-3sensores.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html; charset=utf-8", WEB_DASHBOARD_HTML);
  });
  webServer.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "No encontrado");
  });
  webServer.begin();
  Serial.println("[WEB] Dashboard activo en http://192.168.4.1/ (WebSocket en /ws)");

  // 5. Servidores de stream MJPEG simultaneos: 81 (Web) y 82 (Python/OpenCV)
  if (camera_ov2640_is_ready()) {
    camera_stream_start(STREAM_PORTS, 2);
  }

  // 6. Tarea FreeRTOS para suscripcion MQTT no bloqueante
  xTaskCreatePinnedToCore(mqtt_task, "mqtt_task", 4096, nullptr, 2, nullptr, 0);

  g_last_hb_ms = millis();
  Serial.println("[SISTEMA] Todo listo.\n");
}

void amr_integrado_loop() {
  // 1. Muestreo no bloqueante de los 3 sensores HC-SR04 (PCF8574 P4..P6 + ISR en 32/33/15)
  ultrasonic_update();

  // 2. Control de traccion y navegacion cada 20 ms (50 Hz)
  static uint32_t last_ctrl_ms = 0;
  uint32_t now = millis();
  if (now - last_ctrl_ms >= 20) {
    last_ctrl_ms = now;
    update_navigation();
  }

  // 3. Lectura de IMU y envio de telemetria WebSocket cada 200 ms (5 Hz)
  static uint32_t last_tel_ms = 0;
  if (now - last_tel_ms >= 200) {
    last_tel_ms = now;
    g_imu_ok = imu_read(g_imu_data);
    send_ws_telemetry();
    ws.cleanupClients();
  }

  // 4. Resumen periodico por Monitor Serie cada 1000 ms
  static uint32_t last_log_ms = 0;
  if (now - last_log_ms >= 1000) {
    last_log_ms = now;
    Serial.printf("[AMR] Modo:%s | Est:%s | F:%.1f I:%.1f D:%.1f cm | Mot:%c | Temp:%.1fC | WS:%u\n",
                  g_auto_mode ? "AUTO" : "MAN", g_status_txt,
                  ultrasonic_get_cm(US_FRONT), ultrasonic_get_cm(US_LEFT), ultrasonic_get_cm(US_RIGHT),
                  g_active_mot, g_imu_data.temp_c, (unsigned)ws.count());
  }

  delay(2);
}

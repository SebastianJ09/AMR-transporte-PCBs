/**
 * ============================================================================
 * PROGRAMA PRINCIPAL - Robot AMR "Alternativa 3" (Freenove ESP32-WROVER)
 * Arnes de pruebas unitarias seleccionable con #define TEST_MODE
 * ============================================================================
 *
 * MATRIZ MAESTRA DE PINES Y CABLEADO (definida en include/amr_pins.h)
 * ----------------------------------------------------------------------------
 *  Bus I2C compartido (3.3 V)   SDA -> GPIO 13 | SCL -> GPIO 14
 *     - PCF8574AT (0x3F, A0..A2 abiertos), VCC -> 3V3
 *     - MPU-6050  (0x68, AD0 -> GND),      VCC -> 3V3
 *  Traccion TB6612FNG
 *     - PWMA -> GPIO 32 | PWMB -> GPIO 33 (1000 Hz, LEDC canales 2/3)
 *     - AIN1 -> PCF P4 (pin 11) | AIN2 -> PCF P5 (pin 12)
 *     - BIN1 -> PCF P6 (pin 13) | BIN2 -> PCF P7 (pin 14)
 *     - STBY -> 3V3 | VM -> LiPo (+) 7.4 V | VCC -> LM2596 OUT+ 5.0 V
 *     - AO1/AO2 -> motores izquierdos | BO1/BO2 -> motores derechos
 *  HC-SR04 frontal (unico sensor)
 *     - TRIG -> GPIO 15 | ECHO -> 1k serie -> GPIO 2 -> 2k a GND (OBLIGATORIO)
 *     - VCC -> LM2596 OUT+ 5.0 V
 *  Matriz FC16 8x8 (MAX7219, orientacion GENERIC_HW)
 *     - DIN -> GPIO 23 | CLK -> GPIO 18 | CS -> GPIO 5 | VCC -> 5.0 V
 *  Servo SG90 (paneo)   PWM -> GPIO 12 | VCC -> 5.0 V
 *  Camara OV2640        bus fijo de la placa: 4,5,18,19,21,22,23,25,26,27,34,35,36,39
 *  Masa comun: GND de LiPo, LM2596, ESP32, TB6612FNG y sensores unidos en el riel.
 *
 * REGLAS DE ALIMENTACION
 *  - Con USB conectado al PC: NO unir la salida 5 V del LM2596 al pin 5V del ESP32.
 *  - Autonomo: LM2596 OUT+ (5.0 V calibrados) -> pin 5V del ESP32, sin USB.
 *
 * ADVERTENCIAS DE COMPATIBILIDAD
 *  - La matriz FC16 (23/18/5) comparte pines con la camara: solo se prueba en
 *    TEST_FC16_MATRIX. Ningun modo que inicia la camara inicializa la matriz.
 *    Con la camara activa no quedan GPIO libres para la matriz; para integrarla
 *    habria que manejar el MAX7219 por los pines P0..P2 libres del PCF8574AT.
 *  - GPIO 2, 12 y 15 son pines de arranque (ver include/amr_pins.h).
 *
 * MODIFICACIONES REALIZADAS EN ESTA VERSION (10/11/2026)
 *  1. include/amr_pins.h (nuevo): mapa unico de pines + amr_i2c_begin().
 *  2. motor_driver: pines desde amr_pins.h, direcciones por PCF8574AT (P4..P7),
 *     PWM 1000 Hz en 32/33 (LEDC 2/3), deteccion de fallo I2C y prueba unitaria.
 *  3. ultrasonic: sensor unico en TRIG 15 / ECHO 2 con divisor documentado,
 *     tiempo muerto de 30 ms entre lecturas y prueba unitaria.
 *  4. imu_mpu6050: usa el bus I2C 13/14, verifica WHO_AM_I, escaneo I2C y
 *     prueba unitaria con unidades fisicas (g, dps).
 *  5. fc16_matrix: CS movido de GPIO 15 a GPIO 5, orientacion GENERIC_HW y
 *     patrones asimetricos para verificar la rotacion.
 *  6. servo_cam: movido de GPIO 15 a GPIO 12, timer LEDC 2 exclusivo.
 *  7. Este main: el codigo de camara + MQTT (Mosquitto/OpenCV) y el main de
 *     navegacion A2 se conservan SIN CAMBIOS como modos 7 y 8. Copias exactas
 *     en backup/ (fuera de src/, no se compilan).
 * ============================================================================
 */

#include <Arduino.h>
#include "amr_pins.h"

// ---------------------------------------------------------------
// Selector de pruebas: solo el modo elegido se compila y ejecuta.
// Tambien puede fijarse con -DTEST_MODE=n en platformio.ini
// ---------------------------------------------------------------
#define TEST_NONE         0
#define TEST_FC16_MATRIX  1   // matriz sola (camara apagada)
#define TEST_ULTRASONIC   2   // HC-SR04 frontal
#define TEST_SERVO_CAM    3   // SG90 barrido
#define TEST_IMU_MPU6050  4   // MPU-6050 + escaneo I2C
#define TEST_MOTOR        5   // TB6612FNG + PCF8574AT (ruedas en el aire)
#define TEST_CAMERA       6   // camara OV2640 + stream MJPEG
#define TEST_CAMERA_MQTT  7   // camara AP + receptor MQTT (Mosquitto/OpenCV) - original
#define TEST_NAV_ALT3     8   // navegacion autonoma A2 + camara - original
#define TEST_INTEGRADO    9   // sistema unificado: Dashboard Web + 2x Stream (81/82) + 3x US + PCA9685 + PCF8574 + MQTT

#ifndef TEST_MODE
#define TEST_MODE TEST_INTEGRADO
#endif

#if TEST_MODE == TEST_INTEGRADO
// ============================================================================
//  MODO 9 - SISTEMA INTEGRADO FINAL (Dashboard Web + 81/82 Stream + MQTT + I2C)
// ============================================================================
#include "amr_integrado.h"

void setup() {
  Serial.begin(115200);
  delay(1000);
  amr_integrado_setup();
}

void loop() {
  amr_integrado_loop();
}

#elif TEST_MODE == TEST_CAMERA_MQTT
// ============================================================================
//  MODO 7 - CAMARA AP + RECEPTOR MQTT (Mosquitto / OpenCV en la PC)
//  >>> CODIGO ORIGINAL CONSERVADO SIN MODIFICACIONES <<<
//  Copia exacta: backup/main_camara_mqtt_opencv.cpp.txt
// ============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "camera_ov2640.h"

// IP asignada a tu laptop por el ESP32 al conectarse a AMR_ROBOT_CAM
const IPAddress mqtt_broker_ip(192, 168, 4, 2);
const int mqtt_port = 1883;
const char* mqtt_topic_sub = "amr/comandos/movimiento";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long last_mqtt_check = 0;

// Callback: se dispara al recibir un mensaje desde Python
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String mensaje = "";
  for (unsigned int i = 0; i < length; i++) {
    mensaje += (char)payload[i];
  }

  Serial.println("\n========================================");
  Serial.print("[MQTT RECIBIDO] Canal: ");
  Serial.println(topic);
  Serial.print("[MQTT RECIBIDO] Carga util: ");
  Serial.println(mensaje);

  // Toma de decisiones
  if (mensaje.indexOf("PUNTO_B") >= 0) {
    Serial.println("[ESTADO AMR] Meta alcanzada (PUNTO B). Freno y espera de ruta.");
  } else {
    Serial.println("[ESTADO AMR] Codigo registrado. Fuera de secuencia critica.");
  }
  Serial.println("========================================\n");
}

// Reconexión sin bloquear el servidor de video
void verificarMQTT() {
  // Solo intentar si hay un cliente conectado a la red WiFi del robot
  if (WiFi.softAPgetStationNum() == 0) {
    return;
  }

  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - last_mqtt_check > 3000) { // Evaluar cada 3 segundos
      last_mqtt_check = now;
      Serial.print("[MQTT] Buscando broker Mosquitto en 192.168.4.2:1883... ");
      
      mqttClient.setSocketTimeout(1); // 1 segundo máximo para no pausar el video

      if (mqttClient.connect("ESP32_AMR_Chassis")) {
        Serial.println("Conectado con exito.");
        mqttClient.subscribe(mqtt_topic_sub);
        Serial.print("[MQTT] Suscrito al canal: ");
        Serial.println(mqtt_topic_sub);
      } else {
        Serial.print("Pendiente (rc=");
        Serial.print(mqttClient.state());
        Serial.println("). Esperando servicio en la PC...");
      }
    }
  } else {
    mqttClient.loop();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==================================================");
  Serial.println("  AMR SISTEMA EMBEBIDO: CAMARA AP + RECEPTOR MQTT ");
  Serial.println("==================================================");

  // Inicializa cámara OV2640 y levanta la red AMR_ROBOT_CAM (192.168.4.1)
  camera_ov2640_init();

  // Configura parámetros MQTT
  mqttClient.setServer(mqtt_broker_ip, mqtt_port);
  mqttClient.setCallback(mqttCallback);

  Serial.println("[SISTEMA] Red y Servidor Web inicializados.");
}

void loop() {
  // Streaming de video multipart continuo
  camera_ov2640_test_loop();

  // Chequeo de telemetría MQTT
  verificarMQTT();

  delay(5);
}
#elif TEST_MODE == TEST_NAV_ALT3
// ============================================================================
//  MODO 8 - NAVEGACION AUTONOMA A2 + CAMARA (main del 9/11/2026)
//  >>> CODIGO ORIGINAL CONSERVADO SIN MODIFICACIONES <<<
//  Copia exacta: backup/main_A2_navegacion_camara.cpp.txt
// ============================================================================

/**
 * ============================================================================
Codigo main.cpp 9/11/2926 antes de integrar la cámara
 * PROGRAMA PRINCIPAL: Navegacion Autonoma AMR + Camara AP (OV2640)
 * ============================================================================
 * ARQUITECTURA INTEGRADA:
 *   - ESP32-WROVER-CAM: Crea Access Point "AMR_ROBOT_CAM" (IP 192.168.4.1)
 *   - Camara OV2640: Transmision MJPEG no bloqueante.
 *   - Bus I2C (GPIO 13/14): PCF8574AT (0x3F) + MPU-6050 (0x68)
 *   - Traccion: TB6612FNG (PWM en 32/33 + Direccion I2C)
 *   - Percepcion Frontal: HC-SR04 en GPIO 15 (TRIG) y GPIO 2 (ECHO con divisor)
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>

#include "motor_driver.h"
#include "ultrasonic.h"
#include "imu_mpu6050.h"
#include "camera_ov2640.h"

// Pines I2C (libres de la camara)
#define PIN_I2C_SDA 13
#define PIN_I2C_SCL 14

static unsigned long last_telemetry_time = 0;
static const unsigned long TELEMETRY_INTERVAL_MS = 500;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==================================================");
  Serial.println("  INICIALIZANDO ROBOT AMR + CAMARA (ALT 3) ");
  Serial.println("==================================================");

  // 1. Inicializar bus I2C y Subsistemas Fisicos
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);  
  
  motor_driver_init();
  ultrasonic_init();
  imu_init();

  // 2. Inicializar Camara y servidor AP
  camera_ov2640_init();

  Serial.println("[SISTEMA] Listo. Conectate a la red 'AMR_ROBOT_CAM' y abre http://192.168.4.1/\n");
}

void loop() {
  // Atiende las peticiones HTTP del stream de video (no bloqueante)
  camera_ov2640_test_loop();

  // Lectura de percepcion frontal
  float dist_cm = ultrasonic_read_cm();

  // Lectura inercial
  MPUData imu;
  bool imu_ok = imu_read(imu);

  // Maquina de estados reactiva de navegacion
  if (dist_cm > 0.0f && dist_cm < 20.0f) {
    motor_driver_stop();
    delay(150);
    motor_driver_set(-180, 180); // Giro sobre su eje a la derecha
    delay(450);
    motor_driver_stop();
    delay(100);
  } else {
    motor_driver_set(160, 160); // Avance normal
  }

  // Telemetria por monitor serie (reducida para no ahogar la terminal)
  unsigned long now = millis();
  if (now - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_time = now;
    if (imu_ok) {
      Serial.printf("[NAVEGACION] Dist: %5.1f cm | Temp IMU: %.1f C\n", dist_cm, imu.temp_c);
    }
  }

  // Pequena pausa para ceder tiempo a las tareas de Wi-Fi en segundo plano
  delay(10);
}

#else
// ============================================================================
//  ARNES DE PRUEBAS UNITARIAS (modos 0..6): un solo periferico por compilacion
// ============================================================================

#if TEST_MODE == TEST_FC16_MATRIX
  #include "fc16_matrix.h"
  #define DUT_NAME      "Matriz FC16 (DIN23 CLK18 CS5)"
  #define DUT_INIT()    fc16_matrix_init()
  #define DUT_LOOP()    fc16_matrix_test_loop()
#elif TEST_MODE == TEST_ULTRASONIC
  #include "ultrasonic.h"
  #define DUT_NAME      "HC-SR04 frontal (TRIG15 ECHO2)"
  #define DUT_INIT()    ultrasonic_init()
  #define DUT_LOOP()    ultrasonic_test_loop()
#elif TEST_MODE == TEST_SERVO_CAM
  #include "servo_cam.h"
  #define DUT_NAME      "Servo SG90 (GPIO12)"
  #define DUT_INIT()    servo_cam_init()
  #define DUT_LOOP()    servo_cam_test_loop()
#elif TEST_MODE == TEST_IMU_MPU6050
  #include "imu_mpu6050.h"
  #define DUT_NAME      "IMU MPU-6050 (I2C 13/14, 0x68)"
  #define DUT_INIT()    imu_init()
  #define DUT_LOOP()    imu_mpu6050_test_loop()
#elif TEST_MODE == TEST_MOTOR
  #include "motor_driver.h"
  #define DUT_NAME      "TB6612FNG + PCF8574AT (PWM 32/33, I2C 0x3F)"
  #define DUT_INIT()    motor_driver_init()
  #define DUT_LOOP()    motor_driver_test_loop()
#elif TEST_MODE == TEST_CAMERA
  #include "camera_ov2640.h"
  #define DUT_NAME      "Camara OV2640 + stream MJPEG"
  #define DUT_INIT()    camera_ov2640_init()
  #define DUT_LOOP()    camera_ov2640_test_loop()
#elif TEST_MODE == TEST_NONE
  #define DUT_NAME      "ninguno"
  #define DUT_INIT()
  #define DUT_LOOP()    delay(1000)
#else
  #error "TEST_MODE invalido"
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("\n--- TEST_MODE=%d: %s ---\n", TEST_MODE, DUT_NAME);
  DUT_INIT();
}

void loop() {
  DUT_LOOP();
}

#endif // TEST_MODE

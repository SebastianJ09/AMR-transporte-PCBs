/**
 * ============================================================================
 * MODULO: Unidad de Medicion Inercial (IMU MPU-6050)
 * ============================================================================
 * CONEXIONES FISICAS:
 *   - MPU-6050 VCC --> Pin 3V3 del ESP32
 *   - MPU-6050 GND --> Riel comun GND
 *   - MPU-6050 SDA --> ESP32 GPIO 13 (Compartido con PCF8574AT)
 *   - MPU-6050 SCL --> ESP32 GPIO 14 (Compartido con PCF8574AT)
 *   - MPU-6050 AD0 --> Riel comun GND (Fija la direccion I2C en 0x68)
 *
 * CONSIDERACIONES Y PRECAUCIONES:
 *   1. BUS I2C COMPARTIDO: Convive en las mismas lineas fisicas (GPIO 13 y 14)
 *      junto al expansor PCF8574AT sin colisionar gracias a sus direcciones
 *      distintas (0x68 vs 0x3F).
 *   2. DESPERTAR DEL SENSOR: El MPU-6050 inicia por defecto en modo de bajo
 *      consumo (Sleep). La funcion imu_init() escribe 0x00 en el registro
 *      PWR_MGMT_1 (0x6B) para habilitar el oscilador interno.
 *   3. PINES I2C: se toman de amr_pins.h (AMR_I2C_SDA = 13, AMR_I2C_SCL = 14).
 *      imu_init() abre el bus por si mismo, asi la prueba unitaria funciona
 *      aislada; si main.cpp ya lo abrio, la llamada no tiene efecto.
 *   4. PULL-UPS: las placas GY-521 y PCF8574 ya traen resistencias pull-up.
 *      Con ambas placas en paralelo el bus queda con ~2-5 kOhm: correcto a
 *      100 kHz. No agregar mas.
 *   5. VERIFICACION: imu_init() lee WHO_AM_I (0x75). Un MPU-6050 original
 *      responde 0x68; clones (MPU-6500/9250) responden otro valor pero suelen
 *      ser compatibles con estos registros.
 * ============================================================================
 */

#include "imu_mpu6050.h"
#include "amr_pins.h"
#include <Wire.h>

#define MPU6050_ADDR AMR_MPU6050_ADDR

// Lee un entero de 16 bits big-endian (byte alto primero). Las dos lecturas
// van en sentencias separadas: en "(Wire.read() << 8) | Wire.read()" C++ no
// garantiza cual de las dos llamadas se ejecuta primero.
static int16_t read16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

static int read_reg(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint16_t)MPU6050_ADDR, (size_t)1, true) != 1) return -1;
  return Wire.read();
}

bool imu_init() {
  amr_i2c_begin();   // GPIO 13/14 (idempotente)

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // Registro de gestion de energia PWR_MGMT_1
  Wire.write(0x00); // Salir de modo reposo (despertar oscilador)
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    int who = read_reg(0x75);
    Serial.printf("[MPU-6050] IMU detectada y activa en direccion 0x68 (WHO_AM_I=0x%02X).\n", who);
    return true;
  } else {
    Serial.printf("[MPU-6050] Error al inicializar IMU (codigo: %d)\n", error);
    return false;
  }
}

bool imu_read(MPUData &data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Registro base: ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  // Solicitar 14 bytes consecutivos (Accel X/Y/Z, Temp, Gyro X/Y/Z)
  // Tipos explicitos para evitar la sobrecarga ambigua de requestFrom en ESP32
  Wire.requestFrom((uint16_t)MPU6050_ADDR, (size_t)14, true);
  if (Wire.available() < 14) {
    return false;
  }

  data.ax = read16();
  data.ay = read16();
  data.az = read16();

  int16_t raw_temp = read16();
  data.temp_c = (raw_temp / 340.0f) + 36.53f;

  data.gx = read16();
  data.gy = read16();
  data.gz = read16();

  return true;
}

// Escanea el bus una vez: debe mostrar 0x3F (PCF8574AT) y 0x68 (MPU-6050)
static void i2c_scan() {
  Serial.printf("[I2C] Escaneo en SDA=%u SCL=%u:", AMR_I2C_SDA, AMR_I2C_SCL);
  uint8_t n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); n++; }
  }
  Serial.printf(n ? "  (%u dispositivos)\n" : " ninguno\n", n);
}

void imu_mpu6050_test_loop() {
  static bool escaneado = false;
  if (!escaneado) { i2c_scan(); escaneado = true; }

  MPUData d;
  if (imu_read(d)) {
    // Conversion con los rangos por defecto (+-2 g, +-250 dps)
    Serial.printf("[MPU-6050] a(g) X:%5.2f Y:%5.2f Z:%5.2f | g(dps) X:%6.1f Y:%6.1f Z:%6.1f | %.1f C\n",
                  d.ax / 16384.0f, d.ay / 16384.0f, d.az / 16384.0f,
                  d.gx / 131.0f, d.gy / 131.0f, d.gz / 131.0f, d.temp_c);
  } else {
    Serial.println("[MPU-6050] Lectura fallida, reintentando init...");
    imu_init();
  }
  delay(200);
}

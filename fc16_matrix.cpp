/**
 * ============================================================================
 * MODULO: Matriz LED 8x8 FC16 (MAX7219) - 1 modulo
 * ============================================================================
 * CONEXIONES FISICAS:
 *   - FC16 VCC --> Salida OUT+ (5.0V) del LM2596 (NO desde 3V3 del ESP32)
 *   - FC16 GND --> Riel comun GND
 *   - FC16 DIN --> ESP32 GPIO 23
 *   - FC16 CLK --> ESP32 GPIO 18
 *   - FC16 CS  --> ESP32 GPIO 5
 *
 * ORIENTACION: GENERIC_HW (confirmado)
 *   Se replican los parametros que la libreria MD_MAX72XX asigna a GENERIC_HW
 *   (ver MD_MAX72xx.cpp):  _hwDigRows = false, _hwRevCols = true, _hwRevRows = false
 *   -> cada registro DIGITn del MAX7219 es una COLUMNA, el orden de columnas
 *      esta invertido y el de filas no. Si la imagen sale girada o en espejo,
 *      ajusta FC16_DIG_ROWS / FC16_REV_COLS / FC16_REV_ROWS (FC16_HW = 1/1/0).
 *   El envio se hace con un driver propio (bit-bang lento + interrupciones
 *   bloqueadas por trama): con logica de 3.3 V contra un MAX7219 a 5 V resulto
 *   mas fiable que el shiftOut rapido de la libreria.
 *
 * CONSIDERACIONES Y PRECAUCIONES:
 *   1. CONFLICTO CON LA CAMARA OV2640: GPIO 23 (HREF), 18 (Y4) y 5 (Y3) son
 *      lineas de la camara. La matriz SOLO puede usarse con la camara apagada
 *      (TEST_MODE = TEST_FC16_MATRIX). No llamar fc16_matrix_init() en modos
 *      que inicien la camara.
 *   2. NIVELES LOGICOS: el MAX7219 a 5 V pide VIH >= 3.5 V y el ESP32 entrega
 *      3.3 V (margen justo). Si aparecen LEDs aleatorios: cables cortos, GND
 *      comun, o un buffer 74HCT125 entre ESP32 y DIN/CLK/CS.
 *   3. CONSUMO: 64 LEDs con brillo alto pueden pasar de 300 mA. Alimentar desde
 *      el LM2596, nunca desde el pin 3V3.
 * ============================================================================
 */

#include "fc16_matrix.h"
#include "amr_pins.h"

// ---- Parametros de orientacion (GENERIC_HW de MD_MAX72XX) ----
#define FC16_DIG_ROWS 0   // 0: DIGITn = columna | 1: DIGITn = fila
#define FC16_REV_COLS 1   // invierte el orden de columnas
#define FC16_REV_ROWS 0   // invierte el orden de filas

// Registros MAX7219
static const uint8_t REG_DECODE    = 0x09;
static const uint8_t REG_INTENSITY = 0x0A;
static const uint8_t REG_SCANLIMIT = 0x0B;
static const uint8_t REG_SHUTDOWN  = 0x0C;
static const uint8_t REG_TEST      = 0x0F;

// Patrones (fila 0 arriba, bit 7 = columna 0 izquierda). Asimetricos para
// verificar la orientacion de un vistazo.
static const uint8_t PAT_FLECHA_ARRIBA[8] = {
  0b00011000, 0b00111100, 0b01111110, 0b11011011,
  0b00011000, 0b00011000, 0b00011000, 0b00011000
};
static const uint8_t PAT_LETRA_F[8] = {
  0b01111110, 0b01000000, 0b01000000, 0b01111100,
  0b01000000, 0b01000000, 0b01000000, 0b00000000
};
static const uint8_t PAT_ESQUINA[8] = {   // un solo LED: fila 0, columna 0
  0b10000000, 0, 0, 0, 0, 0, 0, 0
};

static void send_byte(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    digitalWrite(AMR_FC16_DIN, (b >> i) & 1);
    delayMicroseconds(10);
    digitalWrite(AMR_FC16_CLK, HIGH);
    delayMicroseconds(10);
    digitalWrite(AMR_FC16_CLK, LOW);
    delayMicroseconds(10);
  }
}

static void write_reg(uint8_t reg, uint8_t val) {
  // Interrupciones bloqueadas solo durante la trama (~0.5 ms)
  noInterrupts();
  digitalWrite(AMR_FC16_CS, LOW);
  delayMicroseconds(5);
  send_byte(reg);
  send_byte(val);
  delayMicroseconds(5);
  digitalWrite(AMR_FC16_CS, HIGH);   // el MAX7219 enclava los datos en este flanco
  interrupts();
  delayMicroseconds(5);
}

static void write_config() {
  write_reg(REG_TEST, 0x00);       // modo test apagado
  write_reg(REG_DECODE, 0x00);     // sin decodificacion BCD
  write_reg(REG_SCANLIMIT, 0x07);  // 8 digitos activos
  write_reg(REG_SHUTDOWN, 0x01);   // operacion normal
}

void fc16_matrix_init() {
  pinMode(AMR_FC16_DIN, OUTPUT);
  pinMode(AMR_FC16_CLK, OUTPUT);
  pinMode(AMR_FC16_CS, OUTPUT);
  digitalWrite(AMR_FC16_CLK, LOW);
  digitalWrite(AMR_FC16_DIN, LOW);
  digitalWrite(AMR_FC16_CS, HIGH);
  delay(50);

  write_config();
  write_reg(REG_INTENSITY, 0x04);  // brillo moderado
  fc16_matrix_clear();
  Serial.printf("[FC16] Matriz inicializada (GENERIC_HW) DIN:%u CLK:%u CS:%u\n",
                AMR_FC16_DIN, AMR_FC16_CLK, AMR_FC16_CS);
}

void fc16_matrix_clear() {
  for (uint8_t r = 1; r <= 8; r++) write_reg(r, 0x00);
}

void fc16_matrix_all_off() { fc16_matrix_clear(); }

void fc16_matrix_all_on() {
  for (uint8_t r = 1; r <= 8; r++) write_reg(r, 0xFF);
}

void fc16_matrix_set_brightness(uint8_t level) {
  write_reg(REG_INTENSITY, level > 15 ? 15 : level);
}

// rows[f]: fila f (0 = arriba), bit 7 = columna 0 (izquierda)
void fc16_matrix_show(const uint8_t rows[8]) {
  for (uint8_t d = 0; d < 8; d++) {          // d = registro DIGITd
    uint8_t val = 0;
    for (uint8_t k = 0; k < 8; k++) {        // k = bit del registro
#if FC16_DIG_ROWS
      uint8_t fila = FC16_REV_ROWS ? 7 - d : d;
      uint8_t col  = FC16_REV_COLS ? k : 7 - k;
#else
      uint8_t col  = FC16_REV_COLS ? 7 - d : d;
      uint8_t fila = FC16_REV_ROWS ? 7 - k : k;
#endif
      if ((rows[fila] >> (7 - col)) & 1) val |= (1 << k);
    }
    write_reg(d + 1, val);
  }
}

void fc16_matrix_test_loop() {
  // Reconfigura en cada ciclo: si un falso contacto reseteo el MAX7219, despierta
  write_config();

  Serial.println("[FC16] Display test por hardware (2 s)");
  write_reg(REG_TEST, 0x01);
  delay(2000);
  write_reg(REG_TEST, 0x00);

  Serial.println("[FC16] TODOS ENCENDIDOS (por datos)");
  fc16_matrix_all_on();
  delay(2000);

  Serial.println("[FC16] TODOS APAGADOS");
  fc16_matrix_all_off();
  delay(1000);

  Serial.println("[FC16] Esquina superior izquierda (1 LED)");
  fc16_matrix_show(PAT_ESQUINA);
  delay(2000);

  Serial.println("[FC16] Flecha hacia ARRIBA");
  fc16_matrix_show(PAT_FLECHA_ARRIBA);
  delay(2000);

  Serial.println("[FC16] Letra F (no debe verse en espejo)");
  fc16_matrix_show(PAT_LETRA_F);
  delay(2000);

  fc16_matrix_all_off();
  delay(1000);
}

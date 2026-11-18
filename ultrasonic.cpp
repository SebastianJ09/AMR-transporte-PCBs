#include "ultrasonic.h"
#include "pcf_bus.h"
#include "driver/gpio.h"

static const uint32_t TIMEOUT_US    = 25000;
static const uint32_t GAP_US        = 30000;
static const uint8_t  INVALID_LIMIT = 5;
static const float    MIN_CM = 2.0f, MAX_CM = 400.0f;
static const uint8_t  ORDER[] = { US_FRONT, US_LEFT, US_FRONT, US_RIGHT };
static const uint8_t  ORDER_LEN = sizeof(ORDER);

static volatile uint32_t s_rise[US_COUNT];
static volatile uint32_t s_width[US_COUNT];
static volatile bool     s_done[US_COUNT];
static volatile int8_t   s_active = -1;

static void IRAM_ATTR echo_isr(void *arg) {
  uint8_t i = (uint8_t)(uintptr_t)arg;
  if (s_active != (int8_t)i) return;
  uint32_t now = micros();
  if (gpio_get_level((gpio_num_t)AMR_US_ECHO_PIN[i])) {
    s_rise[i] = now;
  } else if (s_rise[i]) {
    s_width[i] = now - s_rise[i];
    s_done[i] = true;
    s_rise[i] = 0;
  }
}

static float   s_cm[US_COUNT];
static bool    s_has[US_COUNT];
static uint8_t s_bad[US_COUNT];

static void register_invalid(uint8_t i) {
  if (s_bad[i] < 255) s_bad[i]++;
  if (s_bad[i] >= INVALID_LIMIT) s_has[i] = false;
}

static void register_width(uint8_t i, uint32_t width_us) {
  float cm = width_us * 0.0343f / 2.0f;
  if (cm < MIN_CM || cm > MAX_CM) { register_invalid(i); return; }
  s_bad[i] = 0;
  s_cm[i]  = s_has[i] ? (0.6f * cm + 0.4f * s_cm[i]) : cm;
  s_has[i] = true;
}

static bool fire(uint8_t i) {
  s_done[i] = false;
  s_rise[i] = 0;
  s_active = (int8_t)i;
  uint8_t bit = (uint8_t)(1u << AMR_PCF_TRIG[i]);
  bool ok = amr_pcf_write_bits(bit, bit);   // P4, P5 o P6 en HIGH
  delayMicroseconds(10);
  amr_pcf_write_bits(bit, 0);               // P4, P5 o P6 en LOW
  return ok;
}

static bool pcf_warned = false;

void ultrasonic_init() {
  amr_i2c_begin();
  bool ok = amr_pcf_write_bits(AMR_PCF_TRIG_MASK, 0x00);
  for (uint8_t i = 0; i < US_COUNT; i++) {
    pinMode(AMR_US_ECHO_PIN[i], INPUT);
    s_cm[i] = 0; s_has[i] = false; s_bad[i] = 0;
    attachInterruptArg(AMR_US_ECHO_PIN[i], echo_isr, (void *)(uintptr_t)i, CHANGE);
  }
  Serial.printf("[HC-SR04 x3] PCF8574 (0x%02X) %s | TRIG P%u/P%u/P%u | ECHO GPIO %u/%u/%u (F/I/D)\n",
                amr_pcf_addr(), ok ? "OK" : "SIN RESPUESTA",
                AMR_PCF_TRIG[0], AMR_PCF_TRIG[1], AMR_PCF_TRIG[2],
                AMR_US_ECHO_PIN[0], AMR_US_ECHO_PIN[1], AMR_US_ECHO_PIN[2]);
}

enum { ST_IDLE, ST_WAIT };
static uint8_t  st = ST_IDLE, idx = 0, cur = 0;
static uint32_t gap_start = 0, t_fire = 0;

static void finish_reading() {
  s_active = -1;
  gap_start = micros();
  idx = (uint8_t)((idx + 1) % ORDER_LEN);
  st = ST_IDLE;
}

void ultrasonic_update() {
  uint32_t now = micros();
  if (st == ST_IDLE) {
    if (now - gap_start < GAP_US) return;
    cur = ORDER[idx];
    if (fire(cur)) {
      pcf_warned = false;
      t_fire = micros();
      st = ST_WAIT;
    } else {
      if (!pcf_warned) { Serial.println("[HC-SR04] PCF8574 no responde: TRIG sin pulso"); pcf_warned = true; }
      s_active = -1;
      register_invalid(cur);
      finish_reading();
    }
  } else {
    if (s_done[cur]) {
      uint32_t w = s_width[cur];
      s_active = -1;
      register_width(cur, w);
      finish_reading();
    } else if (now - t_fire > TIMEOUT_US) {
      s_active = -1;
      register_invalid(cur);
      finish_reading();
    }
  }
}

float ultrasonic_get_cm(UsId id) { return s_has[id] ? s_cm[id] : -1.0f; }
bool  ultrasonic_ok(UsId id)     { return s_has[id] && s_bad[id] < INVALID_LIMIT; }

float ultrasonic_read_cm() {
  static uint32_t last_end_us = 0;
  static bool first = true;
  if (!first) {
    uint32_t elapsed = micros() - last_end_us;
    if (elapsed < GAP_US) {
      uint32_t falta = GAP_US - elapsed;
      delay(falta / 1000);
      delayMicroseconds(falta % 1000);
    }
  }
  fire(US_FRONT);
  uint32_t t0 = micros();
  while (!s_done[US_FRONT] && (micros() - t0) < TIMEOUT_US) { delayMicroseconds(20); }
  s_active = -1;
  last_end_us = micros();
  first = false;
  if (!s_done[US_FRONT]) return -1.0f;
  return s_width[US_FRONT] * 0.0343f / 2.0f;
}

void ultrasonic_test_loop() {
  ultrasonic_update();
  static uint32_t t = 0;
  if (millis() - t >= 250) {
    t = millis();
    auto f = [](UsId id, char *b, size_t n) {
      float c = ultrasonic_get_cm(id);
      if (c < 0) snprintf(b, n, "  ---  "); else snprintf(b, n, "%6.1f ", c);
    };
    char a[16], b[16], c[16];
    f(US_FRONT, a, sizeof a); f(US_LEFT, b, sizeof b); f(US_RIGHT, c, sizeof c);
    Serial.printf("[HC-SR04] Frontal:%scm | Izq:%scm | Der:%scm\n", a, b, c);
  }
  delay(1);
}

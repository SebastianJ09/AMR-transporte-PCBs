#include "camera_stream.h"
#include "camera_ov2640.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char *STREAM_BOUNDARY     = "\r\n--frame\r\n";
static const char *STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static constexpr uint8_t MAX_STREAM_SERVERS = 4;
static httpd_handle_t s_servers[MAX_STREAM_SERVERS] = { nullptr };
static uint8_t        s_server_count = 0;

// Buffer compartido en PSRAM para que los puertos 81 (Web) y 82 (OpenCV)
// puedan consumir el mismo frame sin colisionar en esp_camera_fb_get().
static SemaphoreHandle_t s_cam_mutex  = nullptr;
static uint8_t          *s_shared_buf = nullptr;
static size_t            s_shared_cap = 0;
static size_t            s_shared_len = 0;
static uint32_t          s_shared_seq = 0;
static uint32_t          s_shared_ts  = 0;

static uint8_t *alloc_frame_buf(size_t size) {
  uint8_t *p = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) p = (uint8_t *)malloc(size);
  return p;
}

// Obtiene un frame JPEG reciente y lo copia a 'client_buf' liberando de inmediato
// el framebuffer DMA de la camara.
static size_t acquire_frame(uint8_t **client_buf, size_t *client_cap, uint32_t *last_seq) {
  if (!camera_ov2640_is_ready() || !s_cam_mutex) return 0;

  if (xSemaphoreTake(s_cam_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    return 0;
  }

  uint32_t now = millis();
  bool need_new = (s_shared_seq == 0) ||
                  (s_shared_seq == *last_seq) ||
                  (now - s_shared_ts > 35);

  if (need_new) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      if (fb->len > 2 && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8) {
        if (fb->len > s_shared_cap) {
          size_t new_cap = fb->len + 4096;
          uint8_t *nb = alloc_frame_buf(new_cap);
          if (nb) {
            if (s_shared_buf) free(s_shared_buf);
            s_shared_buf = nb;
            s_shared_cap = new_cap;
          }
        }
        if (fb->len <= s_shared_cap && s_shared_buf) {
          memcpy(s_shared_buf, fb->buf, fb->len);
          s_shared_len = fb->len;
          s_shared_seq++;
          s_shared_ts = millis();
        }
      }
      esp_camera_fb_return(fb);   // devolver SIEMPRE de inmediato
    }
  }

  size_t out_len = 0;
  if (s_shared_len > 0 && s_shared_buf) {
    if (s_shared_len > *client_cap) {
      size_t new_cap = s_shared_len + 4096;
      uint8_t *nb = alloc_frame_buf(new_cap);
      if (nb) {
        if (*client_buf) free(*client_buf);
        *client_buf = nb;
        *client_cap = new_cap;
      }
    }
    if (s_shared_len <= *client_cap && *client_buf) {
      memcpy(*client_buf, s_shared_buf, s_shared_len);
      out_len = s_shared_len;
      *last_seq = s_shared_seq;
    }
  }

  xSemaphoreGive(s_cam_mutex);
  return out_len;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_hdr(req, "X-Framerate", "25");

  uint8_t *buf = nullptr;
  size_t   cap = 0;
  uint32_t seq = 0;
  char     part_hdr[64];

  while (true) {
    size_t len = acquire_frame(&buf, &cap, &seq);
    if (len == 0) {
      vTaskDelay(pdMS_TO_TICKS(15));
      continue;
    }

    esp_err_t res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      int hlen = snprintf(part_hdr, sizeof(part_hdr), STREAM_PART, (unsigned)len);
      res = httpd_resp_send_chunk(req, part_hdr, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)buf, len);
    }
    if (res != ESP_OK) {
      break;   // cliente desconectado
    }
    vTaskDelay(pdMS_TO_TICKS(30));   // ~25-30 FPS max, cede CPU a Wi-Fi y control
  }

  if (buf) free(buf);
  return ESP_OK;
}

bool camera_stream_start(const uint16_t *ports, uint8_t count) {
  if (!s_cam_mutex) {
    s_cam_mutex = xSemaphoreCreateMutex();
  }

  bool all_ok = true;
  for (uint8_t i = 0; i < count && s_server_count < MAX_STREAM_SERVERS; i++) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port      = ports[i];
    config.ctrl_port        = 32768 + s_server_count;   // distinto por instancia
    config.max_open_sockets = 3;
    config.lru_purge_enable = true;
    config.stack_size       = 6144;
    config.task_priority    = 4;

    httpd_handle_t srv = nullptr;
    esp_err_t err = httpd_start(&srv, &config);
    if (err == ESP_OK && srv) {
      httpd_uri_t uri_stream = {
        .uri      = "/stream",
        .method   = HTTP_GET,
        .handler  = stream_handler,
        .user_ctx = nullptr
      };
      httpd_uri_t uri_root = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = stream_handler,
        .user_ctx = nullptr
      };
      httpd_register_uri_handler(srv, &uri_stream);
      httpd_register_uri_handler(srv, &uri_root);
      s_servers[s_server_count++] = srv;
      Serial.printf("[STREAM] Servidor MJPEG activo en http://192.168.4.1:%u/stream\n", ports[i]);
    } else {
      Serial.printf("[STREAM] Error iniciando puerto %u (err=0x%x)\n", ports[i], err);
      all_ok = false;
    }
  }
  return all_ok;
}

bool camera_stream_is_running() {
  return s_server_count > 0;
}

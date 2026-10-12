// ESP32-CAM (AI-Thinker) — вывод изображения ПО ПРОВОДУ (USB/UART), без Wi-Fi.
// Захватывает JPEG-кадр и шлёт его в Serial с заголовком-синхрой.
// Протокол кадра:  AA 55 AA 55 | LEN(4 байта, little-endian) | JPEG[LEN]
// Команды с ПК (ESP их читает):  F0 <n>  — framesize (0..13);  F1 <n> — качество JPEG (0..63, меньше = лучше)
//
// Плата в Arduino IDE: "AI Thinker ESP32-CAM", PSRAM: Enabled, Partition: Huge APP.
// Скорость монитора порта НЕ важна — приложение само открывает порт на SERIAL_BAUD.

#include "esp_camera.h"
#include "board_config.h"   // выбрана CAMERA_MODEL_AI_THINKER

// Скорость UART. 921600 — надёжно на CH340/FTDI без потери байт (приоритет стабильности).
// Можно поднять до 1500000/2000000, но на Windows возможны битые кадры.
// Значение должно совпадать с baud в приложении.
#define SERIAL_BAUD 921600

static const uint8_t MAGIC[4] = {0xAA, 0x55, 0xAA, 0x55};

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setDebugOutput(false);   // никакого мусора в поток кадров

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;     // стартуем с лёгкого, можно поднять командой F0
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  if (!psramFound()) {
    // Без PSRAM — скромнее, чтобы не падать
    config.frame_size = FRAMESIZE_QVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.jpeg_quality = 14;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    // Сигналим ошибку медленным миганием встроенного флеш-светодиода (GPIO4)
    pinMode(4, OUTPUT);
    while (true) { digitalWrite(4, HIGH); delay(200); digitalWrite(4, LOW); delay(800); }
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  // OV2640 на AI-Thinker обычно перевёрнут — выправляем
  s->set_vflip(s, 1);
  s->set_hmirror(s, 0);
}

// --- разбор команд с ПК (2 байта: код + значение) ---
static void handleCommands() {
  while (Serial.available() >= 2) {
    int code = Serial.read();
    int val = Serial.read();
    sensor_t *s = esp_camera_sensor_get();
    if (!s) continue;
    if (code == 0xF0) {                 // framesize 0..13
      if (val >= 0 && val <= 13) s->set_framesize(s, (framesize_t)val);
    } else if (code == 0xF1) {          // jpeg quality 0..63
      if (val >= 0 && val <= 63) s->set_quality(s, val);
    }
  }
}

void loop() {
  handleCommands();

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    delay(10);
    return;
  }

  if (fb->format == PIXFORMAT_JPEG && fb->len > 0) {
    uint32_t len = (uint32_t)fb->len;
    Serial.write(MAGIC, 4);
    Serial.write((uint8_t *)&len, 4);   // little-endian (ESP32 LE)
    Serial.write(fb->buf, fb->len);
  }

  esp_camera_fb_return(fb);
}

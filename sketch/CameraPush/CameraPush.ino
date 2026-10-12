// ESP32-CAM (AI-Thinker, OV2640) — Wi-Fi режим: камера сама шлёт JPEG-кадры
// POST'ом на сервер СКУД, сервер раздаёт их на сайт как MJPEG. UART не используется.
//
// Прошивка: arduino-cli, fqbn esp32:esp32:esp32cam:PartitionScheme=huge_app
// Плата на ESP32-CAM-MB (USB-UART + авто-сброс), порт COM8 — BOOT держать не надо.

#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"   // для отключения brownout-детектора

// ===== ЗАПОЛНИТЬ: Wi-Fi (2.4 ГГц!) =====
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASS";

// ===== Сервер =====
const char* PUSH_URL =
  "http://YOUR_SERVER_HOST:PORT/api/cam/push?token=YOUR_PUSH_TOKEN&cam=indoor";

const int   TARGET_FPS   = 10;          // верхняя планка кадров/сек
const int   FRAME_GAP_MS = 1000 / TARGET_FPS;

// ===== Распиновка AI-Thinker ESP32-CAM =====
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22
#define LED_GPIO_NUM     4   // яркий белый флеш-LED — держим выключенным

unsigned long lastFrameAt = 0;

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA;      // 640x480; при слабом Wi-Fi сменим на QVGA
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;               // 0..63 (меньше = лучше/тяжелее)
  config.fb_count = 2;

  if (!psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);     // (разворот картинки делаем на сайте через CSS, не тут)
    s->set_hmirror(s, 0);
  }
  return true;
}

void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Wi-Fi: подключаюсь к %s\n", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(300); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWi-Fi OK, IP=%s, RSSI=%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("\nWi-Fi не подключился, перезагрузка");
    delay(1000); ESP.restart();
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // отключаем brownout-детектор (USB-питание слабое)
  Serial.begin(115200);
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);     // флеш выключен
  if (!initCamera()) { delay(2000); ESP.restart(); }
  wifiConnect();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) { wifiConnect(); return; }

  if (millis() - lastFrameAt < (unsigned long)FRAME_GAP_MS) { delay(2); return; }
  lastFrameAt = millis();

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { Serial.println("capture failed"); delay(50); return; }

  HTTPClient http;
  http.setConnectTimeout(2000);
  http.setTimeout(3000);
  if (http.begin(PUSH_URL)) {
    http.addHeader("Content-Type", "image/jpeg");
    int code = http.POST(fb->buf, fb->len);
    if (code <= 0) {
      Serial.printf("POST fail: %s\n", http.errorToString(code).c_str());
    }
    http.end();
  }
  esp_camera_fb_return(fb);
}

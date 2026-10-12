# ESP32-CAM Viewer

Десктоп-просмотрщик изображения с ESP32-CAM (AI-Thinker, OV2640): показ потока,
снимок в PNG, запись видео в MP4. Два режима работы.

## Режимы

- **По проводу (USB/UART), рекомендуется** — `serial_viewer.py` + прошивка
  `sketch/CameraSerial`. Камера шлёт JPEG-кадры в Serial с заголовком-синхрой
  `AA 55 AA 55 | LEN(uint32 LE) | JPEG[LEN]`. Скорость UART по умолчанию **921600**
  (стабильно, без битых кадров; выше — на Windows возможны потери байт). Wi-Fi не нужен.
- **По Wi-Fi (MJPEG)** — `viewer.py` + прошивка `sketch/CameraWebServer`. Тянет поток
  с `http://<IP>:81/stream`, разрешение меняется через `http://<IP>/control?var=framesize&val=N`.

## Прошивка

Залить скетч через arduino-cli (ядро `esp32`, плата AI-Thinker):

```
arduino-cli compile --fqbn "esp32:esp32:esp32cam:PartitionScheme=huge_app" sketch/CameraSerial
arduino-cli upload  --fqbn "esp32:esp32:esp32cam:PartitionScheme=huge_app" -p COM8 sketch/CameraSerial
```

Для Wi-Fi-режима заменить `CameraSerial` на `CameraWebServer` и прописать в скетче
свои `YOUR_WIFI_SSID` / `YOUR_WIFI_PASS` (ESP32 поддерживает только 2.4 ГГц).

## Установка и запуск

```
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt

python serial_viewer.py   # режим по проводу
python viewer.py          # режим по Wi-Fi
```

В проводном режиме выбрать COM-порт и Baud (921600), нажать «Подключиться».
В Wi-Fi-режиме ввести IP камеры. Настройки сохраняются в `config.json`
(шаблон — `config.example.json`).

## Зависимости

opencv-python, numpy, requests, PySide6, pyserial (см. `requirements.txt`).

## Подключение ESP32-CAM

Плата-носитель ESP32-CAM-MB (USB-UART, авто-сброс) в USB — держать BOOT при
прошивке не нужно. COM-порт монопольный: закрой Arduino Serial Monitor перед
запуском просмотрщика.

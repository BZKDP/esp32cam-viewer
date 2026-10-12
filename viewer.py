"""ESP32-CAM MJPEG viewer (AI-Thinker / CameraWebServer)."""
from __future__ import annotations

import json
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import requests
from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_FILE = SCRIPT_DIR / "config.json"
SNAPSHOTS_DIR = SCRIPT_DIR / "snapshots"
RECORDINGS_DIR = SCRIPT_DIR / "recordings"

# (label, framesize value) — values совпадают с enum framesize_t из sensor.h
FRAMESIZES: list[tuple[str, int]] = [
    ("0  — 96×96",          0),
    ("1  — QQVGA 160×120",  1),
    ("2  — QCIF 176×144",   2),
    ("3  — HQVGA 240×176",  3),
    ("4  — 240×240",        4),
    ("5  — QVGA 320×240",   5),
    ("6  — CIF 400×296",    6),
    ("7  — HVGA 480×320",   7),
    ("8  — VGA 640×480",    8),
    ("9  — SVGA 800×600",   9),
    ("10 — XGA 1024×768",   10),
    ("11 — HD 1280×720",    11),
    ("12 — SXGA 1280×1024", 12),
    ("13 — UXGA 1600×1200", 13),
]
DEFAULT_FRAMESIZE_INDEX = 8  # VGA

RECONNECT_DELAY_SEC = 3.0
STREAM_TIMEOUT_SEC = 5.0


class StreamWorker(QThread):
    """Тянет MJPEG-поток, парсит JPEG по маркерам SOI/EOI, эмитит готовые кадры."""

    frame_ready = Signal(np.ndarray)
    status_changed = Signal(str)

    def __init__(self, ip: str, parent=None) -> None:
        super().__init__(parent)
        self.ip = ip
        self.url = f"http://{ip}:81/stream"
        self._stop = False

    def stop(self) -> None:
        self._stop = True

    def _interruptible_sleep(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while not self._stop and time.monotonic() < end:
            time.sleep(0.05)

    def run(self) -> None:
        while not self._stop:
            try:
                self.status_changed.emit(f"Подключение к {self.url}…")
                with requests.get(self.url, stream=True, timeout=STREAM_TIMEOUT_SEC) as resp:
                    if resp.status_code != 200:
                        self.status_changed.emit(
                            f"HTTP {resp.status_code}. Переподключение через {int(RECONNECT_DELAY_SEC)} с…"
                        )
                        self._interruptible_sleep(RECONNECT_DELAY_SEC)
                        continue

                    self.status_changed.emit("Подключено")
                    buf = bytearray()
                    for chunk in resp.iter_content(chunk_size=4096):
                        if self._stop:
                            return
                        if not chunk:
                            continue
                        buf.extend(chunk)
                        # Извлекаем все полные JPEG из буфера
                        while True:
                            start = buf.find(b"\xff\xd8")
                            if start == -1:
                                # Маркера старта нет — сбрасываем шум, оставляем хвост (вдруг это половина 0xFF)
                                if len(buf) > 1:
                                    del buf[:-1]
                                break
                            end = buf.find(b"\xff\xd9", start + 2)
                            if end == -1:
                                # JPEG ещё не дочитан
                                if start > 0:
                                    del buf[:start]
                                break
                            jpg = bytes(buf[start : end + 2])
                            del buf[: end + 2]
                            arr = np.frombuffer(jpg, dtype=np.uint8)
                            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                            if img is not None:
                                self.frame_ready.emit(img)
            except requests.exceptions.RequestException as exc:
                if self._stop:
                    return
                self.status_changed.emit(
                    f"Соединение потеряно: {exc.__class__.__name__}. Переподключение через {int(RECONNECT_DELAY_SEC)} с…"
                )
                self._interruptible_sleep(RECONNECT_DELAY_SEC)
            except Exception as exc:  # noqa: BLE001
                if self._stop:
                    return
                self.status_changed.emit(
                    f"Ошибка: {exc}. Переподключение через {int(RECONNECT_DELAY_SEC)} с…"
                )
                self._interruptible_sleep(RECONNECT_DELAY_SEC)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ESP32-CAM Viewer")
        self.resize(960, 720)

        self.worker: StreamWorker | None = None
        self.last_frame: np.ndarray | None = None
        self._frames_since_tick = 0
        self._tick_start = time.monotonic()
        self.current_fps = 0.0

        self.writer: cv2.VideoWriter | None = None
        self.recording = False
        self.recording_path: Path | None = None
        self.recording_size: tuple[int, int] | None = None

        cfg = self._load_config()
        last_ip = cfg.get("ip", "192.168.1.100")

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # --- Верхняя панель: IP, подключение, сохранение IP ---
        top = QHBoxLayout()
        top.addWidget(QLabel("IP камеры:"))
        self.ip_edit = QLineEdit(last_ip)
        self.ip_edit.setPlaceholderText("например, 192.168.1.42")
        top.addWidget(self.ip_edit, stretch=1)
        self.connect_btn = QPushButton("Подключиться")
        self.connect_btn.clicked.connect(self.connect_stream)
        top.addWidget(self.connect_btn)
        self.stop_btn = QPushButton("Стоп")
        self.stop_btn.clicked.connect(self.stop_stream)
        self.stop_btn.setEnabled(False)
        top.addWidget(self.stop_btn)
        self.save_ip_btn = QPushButton("Сохранить IP")
        self.save_ip_btn.clicked.connect(self.save_ip)
        top.addWidget(self.save_ip_btn)
        root.addLayout(top)

        # --- Вторая панель: разрешение, снимок, запись ---
        controls = QHBoxLayout()
        controls.addWidget(QLabel("Разрешение:"))
        self.framesize_combo = QComboBox()
        for label, val in FRAMESIZES:
            self.framesize_combo.addItem(label, val)
        self.framesize_combo.setCurrentIndex(DEFAULT_FRAMESIZE_INDEX)
        self.framesize_combo.currentIndexChanged.connect(self.change_framesize)
        controls.addWidget(self.framesize_combo)
        controls.addStretch(1)
        self.snapshot_btn = QPushButton("Сделать снимок")
        self.snapshot_btn.clicked.connect(self.take_snapshot)
        controls.addWidget(self.snapshot_btn)
        self.record_btn = QPushButton("Записать видео")
        self.record_btn.clicked.connect(self.toggle_record)
        controls.addWidget(self.record_btn)
        root.addLayout(controls)

        # --- Видеообласть ---
        self.video_label = QLabel("Нет изображения")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: #1e1e1e; color: #888; font-size: 14px;")
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video_label.setMinimumSize(320, 240)
        root.addWidget(self.video_label, stretch=1)

        # --- Статус-бар ---
        status = QStatusBar()
        self.setStatusBar(status)
        self.status_state = QLabel("Не подключено")
        self.status_resolution = QLabel("—")
        self.status_fps = QLabel("FPS: —")
        status.addPermanentWidget(self.status_state, 1)
        status.addPermanentWidget(self.status_resolution)
        status.addPermanentWidget(self.status_fps)
        self._status_bar = status

        # FPS-таймер: раз в секунду считаем кадры
        self._fps_timer = QTimer(self)
        self._fps_timer.timeout.connect(self._tick_fps)
        self._fps_timer.start(1000)

    # ---------- config ----------
    def _load_config(self) -> dict:
        if CONFIG_FILE.exists():
            try:
                return json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
            except (json.JSONDecodeError, OSError):
                return {}
        return {}

    def _save_config(self, cfg: dict) -> None:
        CONFIG_FILE.write_text(json.dumps(cfg, indent=2, ensure_ascii=False), encoding="utf-8")

    def save_ip(self) -> None:
        ip = self.ip_edit.text().strip()
        if not ip:
            QMessageBox.warning(self, "IP", "Введите IP-адрес камеры.")
            return
        self._save_config({"ip": ip})
        self._status_bar.showMessage(f"IP {ip} сохранён в config.json", 3000)

    # ---------- stream ----------
    def connect_stream(self) -> None:
        ip = self.ip_edit.text().strip()
        if not ip:
            QMessageBox.warning(self, "IP", "Введите IP-адрес камеры.")
            return
        self._stop_worker()
        self.worker = StreamWorker(ip)
        self.worker.frame_ready.connect(self.on_frame)
        self.worker.status_changed.connect(self.on_status)
        self.worker.start()
        self.connect_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self._frames_since_tick = 0
        self._tick_start = time.monotonic()

    def stop_stream(self) -> None:
        self._stop_worker()
        if self.recording:
            self.toggle_record()
        self.connect_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_state.setText("Не подключено")

    def _stop_worker(self) -> None:
        if self.worker is not None:
            self.worker.stop()
            self.worker.wait(3000)
            self.worker = None

    def on_status(self, text: str) -> None:
        self.status_state.setText(text)

    def on_frame(self, img: np.ndarray) -> None:
        self.last_frame = img
        self._frames_since_tick += 1
        h, w = img.shape[:2]
        self.status_resolution.setText(f"{w}×{h}")

        if self.recording and self.writer is not None and self.recording_size == (w, h):
            self.writer.write(img)

        # BGR (OpenCV) → RGB → QImage. .copy() — чтобы не зависеть от времени жизни буфера numpy.
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        qimg = QImage(
            rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888
        ).copy()
        pix = QPixmap.fromImage(qimg).scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.video_label.setPixmap(pix)

    def _tick_fps(self) -> None:
        now = time.monotonic()
        elapsed = now - self._tick_start
        if elapsed > 0:
            self.current_fps = self._frames_since_tick / elapsed
        self.status_fps.setText(f"FPS: {self.current_fps:.1f}")
        self._frames_since_tick = 0
        self._tick_start = now

    # ---------- camera control ----------
    def change_framesize(self) -> None:
        ip = self.ip_edit.text().strip()
        if not ip:
            return
        val = self.framesize_combo.currentData()
        url = f"http://{ip}/control?var=framesize&val={val}"
        try:
            resp = requests.get(url, timeout=3)
            if resp.status_code == 200:
                self._status_bar.showMessage(f"framesize={val} применено", 3000)
            else:
                self._status_bar.showMessage(f"framesize: HTTP {resp.status_code}", 3000)
        except requests.exceptions.RequestException as exc:
            self._status_bar.showMessage(f"framesize: {exc.__class__.__name__}", 3000)

    # ---------- snapshot / record ----------
    def take_snapshot(self) -> None:
        if self.last_frame is None:
            QMessageBox.information(self, "Снимок", "Нет кадра — подключитесь к потоку.")
            return
        SNAPSHOTS_DIR.mkdir(exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = SNAPSHOTS_DIR / f"snap_{ts}.png"
        ok = cv2.imwrite(str(path), self.last_frame)
        if ok:
            self._status_bar.showMessage(f"Снимок сохранён: {path.name}", 3000)
        else:
            QMessageBox.warning(self, "Снимок", f"Не удалось сохранить {path}")

    def toggle_record(self) -> None:
        if not self.recording:
            if self.last_frame is None:
                QMessageBox.information(self, "Запись", "Нет кадра — подключитесь к потоку.")
                return
            RECORDINGS_DIR.mkdir(exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = RECORDINGS_DIR / f"rec_{ts}.mp4"
            h, w = self.last_frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            # Используем измеренный FPS, но не меньше 10 — иначе плеер видит «слайд-шоу»
            fps = self.current_fps if self.current_fps >= 1.0 else 15.0
            writer = cv2.VideoWriter(str(path), fourcc, fps, (w, h))
            if not writer.isOpened():
                QMessageBox.warning(self, "Запись", "Не удалось открыть VideoWriter (mp4v).")
                return
            self.writer = writer
            self.recording = True
            self.recording_path = path
            self.recording_size = (w, h)
            self.record_btn.setText("Остановить запись")
            self._status_bar.showMessage(f"Запись: {path.name}", 3000)
        else:
            self.recording = False
            if self.writer is not None:
                self.writer.release()
                self.writer = None
            self.record_btn.setText("Записать видео")
            if self.recording_path is not None:
                self._status_bar.showMessage(
                    f"Запись сохранена: {self.recording_path.name}", 5000
                )

    # ---------- lifecycle ----------
    def closeEvent(self, event) -> None:  # noqa: N802
        self._stop_worker()
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())

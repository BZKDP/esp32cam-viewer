"""ESP32-CAM viewer ПО ПРОВОДУ (USB/UART).

Читает кадры с COM-порта (прошивка sketch/CameraSerial), декодирует JPEG и
показывает. Протокол кадра: AA 55 AA 55 | LEN(uint32 LE) | JPEG[LEN].
"""
from __future__ import annotations

import json
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import serial
import serial.tools.list_ports
from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QHBoxLayout,
    QLabel,
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

MAGIC = b"\xaa\x55\xaa\x55"
DEFAULT_BAUD = 921600  # совпадает с SERIAL_BAUD в прошивке CameraSerial.ino
BAUDS = [921600, 1500000, 2000000, 460800, 230400, 115200]
MAX_FRAME_BYTES = 4 * 1024 * 1024  # защита от мусорной длины при рассинхроне

# (label, framesize value) — совпадает с enum framesize_t из sensor.h
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
DEFAULT_FRAMESIZE_INDEX = 5  # QVGA — быстрый и стабильный старт по проводу

RECONNECT_DELAY_SEC = 2.0


class SerialWorker(QThread):
    """Читает кадры с COM-порта по протоколу MAGIC+LEN+JPEG, эмитит готовые кадры."""

    frame_ready = Signal(np.ndarray)
    status_changed = Signal(str)

    def __init__(self, port: str, baud: int, parent=None) -> None:
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._stop = False
        self.ser: serial.Serial | None = None

    def stop(self) -> None:
        self._stop = True

    def send_command(self, code: int, value: int) -> None:
        """Шлёт 2-байтовую команду на ESP (F0=framesize, F1=quality)."""
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write(bytes([code & 0xFF, value & 0xFF]))
            except serial.SerialException:
                pass

    def _interruptible_sleep(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while not self._stop and time.monotonic() < end:
            time.sleep(0.05)

    def _read_exact(self, n: int) -> bytes | None:
        """Читает ровно n байт (или None, если поток прервался/стоп)."""
        buf = bytearray()
        while len(buf) < n and not self._stop:
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
        return bytes(buf) if len(buf) == n else None

    def _sync_to_magic(self) -> bool:
        """Скользящим окном ищет 4-байтовую сигнатуру MAGIC. Терпит boot-мусор."""
        window = bytearray()
        while not self._stop:
            b = self.ser.read(1)
            if not b:
                continue
            window.extend(b)
            if len(window) > 4:
                del window[0]
            if window == MAGIC:
                return True
        return False

    def run(self) -> None:
        while not self._stop:
            try:
                self.status_changed.emit(f"Открываю {self.port} @ {self.baud}…")
                self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
                # Дать плате время на сброс после открытия порта (DTR/RTS toggle).
                self._interruptible_sleep(0.4)
                self.ser.reset_input_buffer()
                self.status_changed.emit("Подключено — жду синхру…")

                while not self._stop:
                    if not self._sync_to_magic():
                        break
                    raw_len = self._read_exact(4)
                    if raw_len is None:
                        break
                    length = int.from_bytes(raw_len, "little")
                    if length == 0 or length > MAX_FRAME_BYTES:
                        # рассинхрон — ищем MAGIC заново
                        continue
                    jpg = self._read_exact(length)
                    if jpg is None:
                        break
                    arr = np.frombuffer(jpg, dtype=np.uint8)
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        self.frame_ready.emit(img)
            except serial.SerialException as exc:
                if self._stop:
                    break
                self.status_changed.emit(
                    f"Порт недоступен: {exc.__class__.__name__}. Повтор через {int(RECONNECT_DELAY_SEC)} с…"
                )
                self._interruptible_sleep(RECONNECT_DELAY_SEC)
            except Exception as exc:  # noqa: BLE001
                if self._stop:
                    break
                self.status_changed.emit(
                    f"Ошибка: {exc}. Повтор через {int(RECONNECT_DELAY_SEC)} с…"
                )
                self._interruptible_sleep(RECONNECT_DELAY_SEC)
            finally:
                if self.ser is not None and self.ser.is_open:
                    self.ser.close()
                self.ser = None


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ESP32-CAM Viewer (по проводу)")
        self.resize(960, 720)

        self.worker: SerialWorker | None = None
        self.last_frame: np.ndarray | None = None
        self._frames_since_tick = 0
        self._tick_start = time.monotonic()
        self.current_fps = 0.0

        self.writer: cv2.VideoWriter | None = None
        self.recording = False
        self.recording_path: Path | None = None
        self.recording_size: tuple[int, int] | None = None

        cfg = self._load_config()

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # --- Верхняя панель: порт, baud, обновить, подключение ---
        top = QHBoxLayout()
        top.addWidget(QLabel("COM-порт:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(220)
        top.addWidget(self.port_combo, stretch=1)
        self.refresh_btn = QPushButton("⟳")
        self.refresh_btn.setToolTip("Обновить список портов")
        self.refresh_btn.setFixedWidth(36)
        self.refresh_btn.clicked.connect(self.refresh_ports)
        top.addWidget(self.refresh_btn)

        top.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        for b in BAUDS:
            self.baud_combo.addItem(str(b), b)
        saved_baud = int(cfg.get("baud", DEFAULT_BAUD))
        idx = self.baud_combo.findData(saved_baud)
        self.baud_combo.setCurrentIndex(idx if idx >= 0 else 0)
        top.addWidget(self.baud_combo)

        self.connect_btn = QPushButton("Подключиться")
        self.connect_btn.clicked.connect(self.connect_stream)
        top.addWidget(self.connect_btn)
        self.stop_btn = QPushButton("Стоп")
        self.stop_btn.clicked.connect(self.stop_stream)
        self.stop_btn.setEnabled(False)
        top.addWidget(self.stop_btn)
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

        self._fps_timer = QTimer(self)
        self._fps_timer.timeout.connect(self._tick_fps)
        self._fps_timer.start(1000)

        self.refresh_ports(saved_port=cfg.get("port"))

        # Автоподключение при старте, если порт выбран — чтобы видео пошло сразу.
        if self.port_combo.currentData():
            QTimer.singleShot(300, self.connect_stream)

    # ---------- ports ----------
    def refresh_ports(self, saved_port: str | None = None) -> None:
        current = saved_port or self.port_combo.currentData()
        self.port_combo.clear()
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            self.port_combo.addItem(f"{p.device} — {p.description}", p.device)
        if current:
            idx = self.port_combo.findData(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
        if self.port_combo.count() == 0:
            self.port_combo.addItem("портов не найдено", None)

    # ---------- config ----------
    def _load_config(self) -> dict:
        if CONFIG_FILE.exists():
            try:
                return json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
            except (json.JSONDecodeError, OSError):
                return {}
        return {}

    def _save_config(self, cfg: dict) -> None:
        existing = self._load_config()
        existing.update(cfg)
        CONFIG_FILE.write_text(json.dumps(existing, indent=2, ensure_ascii=False), encoding="utf-8")

    # ---------- stream ----------
    def connect_stream(self) -> None:
        port = self.port_combo.currentData()
        if not port:
            QMessageBox.warning(self, "Порт", "Не выбран COM-порт. Нажмите ⟳ и выберите плату.")
            return
        baud = self.baud_combo.currentData()
        self._save_config({"port": port, "baud": baud})
        self._stop_worker()
        self.worker = SerialWorker(port, baud)
        self.worker.frame_ready.connect(self.on_frame)
        self.worker.status_changed.connect(self.on_status)
        self.worker.start()
        self.connect_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.port_combo.setEnabled(False)
        self.baud_combo.setEnabled(False)
        self._frames_since_tick = 0
        self._tick_start = time.monotonic()
        # Применить выбранное разрешение на старте
        QTimer.singleShot(1200, self.change_framesize)

    def stop_stream(self) -> None:
        self._stop_worker()
        if self.recording:
            self.toggle_record()
        self.connect_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.port_combo.setEnabled(True)
        self.baud_combo.setEnabled(True)
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

        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy()
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
        if self.worker is None:
            return
        val = self.framesize_combo.currentData()
        self.worker.send_command(0xF0, val)
        self._status_bar.showMessage(f"framesize={val} отправлено", 3000)

    # ---------- snapshot / record ----------
    def take_snapshot(self) -> None:
        if self.last_frame is None:
            QMessageBox.information(self, "Снимок", "Нет кадра — подключитесь к плате.")
            return
        SNAPSHOTS_DIR.mkdir(exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = SNAPSHOTS_DIR / f"snap_{ts}.png"
        if cv2.imwrite(str(path), self.last_frame):
            self._status_bar.showMessage(f"Снимок сохранён: {path.name}", 3000)
        else:
            QMessageBox.warning(self, "Снимок", f"Не удалось сохранить {path}")

    def toggle_record(self) -> None:
        if not self.recording:
            if self.last_frame is None:
                QMessageBox.information(self, "Запись", "Нет кадра — подключитесь к плате.")
                return
            RECORDINGS_DIR.mkdir(exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = RECORDINGS_DIR / f"rec_{ts}.mp4"
            h, w = self.last_frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
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
                self._status_bar.showMessage(f"Запись сохранена: {self.recording_path.name}", 5000)

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

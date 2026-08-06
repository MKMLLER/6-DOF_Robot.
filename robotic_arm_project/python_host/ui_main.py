"""
Main UI window for the robotic arm control application.
PyQt6-based dark themed interface with camera view, motor controls, and coordinate input.
"""

import json
import cv2
import numpy as np
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QSpinBox, QDoubleSpinBox, QComboBox,
    QSlider, QGroupBox, QStatusBar, QSplitter, QFrame,
    QSizePolicy, QToolBar, QTextEdit
)
from PyQt6.QtCore import Qt, QTimer, pyqtSlot, QSize
from PyQt6.QtGui import QPixmap, QImage, QFont, QColor, QMouseEvent, QIcon, QPalette

from serial_comm import SerialComm
from camera import CameraWorker, list_cameras
from cv import ClickToMove


# ─── Style Constants ───
DARK_STYLE = """
QMainWindow {
    background-color: #1a1a2e;
}
QWidget {
    background-color: #1a1a2e;
    color: #e0e0e0;
    font-family: 'Segoe UI', 'Roboto', sans-serif;
    font-size: 13px;
}
QGroupBox {
    border: 1px solid #2a2a4a;
    border-radius: 8px;
    margin-top: 12px;
    padding: 15px 10px 10px 10px;
    background-color: #16213e;
    font-weight: bold;
    font-size: 14px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 2px 12px;
    color: #00d4ff;
    background-color: #16213e;
    border-radius: 4px;
}
QPushButton {
    background-color: #0f3460;
    color: #e0e0e0;
    border: 1px solid #1a5276;
    border-radius: 6px;
    padding: 8px 16px;
    font-weight: 500;
    min-height: 28px;
}
QPushButton:hover {
    background-color: #1a5276;
    border-color: #00d4ff;
}
QPushButton:pressed {
    background-color: #00d4ff;
    color: #1a1a2e;
}
QPushButton#emergencyStop {
    background-color: #c0392b;
    color: white;
    font-weight: bold;
    font-size: 16px;
    border: 2px solid #e74c3c;
    min-height: 40px;
}
QPushButton#emergencyStop:hover {
    background-color: #e74c3c;
}
QPushButton#homeBtn {
    background-color: #1e6f50;
    border-color: #27ae60;
}
QPushButton#homeBtn:hover {
    background-color: #27ae60;
}
QPushButton#connectBtn {
    background-color: #2980b9;
    border-color: #3498db;
}
QPushButton#connectBtn:hover {
    background-color: #3498db;
}
QPushButton#calibrateBtn {
    background-color: #8e44ad;
    border-color: #9b59b6;
}
QPushButton#calibrateBtn:hover {
    background-color: #9b59b6;
}
QSlider::groove:horizontal {
    height: 6px;
    background: #2a2a4a;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    width: 18px;
    height: 18px;
    margin: -6px 0;
    background: #00d4ff;
    border-radius: 9px;
}
QSlider::sub-page:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #0f3460, stop:1 #00d4ff);
    border-radius: 3px;
}
QComboBox {
    background-color: #0f3460;
    border: 1px solid #1a5276;
    border-radius: 5px;
    padding: 5px 10px;
    min-height: 24px;
}
QComboBox::drop-down {
    border: none;
    width: 24px;
}
QComboBox QAbstractItemView {
    background-color: #16213e;
    border: 1px solid #1a5276;
    selection-background-color: #0f3460;
}
QSpinBox, QDoubleSpinBox {
    background-color: #0f3460;
    border: 1px solid #1a5276;
    border-radius: 5px;
    padding: 4px 8px;
    min-height: 24px;
}
QSpinBox::up-button, QDoubleSpinBox::up-button,
QSpinBox::down-button, QDoubleSpinBox::down-button {
    background-color: #1a5276;
    border: none;
    width: 20px;
}
QLabel#statusDot {
    font-size: 18px;
}
QLabel#sectionLabel {
    color: #00d4ff;
    font-size: 12px;
    font-weight: bold;
    padding: 4px 0;
}
QTextEdit#logView {
    background-color: #0d1117;
    color: #8b949e;
    border: 1px solid #2a2a4a;
    border-radius: 4px;
    font-family: 'Consolas', 'Courier New', monospace;
    font-size: 11px;
}
QFrame#separator {
    background-color: #2a2a4a;
    max-height: 1px;
}
"""


class CameraView(QLabel):
    """
    Clickable camera feed widget.
    Emits pixel coordinates on mouse click.
    """
    
    def __init__(self, click_callback=None):
        super().__init__()
        self._click_callback = click_callback
        self.setMinimumSize(320, 240)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet("""
            background-color: #0d1117;
            border: 2px solid #2a2a4a;
            border-radius: 8px;
        """)
        self.setText("📷  Camera not connected")
        self.setScaledContents(False)
        self._frame_size = (640, 480)
        self._display_size = (640, 480)
    
    def set_frame_size(self, w: int, h: int):
        self._frame_size = (w, h)
    
    def mousePressEvent(self, event: QMouseEvent):
        """Convert widget click coordinates to frame pixel coordinates."""
        if event.button() == Qt.MouseButton.LeftButton and self._click_callback:
            # Get the actual pixmap display area
            pm = self.pixmap()
            if pm and not pm.isNull():
                # Calculate offset (centered in widget)
                widget_w, widget_h = self.width(), self.height()
                pm_w, pm_h = pm.width(), pm.height()
                
                # Compute scale factor (the pixmap is scaled to fit)
                scale = min(widget_w / self._frame_size[0], widget_h / self._frame_size[1])
                display_w = int(self._frame_size[0] * scale)
                display_h = int(self._frame_size[1] * scale)
                
                offset_x = (widget_w - display_w) // 2
                offset_y = (widget_h - display_h) // 2
                
                # Click position relative to displayed image
                click_x = event.position().x() - offset_x
                click_y = event.position().y() - offset_y
                
                if 0 <= click_x <= display_w and 0 <= click_y <= display_h:
                    # Map to original frame coordinates
                    frame_x = int(click_x / scale)
                    frame_y = int(click_y / scale)
                    self._click_callback(frame_x, frame_y)
        
        super().mousePressEvent(event)


class JointControl(QWidget):
    """Single joint control row: label + slider + spinbox + jog buttons."""
    
    def __init__(self, name: str, min_val: float, max_val: float, 
                 jog_callback=None, parent=None):
        super().__init__(parent)
        self._name = name
        self._jog_callback = jog_callback
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(6)
        
        # Joint label
        lbl = QLabel(name)
        lbl.setFixedWidth(70)
        lbl.setStyleSheet("color: #00d4ff; font-weight: bold;")
        layout.addWidget(lbl)
        
        # Jog negative button
        self.btn_neg = QPushButton("◀ -")
        self.btn_neg.setFixedSize(50, 30)
        self.btn_neg.clicked.connect(lambda: self._jog(-1))
        layout.addWidget(self.btn_neg)
        
        # Slider
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(int(min_val * 10), int(max_val * 10))
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self._on_slider_changed)
        layout.addWidget(self.slider, stretch=1)
        
        # Jog positive button
        self.btn_pos = QPushButton("+ ▶")
        self.btn_pos.setFixedSize(50, 30)
        self.btn_pos.clicked.connect(lambda: self._jog(1))
        layout.addWidget(self.btn_pos)
        
        # Angle value
        self.spinbox = QDoubleSpinBox()
        self.spinbox.setRange(min_val, max_val)
        self.spinbox.setDecimals(1)
        self.spinbox.setSuffix("°")
        self.spinbox.setFixedWidth(90)
        self.spinbox.valueChanged.connect(self._on_spinbox_changed)
        layout.addWidget(self.spinbox)
    
    def _on_slider_changed(self, val):
        angle = val / 10.0
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(angle)
        self.spinbox.blockSignals(False)
    
    def _on_spinbox_changed(self, val):
        self.slider.blockSignals(True)
        self.slider.setValue(int(val * 10))
        self.slider.blockSignals(False)
        if self._jog_callback:
            self._jog_callback(self._name, "angle", val)
    
    def _jog(self, direction: int):
        """Jog by a fixed step count."""
        if self._jog_callback:
            self._jog_callback(self._name, "steps", direction * 200)
    
    def set_value(self, angle: float):
        """Update display without triggering callbacks."""
        self.slider.blockSignals(True)
        self.spinbox.blockSignals(True)
        self.slider.setValue(int(angle * 10))
        self.spinbox.setValue(angle)
        self.slider.blockSignals(False)
        self.spinbox.blockSignals(False)


class MainWindow(QMainWindow):
    """Main application window."""
    
    # Joint name → joint number mapping
    JOINT_MAP = {
        "J1 Base": 1,
        "J2 Shoulder": 2,
        "J3 Elbow": 3,
        "J4 Wrist": 4,
    }
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🤖 Robotic Arm Control")
        self.setMinimumSize(1200, 750)
        self.resize(1400, 850)
        
        # Core modules
        self.serial = SerialComm()
        self.camera_worker: CameraWorker | None = None
        self.click_to_move = ClickToMove()
        
        # State
        self._camera_running = False
        self._calibrating = False
        
        # Apply style
        self.setStyleSheet(DARK_STYLE)
        
        # Build UI
        self._build_ui()
        self._connect_signals()
        
        # Status polling timer
        self._status_timer = QTimer()
        self._status_timer.timeout.connect(self._poll_status)
        self._status_timer.setInterval(500)
        
        # Calibration update timer
        self._calib_timer = QTimer()
        self._calib_timer.timeout.connect(self._update_calibration)
        self._calib_timer.setInterval(33)
        
        # Refresh ports/cameras on start
        self._refresh_ports()
        self._refresh_cameras()
    
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # ═══ LEFT SIDE: Camera + Connection ═══
        left_panel = QVBoxLayout()
        left_panel.setSpacing(8)
        
        # Connection bar
        conn_group = QGroupBox("Connection")
        conn_layout = QGridLayout(conn_group)
        conn_layout.setSpacing(6)
        
        # Serial port row
        conn_layout.addWidget(QLabel("Serial:"), 0, 0)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(120)
        conn_layout.addWidget(self.port_combo, 0, 1)
        
        self.refresh_ports_btn = QPushButton("⟳")
        self.refresh_ports_btn.setFixedSize(32, 32)
        self.refresh_ports_btn.setToolTip("Refresh COM ports")
        conn_layout.addWidget(self.refresh_ports_btn, 0, 2)
        
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("connectBtn")
        conn_layout.addWidget(self.connect_btn, 0, 3)
        
        self.serial_status = QLabel("🔴")
        self.serial_status.setObjectName("statusDot")
        self.serial_status.setFixedWidth(24)
        conn_layout.addWidget(self.serial_status, 0, 4)
        
        # Camera row
        conn_layout.addWidget(QLabel("Camera:"), 1, 0)
        self.camera_combo = QComboBox()
        conn_layout.addWidget(self.camera_combo, 1, 1)
        
        self.refresh_cameras_btn = QPushButton("⟳")
        self.refresh_cameras_btn.setFixedSize(32, 32)
        self.refresh_cameras_btn.setToolTip("Refresh cameras")
        conn_layout.addWidget(self.refresh_cameras_btn, 1, 2)
        
        self.camera_btn = QPushButton("Start Camera")
        conn_layout.addWidget(self.camera_btn, 1, 3)
        
        self.camera_status = QLabel("🔴")
        self.camera_status.setObjectName("statusDot")
        self.camera_status.setFixedWidth(24)
        conn_layout.addWidget(self.camera_status, 1, 4)
        
        left_panel.addWidget(conn_group)
        
        # Camera view
        self.camera_view = CameraView(click_callback=self._on_camera_click)
        self.camera_view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        left_panel.addWidget(self.camera_view, stretch=1)
        
        # Camera controls row
        cam_controls = QHBoxLayout()
        self.calibrate_btn = QPushButton("🔧 Calibrate Camera")
        self.calibrate_btn.setObjectName("calibrateBtn")
        cam_controls.addWidget(self.calibrate_btn)
        
        self.click_mode_label = QLabel("Click on video to move robot →")
        self.click_mode_label.setStyleSheet("color: #7f8c8d; font-style: italic;")
        cam_controls.addWidget(self.click_mode_label)
        cam_controls.addStretch()
        
        left_panel.addLayout(cam_controls)
        
        main_layout.addLayout(left_panel, stretch=3)
        
        # ═══ RIGHT SIDE: Controls ═══
        right_panel = QVBoxLayout()
        right_panel.setSpacing(8)
        
        # ─── Motor Controls ───
        motor_group = QGroupBox("Joint Controls")
        motor_layout = QVBoxLayout(motor_group)
        motor_layout.setSpacing(4)
        
        self.joint_controls: dict[str, JointControl] = {}
        joint_defs = [
            ("J1 Base",     -180, 180),
            ("J2 Shoulder", 0,    180),
            ("J3 Elbow",    0,    180),
            ("J4 Wrist",    -180, 180),
        ]
        for name, min_v, max_v in joint_defs:
            jc = JointControl(name, min_v, max_v, jog_callback=self._on_joint_control)
            self.joint_controls[name] = jc
            motor_layout.addWidget(jc)
        
        right_panel.addWidget(motor_group)
        
        # ─── Gripper Controls ───
        gripper_group = QGroupBox("Gripper")
        gripper_layout = QGridLayout(gripper_group)
        gripper_layout.setSpacing(6)
        
        # Gripper step size
        gripper_layout.addWidget(QLabel("Steps:"), 0, 0)
        self.gripper_steps = QSpinBox()
        self.gripper_steps.setRange(10, 5000)
        self.gripper_steps.setValue(200)
        self.gripper_steps.setSingleStep(50)
        gripper_layout.addWidget(self.gripper_steps, 0, 1)
        
        # Rotate buttons
        self.grip_rot_cw = QPushButton("↻ Rotate CW")
        self.grip_rot_ccw = QPushButton("↺ Rotate CCW")
        gripper_layout.addWidget(self.grip_rot_ccw, 1, 0)
        gripper_layout.addWidget(self.grip_rot_cw, 1, 1)
        
        # Tilt buttons
        self.grip_tilt_up = QPushButton("▲ Tilt Up")
        self.grip_tilt_down = QPushButton("▼ Tilt Down")
        gripper_layout.addWidget(self.grip_tilt_up, 2, 0)
        gripper_layout.addWidget(self.grip_tilt_down, 2, 1)
        
        right_panel.addWidget(gripper_group)
        
        # ─── Coordinate Input ───
        coord_group = QGroupBox("Coordinates (IK)")
        coord_layout = QGridLayout(coord_group)
        coord_layout.setSpacing(6)
        
        for i, (axis, default) in enumerate([("X", 200), ("Y", 0), ("Z", 0)]):
            lbl = QLabel(f"{axis}:")
            lbl.setStyleSheet("color: #00d4ff; font-weight: bold;")
            coord_layout.addWidget(lbl, 0, i * 2)
            
            sb = QDoubleSpinBox()
            sb.setRange(-600, 600)
            sb.setDecimals(1)
            sb.setValue(default)
            sb.setSuffix(" mm")
            coord_layout.addWidget(sb, 0, i * 2 + 1)
            setattr(self, f"coord_{axis.lower()}", sb)
        
        # Pitch control
        coord_layout.addWidget(QLabel("Pitch:"), 1, 0)
        self.coord_pitch = QDoubleSpinBox()
        self.coord_pitch.setRange(-180, 180)
        self.coord_pitch.setDecimals(1)
        self.coord_pitch.setValue(0)
        self.coord_pitch.setSuffix("°")
        coord_layout.addWidget(self.coord_pitch, 1, 1)
        
        # Go button
        self.go_btn = QPushButton("🎯  GO")
        self.go_btn.setStyleSheet("""
            QPushButton {
                background-color: #1e6f50; 
                font-size: 16px; 
                font-weight: bold;
                min-height: 36px;
                border: 2px solid #27ae60;
            }
            QPushButton:hover { background-color: #27ae60; }
        """)
        coord_layout.addWidget(self.go_btn, 1, 2, 1, 4)
        
        # Current position display
        self.pos_label = QLabel("Position: — / — / —")
        self.pos_label.setStyleSheet("color: #7f8c8d; padding: 4px;")
        coord_layout.addWidget(self.pos_label, 2, 0, 1, 6)
        
        right_panel.addWidget(coord_group)
        
        # ─── Action Buttons ───
        actions_layout = QHBoxLayout()
        
        self.home_btn = QPushButton("🏠  Home")
        self.home_btn.setObjectName("homeBtn")
        actions_layout.addWidget(self.home_btn)
        
        self.estop_btn = QPushButton("⛔  EMERGENCY STOP")
        self.estop_btn.setObjectName("emergencyStop")
        actions_layout.addWidget(self.estop_btn, stretch=1)
        
        right_panel.addLayout(actions_layout)
        
        # ─── Serial Log ───
        log_group = QGroupBox("Serial Log")
        log_layout = QVBoxLayout(log_group)
        self.log_view = QTextEdit()
        self.log_view.setObjectName("logView")
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(120)
        log_layout.addWidget(self.log_view)
        
        right_panel.addWidget(log_group)
        
        right_panel.addStretch()
        
        main_layout.addLayout(right_panel, stretch=2)
        
        # Status bar
        self.statusBar().showMessage("Ready — Connect serial port and camera to begin")
    
    def _connect_signals(self):
        # Connection
        self.refresh_ports_btn.clicked.connect(self._refresh_ports)
        self.refresh_cameras_btn.clicked.connect(self._refresh_cameras)
        self.connect_btn.clicked.connect(self._toggle_serial)
        self.camera_btn.clicked.connect(self._toggle_camera)
        self.calibrate_btn.clicked.connect(self._toggle_calibration)
        
        # Serial
        self.serial.connected.connect(self._on_serial_connected)
        self.serial.disconnected.connect(self._on_serial_disconnected)
        self.serial.status_received.connect(self._on_status_received)
        self.serial.error.connect(self._on_serial_error)
        self.serial.raw_log.connect(self._on_raw_log)
        
        # Controls
        self.go_btn.clicked.connect(self._on_go_clicked)
        self.home_btn.clicked.connect(self._on_home_clicked)
        self.estop_btn.clicked.connect(self._on_estop_clicked)
        
        # Gripper
        self.grip_rot_cw.clicked.connect(lambda: self.serial.gripper_rotate(self.gripper_steps.value()))
        self.grip_rot_ccw.clicked.connect(lambda: self.serial.gripper_rotate(-self.gripper_steps.value()))
        self.grip_tilt_up.clicked.connect(lambda: self.serial.gripper_tilt(self.gripper_steps.value()))
        self.grip_tilt_down.clicked.connect(lambda: self.serial.gripper_tilt(-self.gripper_steps.value()))
    
    # ─── Port / Camera Refresh ───
    
    def _refresh_ports(self):
        self.port_combo.clear()
        ports = SerialComm.list_ports()
        self.port_combo.addItems(ports)
        
        # Try auto-select ESP32
        esp_port = SerialComm.find_esp32()
        if esp_port and esp_port in ports:
            self.port_combo.setCurrentText(esp_port)
    
    def _refresh_cameras(self):
        self.camera_combo.clear()
        cams = list_cameras()
        for idx in cams:
            self.camera_combo.addItem(f"Camera {idx}", idx)
        if not cams:
            self.camera_combo.addItem("No cameras found", -1)
    
    # ─── Serial Connection ───
    
    def _toggle_serial(self):
        if self.serial.is_connected:
            self.serial.disconnect()
        else:
            port = self.port_combo.currentText()
            if port:
                self.serial.connect(port)
    
    @pyqtSlot(str)
    def _on_serial_connected(self, port: str):
        self.serial_status.setText("🟢")
        self.connect_btn.setText("Disconnect")
        self.statusBar().showMessage(f"Connected to {port}")
        self._status_timer.start()
    
    @pyqtSlot()
    def _on_serial_disconnected(self):
        self.serial_status.setText("🔴")
        self.connect_btn.setText("Connect")
        self._status_timer.stop()
    
    @pyqtSlot(dict)
    def _on_status_received(self, data: dict):
        """Handle status JSON from ESP32."""
        status = data.get("status", "")
        
        if "j1" in data:
            self.joint_controls["J1 Base"].set_value(data.get("j1", 0))
            self.joint_controls["J2 Shoulder"].set_value(data.get("j2", 0))
            self.joint_controls["J3 Elbow"].set_value(data.get("j3", 0))
            self.joint_controls["J4 Wrist"].set_value(data.get("j4", 0))
        
        if "x" in data:
            x, y, z = data.get("x", 0), data.get("y", 0), data.get("z", 0)
            moving = "🔄" if data.get("moving", False) else "✅"
            self.pos_label.setText(f"{moving}  X: {x:.1f}  Y: {y:.1f}  Z: {z:.1f} mm")
        
        if status == "error":
            msg = data.get("msg", "unknown error")
            self.statusBar().showMessage(f"⚠ Error: {msg}", 5000)
    
    @pyqtSlot(str)
    def _on_serial_error(self, msg: str):
        self.statusBar().showMessage(f"⚠ {msg}", 5000)
    
    @pyqtSlot(str)
    def _on_raw_log(self, line: str):
        self.log_view.append(line)
        # Auto-scroll to bottom
        scrollbar = self.log_view.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def _poll_status(self):
        if self.serial.is_connected:
            self.serial.request_status()
    
    # ─── Camera ───
    
    def _toggle_camera(self):
        if self._camera_running:
            self._stop_camera()
        else:
            self._start_camera()
    
    def _start_camera(self):
        idx = self.camera_combo.currentData()
        if idx is None or idx < 0:
            self.statusBar().showMessage("No camera selected", 3000)
            return
        
        self.camera_worker = CameraWorker(device_index=idx)
        self.camera_worker.frame_ready.connect(self._on_frame)
        self.camera_worker.error.connect(self._on_camera_error)
        self.camera_worker.camera_closed.connect(self._on_camera_closed)
        self.camera_worker.start()
        
        self._camera_running = True
        self.camera_btn.setText("Stop Camera")
        self.camera_status.setText("🟢")
    
    def _stop_camera(self):
        if self.camera_worker:
            self.camera_worker.stop()
            self.camera_worker = None
        self._camera_running = False
        self.camera_btn.setText("Start Camera")
        self.camera_status.setText("🔴")
        self.camera_view.clear()
        self.camera_view.setText("📷  Camera not connected")
    
    @pyqtSlot(QImage)
    def _on_frame(self, q_img: QImage):
        """Display camera frame with optional CV overlay."""
        # Apply click-to-move overlay if we have a raw frame
        if self.camera_worker:
            raw = self.camera_worker.get_last_frame()
            if raw is not None:
                overlaid = self.click_to_move.draw_overlay(raw)
                rgb = cv2.cvtColor(overlaid, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                bpl = ch * w
                q_img = QImage(rgb.data, w, h, bpl, QImage.Format.Format_RGB888).copy()
                self.camera_view.set_frame_size(w, h)
        
        # Scale to fit widget while keeping aspect ratio
        pixmap = QPixmap.fromImage(q_img)
        scaled = pixmap.scaled(
            self.camera_view.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )
        self.camera_view.setPixmap(scaled)
    
    def _on_camera_error(self, msg: str):
        self.statusBar().showMessage(f"📷 {msg}", 5000)
    
    def _on_camera_closed(self):
        self._camera_running = False
        self.camera_btn.setText("Start Camera")
        self.camera_status.setText("🔴")
    
    # ─── Click-to-Move ───
    
    def _on_camera_click(self, px: int, py: int):
        """Handle click on camera view."""
        self.click_to_move.set_click(px, py)
        coords = self.click_to_move.get_target_robot_coords()
        
        if coords:
            rx, ry, rz = coords
            self.coord_x.setValue(rx)
            self.coord_y.setValue(ry)
            self.coord_z.setValue(rz)
            
            # Auto-send if connected
            if self.serial.is_connected:
                self.serial.move_coord(rx, ry, rz, self.coord_pitch.value())
                self.statusBar().showMessage(
                    f"Click → X:{rx:.0f} Y:{ry:.0f} Z:{rz:.0f}", 3000
                )
    
    # ─── Calibration ───
    
    def _toggle_calibration(self):
        if self._calibrating:
            self._calibrating = False
            self._calib_timer.stop()
            self.calibrate_btn.setText("🔧 Calibrate Camera")
        else:
            if not self._camera_running:
                self.statusBar().showMessage("Start camera first", 3000)
                return
            self._calibrating = True
            self.click_to_move.open_calibration_window()
            self._calib_timer.start()
            self.calibrate_btn.setText("✖ Close Calibration")
    
    def _update_calibration(self):
        """Feed frames to the calibration window."""
        if not self._calibrating or not self.camera_worker:
            return
        
        frame = self.camera_worker.get_last_frame()
        if frame is not None:
            still_open = self.click_to_move.update_calibration_window(frame)
            if not still_open:
                self._calibrating = False
                self._calib_timer.stop()
                self.calibrate_btn.setText("🔧 Calibrate Camera")
    
    # ─── Joint Controls ───
    
    def _on_joint_control(self, name: str, mode: str, value):
        """Handle joint jog/angle from UI controls."""
        joint_num = self.JOINT_MAP.get(name)
        if not joint_num:
            return
        
        if mode == "steps":
            self.serial.jog_joint(joint_num, steps=int(value))
        elif mode == "angle":
            self.serial.jog_joint(joint_num, angle=float(value))
    
    # ─── Coordinate / Actions ───
    
    def _on_go_clicked(self):
        x = self.coord_x.value()
        y = self.coord_y.value()
        z = self.coord_z.value()
        pitch = self.coord_pitch.value()
        self.serial.move_coord(x, y, z, pitch)
        self.statusBar().showMessage(f"Moving to X:{x:.1f} Y:{y:.1f} Z:{z:.1f} P:{pitch:.1f}°", 3000)
    
    def _on_home_clicked(self):
        self.serial.home()
        self.statusBar().showMessage("Homing all axes...", 3000)
    
    def _on_estop_clicked(self):
        self.serial.emergency_stop()
        self.statusBar().showMessage("⛔ EMERGENCY STOP ACTIVATED", 5000)
        self.estop_btn.setText("🔓 Resume")
        self.estop_btn.clicked.disconnect()
        self.estop_btn.clicked.connect(self._on_resume_clicked)
    
    def _on_resume_clicked(self):
        self.serial.resume()
        self.statusBar().showMessage("Resumed", 3000)
        self.estop_btn.setText("⛔  EMERGENCY STOP")
        self.estop_btn.clicked.disconnect()
        self.estop_btn.clicked.connect(self._on_estop_clicked)
    
    # ─── Cleanup ───
    
    def closeEvent(self, event):
        """Clean shutdown."""
        self._stop_camera()
        self.serial.disconnect()
        self._status_timer.stop()
        self._calib_timer.stop()
        super().closeEvent(event)

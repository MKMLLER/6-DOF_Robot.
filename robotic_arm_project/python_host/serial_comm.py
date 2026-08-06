"""
Serial communication module for ESP32 robotic arm.
JSON-based protocol over USB serial (115200 baud).
"""

import json
import threading
import time
from typing import Optional, Callable

import serial
import serial.tools.list_ports
from PyQt6.QtCore import QObject, pyqtSignal, QThread


class SerialWorker(QThread):
    """Background thread that reads serial data and emits signals."""
    
    data_received = pyqtSignal(dict)    # parsed JSON response
    connection_lost = pyqtSignal()       # connection dropped
    raw_received = pyqtSignal(str)       # raw line for debug log
    
    def __init__(self, ser: serial.Serial):
        super().__init__()
        self._ser = ser
        self._running = True
    
    def run(self):
        buffer = ""
        while self._running:
            try:
                if self._ser and self._ser.is_open and self._ser.in_waiting:
                    raw = self._ser.read(self._ser.in_waiting).decode("utf-8", errors="replace")
                    buffer += raw
                    
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        
                        self.raw_received.emit(line)
                        
                        try:
                            data = json.loads(line)
                            self.data_received.emit(data)
                        except json.JSONDecodeError:
                            # Non-JSON line (debug output from ESP32)
                            pass
                else:
                    self.msleep(10)
                    
            except (serial.SerialException, OSError):
                self.connection_lost.emit()
                break
    
    def stop(self):
        self._running = False
        self.wait(2000)


class SerialComm(QObject):
    """
    High-level serial communication manager.
    
    Signals:
        connected(str)      — emitted when connection established (port name)
        disconnected()      — emitted when disconnected
        status_received(dict) — parsed status JSON from ESP32
        error(str)          — error message
        raw_log(str)        — raw serial line for debug
    """
    
    connected = pyqtSignal(str)
    disconnected = pyqtSignal()
    status_received = pyqtSignal(dict)
    error = pyqtSignal(str)
    raw_log = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self._ser: Optional[serial.Serial] = None
        self._worker: Optional[SerialWorker] = None
        self._lock = threading.Lock()
    
    @staticmethod
    def list_ports() -> list[str]:
        """List available COM ports."""
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]
    
    @staticmethod
    def find_esp32() -> Optional[str]:
        """Try to auto-detect an ESP32 by USB VID/PID."""
        for p in serial.tools.list_ports.comports():
            desc = (p.description or "").lower()
            if "cp210" in desc or "ch340" in desc or "silicon labs" in desc or "usb" in desc:
                return p.device
        ports = serial.tools.list_ports.comports()
        if ports:
            return ports[0].device
        return None
    
    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open
    
    def connect(self, port: str, baudrate: int = 115200) -> bool:
        """Open serial connection to ESP32."""
        try:
            self.disconnect()
            
            self._ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1,
                write_timeout=1.0
            )
            
            # Small delay for ESP32 boot
            time.sleep(0.5)
            
            # Flush any boot messages
            if self._ser.in_waiting:
                self._ser.read(self._ser.in_waiting)
            
            # Start reader thread
            self._worker = SerialWorker(self._ser)
            self._worker.data_received.connect(self._on_data)
            self._worker.connection_lost.connect(self._on_connection_lost)
            self._worker.raw_received.connect(self.raw_log.emit)
            self._worker.start()
            
            self.connected.emit(port)
            return True
            
        except serial.SerialException as e:
            self.error.emit(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection."""
        if self._worker:
            self._worker.stop()
            self._worker = None
        
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        
        self.disconnected.emit()
    
    def send_command(self, cmd: dict) -> bool:
        """Send a JSON command to ESP32."""
        with self._lock:
            if not self.is_connected:
                self.error.emit("Not connected")
                return False
            
            try:
                line = json.dumps(cmd, separators=(",", ":")) + "\n"
                self._ser.write(line.encode("utf-8"))
                self._ser.flush()
                return True
            except serial.SerialException as e:
                self.error.emit(f"Send failed: {e}")
                return False
    
    # ─── Convenience command methods ───
    
    def jog_joint(self, joint: int, steps: int = None, angle: float = None):
        """Jog a single joint by steps or to an angle."""
        cmd = {"cmd": "jog", "joint": joint}
        if steps is not None:
            cmd["steps"] = steps
        elif angle is not None:
            cmd["angle"] = angle
        self.send_command(cmd)
    
    def move_coord(self, x: float, y: float, z: float, pitch: float = None):
        """Move to XYZ coordinates using inverse kinematics."""
        cmd = {"cmd": "coord", "x": x, "y": y, "z": z}
        if pitch is not None:
            cmd["pitch"] = pitch
        self.send_command(cmd)
    
    def gripper_rotate(self, steps: int):
        """Rotate gripper (opposite motor directions)."""
        self.send_command({"cmd": "gripper_rotate", "steps": steps})
    
    def gripper_tilt(self, steps: int):
        """Tilt gripper up/down (same motor direction)."""
        self.send_command({"cmd": "gripper_tilt", "steps": steps})
    
    def home(self):
        """Home all axes."""
        self.send_command({"cmd": "home"})

    def calibrate(self, j1: float = None, j2: float = None, j3: float = None, j4: float = None):
        """Calibrate current physical motor positions as specified angles without moving motors."""
        cmd = {"cmd": "calibrate"}
        if j1 is not None:
            cmd["j1"] = j1
        if j2 is not None:
            cmd["j2"] = j2
        if j3 is not None:
            cmd["j3"] = j3
        if j4 is not None:
            cmd["j4"] = j4
        self.send_command(cmd)
    
    def emergency_stop(self):
        """Emergency stop — halt all motors immediately."""
        self.send_command({"cmd": "stop"})
    
    def resume(self):
        """Resume after emergency stop."""
        self.send_command({"cmd": "resume"})
    
    def request_status(self):
        """Request current position status."""
        self.send_command({"cmd": "status"})
    
    def set_pitch(self, angle: float):
        """Set desired end-effector pitch for IK."""
        self.send_command({"cmd": "set_pitch", "angle": angle})
    
    # ─── Internal handlers ───
    
    def _on_data(self, data: dict):
        """Handle parsed JSON from ESP32."""
        self.status_received.emit(data)
    
    def _on_connection_lost(self):
        """Handle unexpected disconnect."""
        self.disconnect()
        self.error.emit("Connection lost")

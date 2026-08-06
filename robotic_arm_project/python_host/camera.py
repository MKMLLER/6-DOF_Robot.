"""
Camera capture module.
USB camera capture in a background thread with frame buffer for PyQt6 UI.
"""

import cv2
import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal, QMutex, QMutexLocker
from PyQt6.QtGui import QImage


class CameraWorker(QThread):
    """
    Background thread that captures frames from a USB camera.
    
    Signals:
        frame_ready(QImage)  — new frame available for display
        error(str)           — camera error message
        camera_closed()      — camera was released
    """
    
    frame_ready = pyqtSignal(QImage)
    error = pyqtSignal(str)
    camera_closed = pyqtSignal()
    
    def __init__(self, device_index: int = 0, width: int = 640, height: int = 480, fps: int = 30):
        super().__init__()
        self._device_index = device_index
        self._width = width
        self._height = height
        self._fps = fps
        self._running = False
        self._cap = None
        self._mutex = QMutex()
        self._last_frame: np.ndarray | None = None
    
    @property
    def is_running(self) -> bool:
        return self._running
    
    def get_last_frame(self) -> np.ndarray | None:
        """Get the most recent raw frame (BGR numpy array)."""
        with QMutexLocker(self._mutex):
            if self._last_frame is not None:
                return self._last_frame.copy()
        return None
    
    @property
    def frame_width(self) -> int:
        return self._width
    
    @property
    def frame_height(self) -> int:
        return self._height
    
    def run(self):
        self._running = True
        
        self._cap = cv2.VideoCapture(self._device_index, cv2.CAP_DSHOW)
        
        if not self._cap.isOpened():
            # Fallback without DirectShow
            self._cap = cv2.VideoCapture(self._device_index)
        
        if not self._cap.isOpened():
            self.error.emit(f"Cannot open camera {self._device_index}")
            self._running = False
            return
        
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(cv2.CAP_PROP_FPS, self._fps)
        
        # Read actual resolution
        self._width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                self.error.emit("Camera read failed")
                break
            
            # Store raw frame
            with QMutexLocker(self._mutex):
                self._last_frame = frame.copy()
            
            # Convert BGR → RGB for Qt
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            q_img = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            
            # Emit a copy (the data buffer won't survive the loop iteration)
            self.frame_ready.emit(q_img.copy())
            
            self.msleep(int(1000 / self._fps))
        
        if self._cap:
            self._cap.release()
        self.camera_closed.emit()
    
    def stop(self):
        self._running = False
        self.wait(3000)


def list_cameras(max_check: int = 2) -> list[int]:
    """
    Probe available camera indices cleanly.
    Returns list of device indices that can be opened.
    """
    available = []
    for i in range(max_check):
        try:
            cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
            if cap.isOpened():
                available.append(i)
                cap.release()
        except Exception:
            pass
    if not available:
        available = [0]
    return available



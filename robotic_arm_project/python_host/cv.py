"""
Computer vision module for click-to-move functionality.
Maps pixel coordinates from camera frame to robot workspace coordinates.
Provides OpenCV trackbar-based calibration UI.
"""

import cv2
import numpy as np
from dataclasses import dataclass, field


@dataclass
class CalibrationParams:
    """
    Camera-to-robot coordinate mapping parameters.
    Pixel (px, py) → Robot (rx, ry, rz):
        rx = (px - offset_x) * scale_x
        rz = (py - offset_y) * scale_z
        ry = fixed_y  (height is fixed, adjustable via UI)
    """
    offset_x: float = 320.0    # pixel X origin (center of frame)
    offset_y: float = 240.0    # pixel Y origin (center of frame)
    scale_x: float = 1.0       # pixels to mm scale X
    scale_z: float = 1.0       # pixels to mm scale Z  
    fixed_y: float = 0.0       # fixed robot Y coordinate (height)
    rotation_deg: float = 0.0  # rotation correction (degrees)


class ClickToMove:
    """
    Handles pixel-to-robot coordinate conversion and visual overlays.
    """
    
    def __init__(self):
        self.params = CalibrationParams()
        self._target_pixel: tuple[int, int] | None = None
        self._target_robot: tuple[float, float, float] | None = None
        self._calibration_window_open = False
    
    def pixel_to_robot(self, px: int, py: int) -> tuple[float, float, float]:
        """
        Convert pixel coordinates to robot workspace coordinates.
        
        Args:
            px: pixel X coordinate
            py: pixel Y coordinate
            
        Returns:
            (robot_x, robot_y, robot_z) in mm
        """
        # Center-relative pixel coordinates
        cx = px - self.params.offset_x
        cy = py - self.params.offset_y
        
        # Apply rotation if needed
        if abs(self.params.rotation_deg) > 0.01:
            rad = np.radians(self.params.rotation_deg)
            cos_r = np.cos(rad)
            sin_r = np.sin(rad)
            rx = cx * cos_r - cy * sin_r
            ry = cx * sin_r + cy * cos_r
            cx, cy = rx, ry
        
        # Scale to robot coordinates
        robot_x = cx * self.params.scale_x
        robot_z = cy * self.params.scale_z
        robot_y = self.params.fixed_y
        
        return (robot_x, robot_y, robot_z)
    
    def set_click(self, px: int, py: int):
        """Register a click on the camera frame."""
        self._target_pixel = (px, py)
        self._target_robot = self.pixel_to_robot(px, py)
    
    def get_target_robot_coords(self) -> tuple[float, float, float] | None:
        """Get the robot coordinates of the last click."""
        return self._target_robot
    
    def clear_target(self):
        """Clear the current target."""
        self._target_pixel = None
        self._target_robot = None
    
    def draw_overlay(self, frame: np.ndarray) -> np.ndarray:
        """
        Draw targeting overlay on the camera frame.
        
        Args:
            frame: BGR numpy array (camera frame)
            
        Returns:
            frame with overlay drawn
        """
        overlay = frame.copy()
        h, w = overlay.shape[:2]
        
        # Draw center crosshair (thin, semi-transparent)
        cx, cy = int(self.params.offset_x), int(self.params.offset_y)
        color_center = (100, 100, 100)
        cv2.line(overlay, (cx - 20, cy), (cx + 20, cy), color_center, 1)
        cv2.line(overlay, (cx, cy - 20), (cx, cy + 20), color_center, 1)
        
        # Draw target if clicked
        if self._target_pixel:
            tx, ty = self._target_pixel
            color_target = (0, 255, 0)  # green
            
            # Crosshair at target
            cv2.drawMarker(overlay, (tx, ty), color_target, 
                          cv2.MARKER_CROSS, 30, 2)
            
            # Circle around target
            cv2.circle(overlay, (tx, ty), 15, color_target, 2)
            
            # Coordinate text
            if self._target_robot:
                rx, ry, rz = self._target_robot
                text = f"X:{rx:.0f} Y:{ry:.0f} Z:{rz:.0f}"
                cv2.putText(overlay, text, (tx + 20, ty - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_target, 1)
        
        return overlay
    
    def open_calibration_window(self):
        """
        Open an OpenCV window with trackbars for manual calibration.
        This runs in a blocking loop — call from a separate thread or 
        use the non-blocking update method.
        """
        window_name = "Camera Calibration"
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        
        # Trackbar ranges: value = actual * 10 for float precision
        cv2.createTrackbar("Offset X", window_name, int(self.params.offset_x), 1280,
                          lambda v: setattr(self.params, 'offset_x', float(v)))
        cv2.createTrackbar("Offset Y", window_name, int(self.params.offset_y), 960,
                          lambda v: setattr(self.params, 'offset_y', float(v)))
        cv2.createTrackbar("Scale X*10", window_name, int(self.params.scale_x * 10), 100,
                          lambda v: setattr(self.params, 'scale_x', v / 10.0) if v > 0 else None)
        cv2.createTrackbar("Scale Z*10", window_name, int(self.params.scale_z * 10), 100,
                          lambda v: setattr(self.params, 'scale_z', v / 10.0) if v > 0 else None)
        cv2.createTrackbar("Fixed Y", window_name, int(self.params.fixed_y + 500), 1000,
                          lambda v: setattr(self.params, 'fixed_y', float(v - 500)))
        cv2.createTrackbar("Rotation", window_name, int(self.params.rotation_deg + 180), 360,
                          lambda v: setattr(self.params, 'rotation_deg', float(v - 180)))
        
        self._calibration_window_open = True
    
    def update_calibration_window(self, frame: np.ndarray) -> bool:
        """
        Update the calibration window with current frame and overlay.
        Call this in a loop while calibration window is open.
        
        Returns:
            True if window is still open, False if user closed it.
        """
        if not self._calibration_window_open:
            return False
        
        window_name = "Camera Calibration"
        
        # Draw calibration overlay
        display = self.draw_overlay(frame)
        
        # Draw grid lines for reference
        h, w = display.shape[:2]
        cx, cy = int(self.params.offset_x), int(self.params.offset_y)
        
        # Grid at every 50mm equivalent
        if self.params.scale_x > 0:
            step_px = int(50 / self.params.scale_x)
            if step_px > 5:
                for x in range(cx % step_px, w, step_px):
                    cv2.line(display, (x, 0), (x, h), (40, 40, 40), 1)
                for y in range(cy % step_px, h, step_px):
                    cv2.line(display, (0, y), (w, y), (40, 40, 40), 1)
        
        # Parameter text
        info_lines = [
            f"Offset: ({self.params.offset_x:.0f}, {self.params.offset_y:.0f})",
            f"Scale: X={self.params.scale_x:.1f}  Z={self.params.scale_z:.1f}",
            f"Fixed Y: {self.params.fixed_y:.0f}mm",
            f"Rotation: {self.params.rotation_deg:.0f} deg",
            "Press 'q' to close"
        ]
        for i, line in enumerate(info_lines):
            cv2.putText(display, line, (10, 25 + i * 22),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 200), 1)
        
        cv2.imshow(window_name, display)
        
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:
            cv2.destroyWindow(window_name)
            self._calibration_window_open = False
            return False
        
        return True
    
    def get_params_dict(self) -> dict:
        """Export current calibration params as dict."""
        return {
            "offset_x": self.params.offset_x,
            "offset_y": self.params.offset_y,
            "scale_x": self.params.scale_x,
            "scale_z": self.params.scale_z,
            "fixed_y": self.params.fixed_y,
            "rotation_deg": self.params.rotation_deg,
        }
    
    def load_params_dict(self, d: dict):
        """Load calibration params from dict."""
        self.params.offset_x = d.get("offset_x", self.params.offset_x)
        self.params.offset_y = d.get("offset_y", self.params.offset_y)
        self.params.scale_x = d.get("scale_x", self.params.scale_x)
        self.params.scale_z = d.get("scale_z", self.params.scale_z)
        self.params.fixed_y = d.get("fixed_y", self.params.fixed_y)
        self.params.rotation_deg = d.get("rotation_deg", self.params.rotation_deg)

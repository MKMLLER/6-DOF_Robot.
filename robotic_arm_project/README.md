# Robotic Arm Control System

6-axis robotic arm control with ESP32 firmware and PyQt6 desktop application.

## Project Structure

```
robotic_arm_project/
├── esp32_firmware/
│   └── esp32_firmware.ino    # ESP32 firmware (C++, PlatformIO/Arduino)
├── python_host/
│   ├── main.py               # Entry point
│   ├── ui_main.py            # Main window UI (PyQt6)
│   ├── serial_comm.py        # Serial communication module
│   ├── camera.py             # USB camera capture
│   ├── cv.py                 # Computer vision / click-to-move
│   └── requirements.txt      # Python dependencies
└── README.md                 # This file
```

## Serial Protocol

JSON-based communication over USB serial at **115200 baud**.

### Commands (PC → ESP32)

| Command | JSON | Description |
|---------|------|-------------|
| Jog by steps | `{"cmd":"jog","joint":1,"steps":200}` | Move joint by N steps (relative) |
| Jog to angle | `{"cmd":"jog","joint":1,"angle":45.0}` | Move joint to angle (absolute) |
| Coordinate | `{"cmd":"coord","x":200,"y":100,"z":50,"pitch":0}` | Move to XYZ via inverse kinematics |
| Gripper rotate | `{"cmd":"gripper_rotate","steps":200}` | Rotate gripper (opposite motor dirs) |
| Gripper tilt | `{"cmd":"gripper_tilt","steps":200}` | Tilt gripper up/down (same motor dirs) |
| Home | `{"cmd":"home"}` | Home all axes |
| Stop | `{"cmd":"stop"}` | Emergency stop |
| Resume | `{"cmd":"resume"}` | Resume after e-stop |
| Status | `{"cmd":"status"}` | Request current position |
| Set pitch | `{"cmd":"set_pitch","angle":0}` | Set desired end-effector pitch |

### Responses (ESP32 → PC)

```json
{
  "status": "ok",
  "msg": "coord move",
  "j1": 15.2,
  "j2": 67.3,
  "j3": 45.8,
  "j4": -12.5,
  "x": 200.0,
  "y": 100.0,
  "z": 50.0,
  "moving": true
}
```

## Joint Configuration

| Joint | Function | Step/Dir Pins | MaxSpeed | Reduction | Microsteps |
|-------|----------|---------------|----------|-----------|------------|
| J1 | Base rotation | 25/26 | 1000 | 6.0 | 4 |
| J2 | Shoulder | 16/18 | 8000 | 48.33 | 4 |
| J3 | Elbow | 32/33 | 4108 | 24.95 | 4 |
| J4 | Wrist pitch | 17/19 | 1000 | 36.0 | 8 |
| J5 | Gripper 1 | 23/22 | 1000 | — | — |
| J6 | Gripper 2 | 27/14 | 1000 | — | — |

## Inverse Kinematics

- **Arm geometry**: Upper arm = 250mm, Forearm = 275mm
- **J1**: Base rotation via `atan2(Z, X)` in XZ plane
- **J3**: Elbow angle via law of cosines
- **J2**: Shoulder angle = triangle angle + elevation angle
- **J4**: Wrist pitch compensation: `J4 = desired_pitch - (J2 + J3 - 180°)`

## Setup

### ESP32 Firmware
1. Open in PlatformIO (or Arduino IDE)
2. Install library: `ArduinoJson` (by Benoit Blanchon)
3. Board: ESP32 Dev Module
4. Upload

### Python Application
```bash
cd python_host
pip install -r requirements.txt
python main.py
```

## Camera Calibration

1. Click "Calibrate Camera" in the app
2. Use OpenCV trackbars to adjust:
   - **Offset X/Y**: Center of the robot workspace in pixels
   - **Scale X/Z**: Pixels-to-millimeters conversion
   - **Fixed Y**: Robot Y coordinate (height) 
   - **Rotation**: Rotation correction if camera is tilted
3. Press `q` to close calibration window

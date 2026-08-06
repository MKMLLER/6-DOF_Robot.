/*
 * ESP32 Robotic Arm Firmware
 * 6-axis manipulator control via JSON serial protocol
 * 
 * Motors:
 *   J1 (base rotation)   — stepper1  (pins 25/26)
 *   J2 (shoulder)        — stepper2  (pins 16/18)
 *   J3 (elbow)           — stepper3  (pins 32/33)
 *   J4 (wrist pitch)     — stepper4  (pins 17/19)
 *   J5/J6 gripper pair   — stepper56_1 (pins 23/22), stepper56_2 (pins 27/14)
 *     Same direction   → tilt gripper up/down
 *     Opposite direction → rotate gripper
 */

#include <Arduino.h>
#include <AccelStepper.h>
#include <ArduinoJson.h>
#include <math.h>

// ─── Motor Pin Definitions ───
AccelStepper stepper1(AccelStepper::DRIVER, 25, 26);    // J1 — base
AccelStepper stepper2(AccelStepper::DRIVER, 16, 18);    // J2 — shoulder
AccelStepper stepper3(AccelStepper::DRIVER, 32, 33);    // J3 — elbow
AccelStepper stepper4(AccelStepper::DRIVER, 17, 19);    // J4 — wrist pitch
AccelStepper stepper56_1(AccelStepper::DRIVER, 23, 22); // Gripper motor 1
AccelStepper stepper56_2(AccelStepper::DRIVER, 27, 14); // Gripper motor 2

// ─── Kinematics Constants ───
static const float FIRST_PART_LENGTH  = 250.0f;   // mm — upper arm
static const float SECOND_PART_LENGTH = 275.0f;   // mm — forearm
static const float J1_RED  = 6.0f;     // J1 gear reduction
static const float J2_RED  = 48.33f;   // J2 gear reduction
static const float J3_RED  = 24.95f;   // J3 gear reduction
static const float J4_RED  = 36.0f;    // J4 gear reduction
static const float J1_DRV  = 4.0f;     // J1 driver microsteps
static const float J2_DRV  = 4.0f;     // J2 driver microsteps
static const float J3_DRV  = 4.0f;     // J3 driver microsteps
static const float J4_DRV  = 8.0f;     // J4 driver microsteps
static const float PI_F    = 3.14159265f;

// ─── Current state ───
float currentJ1 = 0.0f, currentJ2 = 90.0f, currentJ3 = 90.0f, currentJ4 = 0.0f;
float currentX = 275.0f, currentY = 250.0f, currentZ = 0.0f;
float desiredPitch = 0.0f;  // desired end-effector pitch angle (degrees)
bool eStop = false;

// ─── Helper: Convert angle to stepper steps ───
long goalToSteps(float angleDeg, float driverMicrosteps, float reduction) {
  float stepsPerDeg = (driverMicrosteps * 200.0f * reduction) / 360.0f;
  return (long)(stepsPerDeg * angleDeg);
}

// ─── Helper: Vector length ───
float vectorLength2D(float a, float b) {
  return sqrtf(a * a + b * b);
}

// ─── Forward Kinematics: J1, J2, J3 → XYZ ───
void updateFK() {
  float j1_rad = currentJ1 * PI_F / 180.0f;
  float j2_rad = currentJ2 * PI_F / 180.0f;
  float j3_rad = currentJ3 * PI_F / 180.0f;

  // Upper arm projection
  float r1 = FIRST_PART_LENGTH * cosf(j2_rad);
  float y1 = FIRST_PART_LENGTH * sinf(j2_rad);

  // Forearm projection (angle relative to horizontal = J2 + J3 - 180 deg)
  float forearm_angle_rad = j2_rad + j3_rad - PI_F;
  float r2 = SECOND_PART_LENGTH * cosf(forearm_angle_rad);
  float y2 = SECOND_PART_LENGTH * sinf(forearm_angle_rad);

  float r_total = r1 + r2;
  currentX = r_total * cosf(j1_rad);
  currentZ = r_total * sinf(j1_rad);
  currentY = y1 + y2;
}

// ─── Inverse Kinematics: XYZ → J1, J2, J3, J4 ───
bool setStepAngle(float X, float Y, float Z) {
  float r = vectorLength2D(X, Z);
  float rY = vectorLength2D(r, Y);

  // Reach check
  float maxReach = FIRST_PART_LENGTH + SECOND_PART_LENGTH - 1.0f;
  float minReach = fabsf(FIRST_PART_LENGTH - SECOND_PART_LENGTH) + 1.0f;
  if (rY > maxReach || rY < minReach) {
    return false;  // Out of workspace reach
  }

  // J1 angle (base)
  float J1_deg = 0.0f;
  if (r > 0.001f) {
    float J1_rad = atan2f(Z, X);
    J1_deg = J1_rad * 180.0f / PI_F;
  }

  // J3 angle (elbow) via law of cosines
  float J3_cos = (rY * rY - FIRST_PART_LENGTH * FIRST_PART_LENGTH - SECOND_PART_LENGTH * SECOND_PART_LENGTH) 
                 / (-2.0f * FIRST_PART_LENGTH * SECOND_PART_LENGTH);
  if (J3_cos > 1.0f) J3_cos = 1.0f;
  if (J3_cos < -1.0f) J3_cos = -1.0f;

  float J3_rad = acosf(J3_cos);
  float J3_deg = J3_rad * 180.0f / PI_F;

  // J2 angle (shoulder) = elevation angle + inner triangle angle
  float elevation_rad = atan2f(Y, r);

  float J2_tri_cos = (SECOND_PART_LENGTH * SECOND_PART_LENGTH - FIRST_PART_LENGTH * FIRST_PART_LENGTH - rY * rY) 
                     / (-2.0f * rY * FIRST_PART_LENGTH);
  if (J2_tri_cos > 1.0f) J2_tri_cos = 1.0f;
  if (J2_tri_cos < -1.0f) J2_tri_cos = -1.0f;

  float J2_tri_rad = acosf(J2_tri_cos);
  float J2_deg = (elevation_rad + J2_tri_rad) * 180.0f / PI_F;

  // J4 angle (wrist pitch compensation)
  float J4_deg = desiredPitch - (J2_deg + J3_deg - 180.0f);

  // Apply target positions to steppers
  stepper1.moveTo(goalToSteps(J1_deg, J1_DRV, J1_RED));
  stepper2.moveTo(goalToSteps(J2_deg, J2_DRV, J2_RED));
  stepper3.moveTo(goalToSteps(J3_deg, J3_DRV, J3_RED));
  stepper4.moveTo(goalToSteps(J4_deg, J4_DRV, J4_RED));

  // Update internal joint angles and FK coordinates
  currentJ1 = J1_deg;
  currentJ2 = J2_deg;
  currentJ3 = J3_deg;
  currentJ4 = J4_deg;
  currentX = X;
  currentY = Y;
  currentZ = Z;

  return true;
}

// ─── Send JSON response ───
void sendStatus(const char* status, const char* msg = nullptr) {
  JsonDocument doc;
  doc["status"] = status;
  if (msg) doc["msg"] = msg;
  doc["j1"] = currentJ1;
  doc["j2"] = currentJ2;
  doc["j3"] = currentJ3;
  doc["j4"] = currentJ4;
  doc["x"] = currentX;
  doc["y"] = currentY;
  doc["z"] = currentZ;
  
  bool allStopped = (stepper1.distanceToGo() == 0 &&
                     stepper2.distanceToGo() == 0 &&
                     stepper3.distanceToGo() == 0 &&
                     stepper4.distanceToGo() == 0);
  doc["moving"] = !allStopped;
  
  serializeJson(doc, Serial);
  Serial.println();
}

// ─── Process incoming JSON command ───
void processCommand(const String& input) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, input);
  
  if (err) {
    sendStatus("error", "JSON parse error");
    return;
  }

  const char* cmd = doc["cmd"];
  if (!cmd) {
    sendStatus("error", "missing cmd");
    return;
  }

  // ── Emergency Stop ──
  if (strcmp(cmd, "stop") == 0) {
    eStop = true;
    stepper1.setSpeed(0); stepper1.moveTo(stepper1.currentPosition());
    stepper2.setSpeed(0); stepper2.moveTo(stepper2.currentPosition());
    stepper3.setSpeed(0); stepper3.moveTo(stepper3.currentPosition());
    stepper4.setSpeed(0); stepper4.moveTo(stepper4.currentPosition());
    stepper56_1.setSpeed(0); stepper56_1.moveTo(stepper56_1.currentPosition());
    stepper56_2.setSpeed(0); stepper56_2.moveTo(stepper56_2.currentPosition());
    sendStatus("ok", "emergency stop");
    return;
  }

  // ── Resume after e-stop ──
  if (strcmp(cmd, "resume") == 0) {
    eStop = false;
    sendStatus("ok", "resumed");
    return;
  }

  if (eStop) {
    sendStatus("error", "emergency stop active, send resume");
    return;
  }

  // ── Status query ──
  if (strcmp(cmd, "status") == 0) {
    sendStatus("ok");
    return;
  }

  // ── Calibrate current position to specified angles ──
  if (strcmp(cmd, "calibrate") == 0) {
    float j1_cal = doc["j1"] | currentJ1;
    float j2_cal = doc["j2"] | currentJ2;
    float j3_cal = doc["j3"] | currentJ3;
    float j4_cal = doc["j4"] | currentJ4;

    // If no specific angles provided in JSON, default to Home 90-degree pose
    if (!doc.containsKey("j1") && !doc.containsKey("j2") && !doc.containsKey("j3") && !doc.containsKey("j4")) {
      j1_cal = 0.0f;
      j2_cal = 90.0f;
      j3_cal = 90.0f;
      j4_cal = 0.0f;
    }

    stepper1.setCurrentPosition(goalToSteps(j1_cal, J1_DRV, J1_RED));
    stepper2.setCurrentPosition(goalToSteps(j2_cal, J2_DRV, J2_RED));
    stepper3.setCurrentPosition(goalToSteps(j3_cal, J3_DRV, J3_RED));
    stepper4.setCurrentPosition(goalToSteps(j4_cal, J4_DRV, J4_RED));

    currentJ1 = j1_cal;
    currentJ2 = j2_cal;
    currentJ3 = j3_cal;
    currentJ4 = j4_cal;

    updateFK();
    sendStatus("ok", "calibrated");
    return;
  }

  // ── Home all axes ──
  if (strcmp(cmd, "home") == 0) {
    stepper1.moveTo(goalToSteps(0, J1_DRV, J1_RED));
    stepper2.moveTo(goalToSteps(90, J2_DRV, J2_RED));
    stepper3.moveTo(goalToSteps(90, J3_DRV, J3_RED));
    stepper4.moveTo(goalToSteps(0, J4_DRV, J4_RED));
    currentJ1 = 0; currentJ2 = 90; currentJ3 = 90; currentJ4 = 0;
    updateFK();
    sendStatus("ok", "homing");
    return;
  }

  // ── Jog single joint ──
  if (strcmp(cmd, "jog") == 0) {
    int joint = doc["joint"] | 0;
    
    if (doc["steps"].is<int>()) {
      long steps = doc["steps"];
      switch (joint) {
        case 1: stepper1.move(steps); break;
        case 2: stepper2.move(steps); break;
        case 3: stepper3.move(steps); break;
        case 4: stepper4.move(steps); break;
        default: sendStatus("error", "invalid joint 1-4"); return;
      }
      sendStatus("ok", "jog steps");
    }
    else if (doc["angle"].is<float>() || doc["angle"].is<int>()) {
      float angle = doc["angle"];
      switch (joint) {
        case 1: 
          stepper1.moveTo(goalToSteps(angle, J1_DRV, J1_RED));
          currentJ1 = angle;
          updateFK();
          break;
        case 2: 
          stepper2.moveTo(goalToSteps(angle, J2_DRV, J2_RED));
          currentJ2 = angle;
          updateFK();
          break;
        case 3: 
          stepper3.moveTo(goalToSteps(angle, J3_DRV, J3_RED));
          currentJ3 = angle;
          updateFK();
          break;
        case 4: 
          stepper4.moveTo(goalToSteps(angle, J4_DRV, J4_RED));
          currentJ4 = angle;
          updateFK();
          break;
        default: sendStatus("error", "invalid joint 1-4"); return;
      }
      sendStatus("ok", "jog angle");
    }
    else {
      sendStatus("error", "need steps or angle");
    }
    return;
  }

  // ── Coordinate move (IK) ──
  if (strcmp(cmd, "coord") == 0) {
    float x = doc["x"] | 0.0f;
    float y = doc["y"] | 0.0f;
    float z = doc["z"] | 0.0f;
    
    if (doc["pitch"].is<float>() || doc["pitch"].is<int>()) {
      desiredPitch = doc["pitch"];
    }
    
    if (setStepAngle(x, y, z)) {
      sendStatus("ok", "coord move");
    } else {
      sendStatus("error", "out of range");
    }
    return;
  }

  // ── Gripper rotate ──
  if (strcmp(cmd, "gripper_rotate") == 0) {
    long steps = doc["steps"] | 0;
    stepper56_1.move(steps);
    stepper56_2.move(-steps);
    sendStatus("ok", "gripper rotate");
    return;
  }

  // ── Gripper tilt ──
  if (strcmp(cmd, "gripper_tilt") == 0) {
    long steps = doc["steps"] | 0;
    stepper56_1.move(steps);
    stepper56_2.move(steps);
    sendStatus("ok", "gripper tilt");
    return;
  }

  // ── Set desired pitch ──
  if (strcmp(cmd, "set_pitch") == 0) {
    desiredPitch = doc["angle"] | 0.0f;
    sendStatus("ok", "pitch set");
    return;
  }

  sendStatus("error", "unknown cmd");
}

// ─── Serial reader task (FreeRTOS) ───
void serialTask(void* pvParameters) {
  String buffer = "";
  for (;;) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        buffer.trim();
        if (buffer.length() > 0) {
          processCommand(buffer);
          buffer = "";
        }
      } else {
        buffer += c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ─── Setup ───
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(2, OUTPUT);

  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(800);
  stepper1.setCurrentPosition(goalToSteps(0, J1_DRV, J1_RED));
  stepper1.setPinsInverted(true, false, false);

  stepper2.setMaxSpeed(8000);
  stepper2.setAcceleration(800);
  stepper2.setCurrentPosition(goalToSteps(90, J2_DRV, J2_RED));
  stepper2.setPinsInverted(true, false, false);

  stepper3.setMaxSpeed(4108);
  stepper3.setAcceleration(800);
  stepper3.setCurrentPosition(goalToSteps(90, J3_DRV, J3_RED));
  stepper3.setPinsInverted(true, false, false);

  stepper4.setMaxSpeed(1000);
  stepper4.setAcceleration(800);
  stepper4.setCurrentPosition(goalToSteps(0, J4_DRV, J4_RED));

  stepper56_1.setMaxSpeed(1000);
  stepper56_1.setAcceleration(800);
  stepper56_1.setCurrentPosition(0);

  stepper56_2.setMaxSpeed(1000);
  stepper56_2.setAcceleration(800);
  stepper56_2.setCurrentPosition(0);

  updateFK();

  xTaskCreatePinnedToCore(serialTask, "Serial", 8192, NULL, 1, NULL, 0);
  sendStatus("ok", "ready");
}

// ─── Main loop ───
void loop() {
  if (!eStop) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
    stepper56_1.run();
    stepper56_2.run();
  }

  static unsigned long ledTimer = 0;
  if (millis() - ledTimer > 500) {
    digitalWrite(2, !digitalRead(2));
    ledTimer = millis();
  }
}

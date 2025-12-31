/*
 * VR Robot Arm Control - Teensy 4.1
 * 
 * Receives hand position data from Quest 2 via Serial (Web Serial API)
 * Maps VR coordinates to 3 ODrive motors via CAN
 * 
 * Serial Protocol:
 *   Position: "P:x,y,z\n"     - Set target position from VR hand
 *   Enable:   "E:1\n"         - Enable motors
 *   Disable:  "E:0\n"         - Disable motors  
 *   Stop:     "STOP\n"        - Emergency stop
 *   Home:     "HOME\n"        - Return to home position
 *   
 * Keyboard fallback (same as original):
 *   X = Enable/Disable, W/S = Motor3, A/D = Motor1, Q/E = Motor2
 *   SPACE = Emergency stop, 0 = Zero all, P = Print positions
 */

#include <Arduino.h>
#include "ODriveCAN.h"

#define CAN_BAUDRATE 250000

#define ODRV3_NODE_ID 3
#define ODRV1_NODE_ID 1
#define ODRV2_NODE_ID 2

#define IS_TEENSY_BUILTIN

#ifdef IS_TEENSY_BUILTIN
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}
#endif

// ODrive instances
ODriveCAN odrv3(wrap_can_intf(can_intf), ODRV3_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID);

ODriveCAN* odrives[] = {&odrv3, &odrv1, &odrv2};

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
  bool is_active = false;
};

ODriveUserData odrv3_user_data;
ODriveUserData odrv1_user_data;
ODriveUserData odrv2_user_data;

// ============ ARM GEOMETRY (inches) ============
const float BASE_HEIGHT = 5.5f;    // Length 1 - base to shoulder
const float UPPER_ARM = 16.0f;     // Length 2 - shoulder to elbow
const float FOREARM = 15.0f;       // Length 3 - elbow to tip

// NOTE: Gear ratios are now calibrated at runtime
// See calibration variables: base_ratio, shoulder_ratio, elbow_ratio

// VR to real-world scale (meters to inches)
const float VR_SCALE = 39.37f * 2.0f;  // 1m VR = ~80 inches real movement

// Motor position limits (turns) - SAFETY LIMITS
const float MOTOR_MIN = -25.0f;
const float MOTOR_MAX = 25.0f;

// Smoothing factor (0.0-1.0, lower = smoother but slower)
const float SMOOTHING = 0.15f;  // Faster response

// ============ CONTROL VARIABLES ============
// SAFETY: All positions start at 0 (current physical position)
// The arm will NOT move on startup - it stays exactly where it is
// Movement only happens after: 1) typing 'x' to enable, 2) typing 'v' for VR mode, 3) pulling trigger
float motor3_position = 0.0f;
float motor1_position = 0.0f;
float motor2_position = 0.0f;

float motor3_target = 0.0f;  // Target = current position (no movement)
float motor1_target = 0.0f;
float motor2_target = 0.0f;

bool vr_enabled = false;
bool safety_enabled = false;
const float STEP_SIZE = 0.5f;

// Calibration mode
bool calibrating = false;
float calibration_start = 0.0f;

// Gear ratio (turns per degree) - same for all motors
// Calibrated: 25.49 turns = 180 degrees
float motor_ratio = 0.141618f;  // turns per degree
unsigned long last_command_time = 0;
const unsigned long COMMAND_TIMEOUT = 60000;  // 60 second timeout (no rush)

// Serial input buffer
String serialBuffer = "";

// ============ CALLBACKS ============
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
  odrv_user_data->is_active = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}

// ============ INVERSE KINEMATICS ============
// Calculates joint angles for desired tip position
// Motor 1 (Q/E) = Base rotation
// Motor 2 (A/D) = Shoulder joint
// Motor 3 (W/S) = Elbow joint

bool calculateIK(float x, float y, float z, float &baseAngle, float &shoulderAngle, float &elbowAngle) {
  // x = left/right, y = up/down, z = forward/back
  // All units in inches
  
  // Base rotation (around Y axis)
  baseAngle = atan2(x, z) * 180.0f / PI;
  
  // Distance in horizontal plane from base
  float r = sqrt(x * x + z * z);
  
  // Height relative to shoulder joint
  float h = y - BASE_HEIGHT;
  
  // Distance from shoulder to target
  float d = sqrt(r * r + h * h);
  
  // Check if target is reachable
  float maxReach = UPPER_ARM + FOREARM - 0.5f;  // Small margin
  float minReach = abs(UPPER_ARM - FOREARM) + 0.5f;
  
  if (d > maxReach || d < minReach) {
    // Target out of reach - clamp to reachable distance
    if (d > maxReach) d = maxReach;
    if (d < minReach) d = minReach;
  }
  
  // Law of cosines for elbow angle
  float cosElbow = (UPPER_ARM * UPPER_ARM + FOREARM * FOREARM - d * d) / (2.0f * UPPER_ARM * FOREARM);
  cosElbow = constrain(cosElbow, -1.0f, 1.0f);
  elbowAngle = acos(cosElbow) * 180.0f / PI;
  
  // Convert to elbow bend (180 = straight, less = bent)
  elbowAngle = 180.0f - elbowAngle;
  
  // Shoulder angle
  float alpha = atan2(h, r);  // Angle to target from horizontal
  float cosGamma = (UPPER_ARM * UPPER_ARM + d * d - FOREARM * FOREARM) / (2.0f * UPPER_ARM * d);
  cosGamma = constrain(cosGamma, -1.0f, 1.0f);
  float gamma = acos(cosGamma);
  
  shoulderAngle = (alpha + gamma) * 180.0f / PI;
  
  return true;
}

// ============ VR POSITION PROCESSING ============
void processVrPosition(float deltaX, float deltaY, float deltaZ) {
  // Convert VR coordinates (meters) to arm coordinates (inches)
  // VR: x=right, y=up, z=back (towards user)
  // Arm: x=right, y=up, z=forward
  
  float targetX = deltaX * VR_SCALE;           // Left/right
  float targetY = deltaY * VR_SCALE + 20.0f;   // Up/down (offset to reasonable height)
  float targetZ = -deltaZ * VR_SCALE + 20.0f;  // Forward (flip Z, offset forward)
  
  float baseAngle, shoulderAngle, elbowAngle;
  
  if (calculateIK(targetX, targetY, targetZ, baseAngle, shoulderAngle, elbowAngle)) {
    // Convert angles to motor turns using calibrated ratios
    // Motor mapping (verified): Q/E=Base, A/D=Shoulder, W/S=Elbow
    // motor2_target -> ODrive 2 -> Q/E -> Base rotation
    // motor1_target -> ODrive 1 -> A/D -> Shoulder
    // motor3_target -> ODrive 3 -> W/S -> Elbow
    motor2_target = constrain(baseAngle * motor_ratio, MOTOR_MIN, MOTOR_MAX);      // Base (Q/E)
    motor1_target = constrain(shoulderAngle * motor_ratio, MOTOR_MIN, MOTOR_MAX);  // Shoulder (A/D)
    motor3_target = constrain(elbowAngle * motor_ratio, MOTOR_MIN, MOTOR_MAX);     // Elbow (W/S)
    
    // Debug output occasionally
    static int ikCount = 0;
    if (++ikCount % 100 == 0) {
      Serial.print("IK: target(");
      Serial.print(targetX, 1); Serial.print(", ");
      Serial.print(targetY, 1); Serial.print(", ");
      Serial.print(targetZ, 1); Serial.print(") -> angles(");
      Serial.print(baseAngle, 1); Serial.print(", ");
      Serial.print(shoulderAngle, 1); Serial.print(", ");
      Serial.print(elbowAngle, 1); Serial.println(")");
    }
  }
  
  last_command_time = millis();
}

// ============ SERIAL COMMAND PARSING ============
void processSerialCommand(String cmd) {
  cmd.trim();
  
  if (cmd.startsWith("P:")) {
    // Position command: "P:x,y,z"
    String coords = cmd.substring(2);
    int comma1 = coords.indexOf(',');
    int comma2 = coords.indexOf(',', comma1 + 1);
    
    if (comma1 > 0 && comma2 > comma1) {
      float x = coords.substring(0, comma1).toFloat();
      float y = coords.substring(comma1 + 1, comma2).toFloat();
      float z = coords.substring(comma2 + 1).toFloat();
      
      // Debug: show received position occasionally
      static int posCount = 0;
      if (++posCount % 50 == 0) {
        Serial.print("VR pos: ");
        Serial.print(x, 3); Serial.print(", ");
        Serial.print(y, 3); Serial.print(", ");
        Serial.println(z, 3);
      }
      
      // Auto-enable on first position command
      if (!safety_enabled) {
        safety_enabled = true;
        Serial.println(">>> AUTO-ENABLED <<<");
      }
      processVrPosition(x, y, z);
    }
  }
  else if (cmd.startsWith("E:")) {
    // Enable command: "E:1" or "E:0"
    int enable = cmd.substring(2).toInt();
    vr_enabled = (enable == 1);
    Serial.println(vr_enabled ? "VR_ENABLED" : "VR_DISABLED");
  }
  else if (cmd == "STOP") {
    emergencyStop();
  }
  else if (cmd == "HOME") {
    if (safety_enabled) {
      motor3_target = motor1_target = motor2_target = 0;
      motor3_position = motor1_position = motor2_position = 0;
      sendMotorPositions();
      Serial.println("HOMED");
    }
  }
  else if (cmd == "STATUS") {
    printStatus();
  }
  else if (cmd.startsWith("K:")) {
    // Keyboard command from web: "K:base,shoulder,elbow" (-1, 0, or 1 for each)
    last_command_time = millis();  // Reset timeout
    
    String params = cmd.substring(2);
    int comma1 = params.indexOf(',');
    int comma2 = params.indexOf(',', comma1 + 1);
    
    if (comma1 > 0 && comma2 > comma1) {
      int base = params.substring(0, comma1).toInt();
      int shoulder = params.substring(comma1 + 1, comma2).toInt();
      int elbow = params.substring(comma2 + 1).toInt();
      
      // Auto-enable on first keyboard command
      if (!safety_enabled) {
        safety_enabled = true;
        // IMPORTANT: Read current encoder positions from ODrives
        // This syncs software state with actual motor positions - NO JUMP
        if (odrv1_user_data.received_feedback) {
          motor1_position = motor1_target = odrv1_user_data.last_feedback.Pos_Estimate;
        }
        if (odrv2_user_data.received_feedback) {
          motor2_position = motor2_target = odrv2_user_data.last_feedback.Pos_Estimate;
        }
        if (odrv3_user_data.received_feedback) {
          motor3_position = motor3_target = odrv3_user_data.last_feedback.Pos_Estimate;
        }
        Serial.println(">>> AUTO-ENABLED (synced to current positions) <<<");
      }
      
      // Apply keyboard movement (same step size as local keyboard)
      motor2_target = constrain(motor2_target + base * STEP_SIZE, MOTOR_MIN, MOTOR_MAX);
      motor3_target = constrain(motor3_target + shoulder * STEP_SIZE, MOTOR_MIN, MOTOR_MAX);
      motor1_target = constrain(motor1_target + elbow * STEP_SIZE, MOTOR_MIN, MOTOR_MAX);
      
      // Debug occasionally
      static int kbCount = 0;
      if (++kbCount % 20 == 0) {
        Serial.print("KB: base="); Serial.print(base);
        Serial.print(" shoulder="); Serial.print(shoulder);
        Serial.print(" elbow="); Serial.println(elbow);
      }
    }
  }
}

void emergencyStop() {
  Serial.println("!!! EMERGENCY STOP !!!");
  
  // Stop in place - set target to current position (no movement)
  motor3_target = motor3_position;
  motor1_target = motor1_position;
  motor2_target = motor2_position;
  
  // Command motors to hold current position
  if (odrv3_user_data.is_active) odrv3.setPosition(motor3_position);
  if (odrv1_user_data.is_active) odrv1.setPosition(motor1_position);
  if (odrv2_user_data.is_active) odrv2.setPosition(motor2_position);
  
  safety_enabled = false;
  vr_enabled = false;
}

void sendMotorPositions() {
  if (odrv3_user_data.is_active) odrv3.setPosition(motor3_position);
  if (odrv1_user_data.is_active) odrv1.setPosition(motor1_position);
  if (odrv2_user_data.is_active) odrv2.setPosition(motor2_position);
}

void printStatus() {
  Serial.println("\n--- STATUS ---");
  Serial.print("Safety: "); Serial.println(safety_enabled ? "ON" : "OFF");
  Serial.print("VR Mode: "); Serial.println(vr_enabled ? "ON" : "OFF");
  Serial.print("Base (Q/E):     "); Serial.print(motor2_position, 2); 
  Serial.print(" -> "); Serial.println(motor2_target, 2);
  Serial.print("Shoulder (A/D): "); Serial.print(motor1_position, 2);
  Serial.print(" -> "); Serial.println(motor1_target, 2);
  Serial.print("Elbow (W/S):    "); Serial.print(motor3_position, 2);
  Serial.print(" -> "); Serial.println(motor3_target, 2);
  Serial.print("Motor ratio: "); Serial.print(motor_ratio, 6);
  Serial.println(" turns/deg");
  Serial.println("--------------\n");
}

void startCalibration() {
  calibrating = true;
  
  // Sync with actual encoder position first, then zero our counter
  // This way we track relative movement from here
  if (odrv3_user_data.received_feedback) {
    float actual_pos = odrv3_user_data.last_feedback.Pos_Estimate;
    motor3_target = actual_pos;
    motor3_position = actual_pos;
  }
  // Store the starting position for calibration
  calibration_start = motor3_position;
  
  Serial.println("\n=== CALIBRATING MOTOR RATIO ===");
  Serial.println("Using Elbow (W/S) for calibration");
  Serial.print("Starting position: "); Serial.println(calibration_start, 2);
  Serial.println("Move elbow to 180 degrees using W/S, then press C again");
}

void finishCalibration() {
  // Calculate turns moved since calibration started
  float turns = abs(motor3_position - calibration_start);
  motor_ratio = turns / 180.0f;
  
  Serial.print("\nResult: "); Serial.print(turns, 2);
  Serial.print(" turns = 180 deg\n");
  Serial.print("Motor ratio: "); Serial.print(motor_ratio, 6);
  Serial.println(" turns/degree");
  Serial.println("\n=== CALIBRATION COMPLETE ===");
  Serial.println("This ratio applies to all motors.\n");
  
  calibrating = false;
}

// ============ KEYBOARD CONTROL (FALLBACK) ============
void processKeyboard(char key) {
  last_command_time = millis();
  
  if (key == 'x' || key == 'X') {
    safety_enabled = !safety_enabled;
    Serial.println(safety_enabled ? "\n>>> ENABLED <<<\n" : "\n>>> DISABLED <<<\n");
    return;
  }
  
  if (key == 'v' || key == 'V') {
    vr_enabled = !vr_enabled;
    Serial.println(vr_enabled ? "\n>>> VR MODE ON <<<\n" : "\n>>> VR MODE OFF <<<\n");
    return;
  }
  
  if (key == ' ') {
    emergencyStop();
    return;
  }
  
  if (key == '0') {
    if (safety_enabled) {
      motor3_target = motor1_target = motor2_target = 0;
      motor3_position = motor1_position = motor2_position = 0;
      sendMotorPositions();
      Serial.println("Zeroed");
    }
    return;
  }
  
  if (key == 'p' || key == 'P') {
    printStatus();
    return;
  }
  
  // Calibration mode
  if (key == 'c' || key == 'C') {
    if (!safety_enabled) {
      Serial.println("Press X to enable first");
      return;
    }
    if (calibrating) {
      finishCalibration();
    } else {
      startCalibration();
    }
    return;
  }
  
  if (!safety_enabled) {
    if (strchr("wsadqeWSADQE", key)) Serial.println("Press X first");
    return;
  }
  
  // Keyboard movement (when not in VR mode)
  if (!vr_enabled) {
    bool moved = false;
    
    if ((key == 'W' || key == 'w') && odrv3_user_data.is_active) {
      motor3_target += STEP_SIZE;
      moved = true;
    }
    else if ((key == 'S' || key == 's') && odrv3_user_data.is_active) {
      motor3_target -= STEP_SIZE;
      moved = true;
    }
    else if ((key == 'D' || key == 'd') && odrv1_user_data.is_active) {
      motor1_target += STEP_SIZE;
      moved = true;
    }
    else if ((key == 'A' || key == 'a') && odrv1_user_data.is_active) {
      motor1_target -= STEP_SIZE;
      moved = true;
    }
    else if ((key == 'E' || key == 'e') && odrv2_user_data.is_active) {
      motor2_target += STEP_SIZE;
      moved = true;
    }
    else if ((key == 'Q' || key == 'q') && odrv2_user_data.is_active) {
      motor2_target -= STEP_SIZE;
      moved = true;
    }
    
    if (moved) {
      motor3_target = constrain(motor3_target, MOTOR_MIN, MOTOR_MAX);
      motor1_target = constrain(motor1_target, MOTOR_MIN, MOTOR_MAX);
      motor2_target = constrain(motor2_target, MOTOR_MIN, MOTOR_MAX);
    }
  }
}

// ============ SETUP ============
void setup() {
  Serial.begin(250000);  // Match Web Serial baud rate

  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);

  
  Serial.println("\n====================================");
  Serial.println("   VR ROBOT ARM CONTROL");
  Serial.println("====================================");
  Serial.println("KEYBOARD CONTROLS:");
  Serial.println("  X       = Enable/Disable motors");
  Serial.println("  V       = Toggle VR mode");
  Serial.println("  W / S   = Motor 3 +/-");
  Serial.println("  A / D   = Motor 1 -/+");
  Serial.println("  Q / E   = Motor 2 -/+");
  Serial.println("  SPACE   = Emergency stop");
  Serial.println("  0       = Zero all");
  Serial.println("  P       = Print status");
  Serial.println("");
  Serial.println("VR SERIAL COMMANDS:");
  Serial.println("  P:x,y,z = Position");
  Serial.println("  E:1/0   = Enable/Disable VR");
  Serial.println("  STOP    = Emergency stop");
  Serial.println("  HOME    = Return to home");
  Serial.println("====================================\n");

  // Setup ODrive callbacks
  odrv3.onFeedback(onFeedback, &odrv3_user_data);
  odrv3.onStatus(onHeartbeat, &odrv3_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);
  odrv2.onFeedback(onFeedback, &odrv2_user_data);
  odrv2.onStatus(onHeartbeat, &odrv2_user_data);

  if (!setupCan()) {
    Serial.println("CAN init failed!");
    while (true);
  }

  // Assume all ODrives are active (skip detection)
  Serial.println("Assuming all 3 ODrives are connected...");
  odrv1_user_data.is_active = true;
  odrv2_user_data.is_active = true;
  odrv3_user_data.is_active = true;
  
  // Brief pause to let CAN stabilize
  for (int i = 0; i < 20; i++) {
    pumpEvents(can_intf);
    delay(50);
  }

  // Set gains
  if (odrv3_user_data.is_active) {
    odrv3.setPosGain(20.0);
    odrv3.setVelGains(0.16, 0.32);
  }
  if (odrv1_user_data.is_active) {
    odrv1.setPosGain(20.0);
    odrv1.setVelGains(0.16, 0.32);
  }
  if (odrv2_user_data.is_active) {
    odrv2.setPosGain(20.0);
    odrv2.setVelGains(0.16, 0.32);
  }

  // Enable closed loop control
  Serial.println("Enabling closed loop control...");
  
  while (true) {
    bool all_ready = true;
    
    if (odrv3_user_data.is_active) {
      if (odrv3_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrv3.clearErrors();
        delay(1);
        odrv3.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        all_ready = false;
      }
    }
    
    if (odrv1_user_data.is_active) {
      if (odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrv1.clearErrors();
        delay(1);
        odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        all_ready = false;
      }
    }
    
    if (odrv2_user_data.is_active) {
      if (odrv2_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrv2.clearErrors();
        delay(1);
        odrv2.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        all_ready = false;
      }
    }

    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
    
    if (all_ready) break;
  }

  Serial.println("\n✓✓✓ READY ✓✓✓");
  Serial.println("Press 'X' to enable, 'V' for VR mode\n");
}

// ============ MAIN LOOP ============
void loop() {
  pumpEvents(can_intf);

  // Timeout check
  if (safety_enabled && (millis() - last_command_time > COMMAND_TIMEOUT)) {
    safety_enabled = false;
    vr_enabled = false;
    Serial.println("\n>>> TIMEOUT - DISABLED <<<\n");
  }

  // Read serial input
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else if (serialBuffer.length() < 64) {
      serialBuffer += c;
      
      // Also check for single-character keyboard commands
      // Note: Don't include P here - it conflicts with P:x,y,z position commands
      if (serialBuffer.length() == 1 && strchr("xXvVwWsSaAdDqQeE 0cC", c)) {
        processKeyboard(c);
        serialBuffer = "";
      }
    }
  }

  // Smooth interpolation towards target positions
  if (safety_enabled) {
    bool moved = false;
    
    if (abs(motor3_position - motor3_target) > 0.01f) {
      motor3_position += (motor3_target - motor3_position) * SMOOTHING;
      moved = true;
    }
    if (abs(motor1_position - motor1_target) > 0.01f) {
      motor1_position += (motor1_target - motor1_position) * SMOOTHING;
      moved = true;
    }
    if (abs(motor2_position - motor2_target) > 0.01f) {
      motor2_position += (motor2_target - motor2_position) * SMOOTHING;
      moved = true;
    }
    
    if (moved) {
      sendMotorPositions();
    }
  }
  
  delay(10);  // ~100Hz update rate
}

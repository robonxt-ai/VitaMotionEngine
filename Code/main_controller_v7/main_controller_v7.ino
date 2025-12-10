/*
 * VitaMotion Engine - Main Controller
 *
 * Created by robonxt
 * Assisted by AI
 * Last modified: 2025-12-09
 *
 * Changelog:
 * 2025-06-21: Initial commit
 * 2025-06-27: Integrated Python-style keyframe trot gait (by AI)
 * 2025-06-29: updated to use v0.03 of HiBusServo library
 * 2025-07-01: Cleaned up code
 * 2025-07-02: Cleaned up more code, change to command pattern
 * 2025-07-03: V4, added display and button support
 * 2025-09-23: V5, cleanup and prep for wireless, hid some warnings (BTSerial and TFT_eSPI)
 * 2025-12-06: V6, Geometric IK refactor with command/touch UI kept from V5
 * 2025-12-07: V7, Starting to work on a really nice gait! hopefully this works. cleaning up from last versions.
 * 2025-12-09: V7, Added PS4 controller support. Added WalkV6 gait. Cleaned up and made controller more fun. Tested on v0.0.4/v0.0.5 of HiBusServo library.
 */

#include <HiBusServo.h>
#include "SimpleIK.h"

// ---- Hardware Configuration Flags ----
#define ENABLE_DISPLAY true
#define ENABLE_BUTTONS true
#define ENABLE_PS4 true

#if ENABLE_PS4
#include "Ps4Input.h"
#endif

// Control Scheme Configuration
#define USE_UNITREE_CONTROLS true  // If true, uses L2 Modifier style. If false, uses L3 Toggle style.

#define MOVE_ONLY_WHEN_NEEDED true  // Set to true to move only when needed, false to always move
#define SERIAL_BAUDRATE 115199      // SERIAL = Serial communication

const float JOYSTICK_DEADZONE = 0.1f;

#define __UPLOADTIME__ __DATE__ " " __TIME__  // When was the program uploaded?
char *__SKETCHNAME__;                         // What is the name of the sketch?

//// For TFT stuff ////
#if ENABLE_DISPLAY
#include "robonxt_logos.h"
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI(135, 240);
#endif

//// For Button stuff ////
#if ENABLE_BUTTONS
#include <Button2.h>
#define BUTTON_PIN_1 35
#define BUTTON_PIN_2 0
#define LONG_PRESS_TIME 1000
Button2 button_1;
Button2 button_2;
#endif

// ---- State Machine ----
enum class ControllerState {
  STATE_IDLE,
  STATE_DISABLED,
  DEMO,
  TROT_ADS_V1,
  WALK_V4,  // Bezier Curve Gait (New V4)
  WALK_V5,  // Differential Bezier (V5)
  WALK_V6,  // Natural Geometric Gait (V6)
  SPIN_V2,  // Geometric spin V2
  JUMP_V1,
  JUMP_V2,
  STATE_MANIPULATION  // Body manipulation mode (Standing still)
};

// ---- Default Walking Mode ----
// Change this to easily swap the walking algorithm used by the joystick/buttons
const ControllerState DEFAULT_WALK_MODE = ControllerState::WALK_V6;

// Basic 2D point used by some motion modules (e.g., Bezier walk V4)
struct Point2D {
  float y;
  float z;
};

ControllerState state = ControllerState::STATE_IDLE;

// ---- Leg Struct Definition ----
struct Leg {
  int hip_id;
  int knee_id;
  int lengths[2];
  SimpleIK simple_ik;
  float hip_offset;
  float knee_offset;
  bool is_left;
  bool is_front;
  bool knee_bends_positive;

  float lastHipPos;
  float lastKneePos;

  Leg(int hip, int knee, int l1, int l2, float hipOff, float kneeOff, bool left, bool front)
    : hip_id(hip), knee_id(knee), lengths{ l1, l2 }, simple_ik(l1, l2), hip_offset(hipOff), knee_offset(kneeOff),
      is_left(left), is_front(front), knee_bends_positive(false), lastHipPos(0), lastKneePos(0) {
  }
};

// ---- Configuration ----
const int FL_HIP_ID = 1;
const int FL_KNEE_ID = 2;
const int FR_HIP_ID = 3;
const int FR_KNEE_ID = 4;
const int RL_HIP_ID = 5;
const int RL_KNEE_ID = 6;
const int RR_HIP_ID = 7;
const int RR_KNEE_ID = 8;

// Leg Indices in 'legs' array
const int LEG_FL = 0;
const int LEG_FR = 1;
const int LEG_RL = 2;
const int LEG_RR = 3;

const int THIGH_LENGTH = 100;
const int CALF_LENGTH = 100;
const float FOOT_RADIUS = 12.5f;  // Radius of the foot/wheel in mm

const float FL_HIP_OFFSET_LEFT = -0.48f;
const float FL_KNEE_OFFSET_LEFT = 41.76f;
const float FR_HIP_OFFSET_RIGHT = 1.44f;
const float FR_KNEE_OFFSET_RIGHT = -46.80f;
const float RL_HIP_OFFSET_LEFT = 0.96f;
const float RL_KNEE_OFFSET_LEFT = 42.48f;
const float RR_HIP_OFFSET_RIGHT = -0.24f;
const float RR_KNEE_OFFSET_RIGHT = -52.08f;

const float LEG_MIN_Y = -100.0f;  // Maximum backward movement (mm)
const float LEG_MAX_Y = 100.0f;   // Maximum forward movement (mm)
const float LEG_MIN_Z = 60.0f;    // Minimum height (mm)
const float LEG_MID_Z = 140.0f;   // Medium height for standing (mm)
const float LEG_MAX_Z = 180.0f;   // Maximum height (mm)

// Default movement parameters
const unsigned long DEFAULT_MOVE_TIME = 1000;
const unsigned long DEFAULT_PAUSE_TIME = DEFAULT_MOVE_TIME + 100;

// Global stance tuning (mm)
// GLOBAL_STANCE_SPREAD_Y moves front feet forward and rear feet backward to widen the support polygon
// Note: This spread is DYNAMICALLY SCALED by leg height to prevent kinematic binding at low heights.
float GLOBAL_STANCE_SPREAD_Y = 15.0f;         // Positive = Forward (Max Spread)
float GLOBAL_SPEED_MULTIPLIER = 1.0f;         // Runtime speed multiplier for walking/turning (1.0 = default). Adjusted via controller buttons/sliders.
float GLOBAL_COG_Y_OFFSET = -8.0f;            // Global COG compensation (mm) - Positive value moves the body backwards (COG back)
float SPREAD_THRESHOLD_Z = LEG_MAX_Z * 0.8f;  // Height above which full spread is applied

struct GaitTurnState {
  unsigned long start_time = 0;
  unsigned long last_update_time = 0;
  int direction = 1;  // 1 = right, -1 = left
};

GaitTurnState turn_in_place_state;

enum class JumpPhase {
  JUMP_START,
  JUMP_EXTEND,
  JUMP_RESET,
  JUMP_FINISH
};

struct JumpState {
  JumpPhase phase = JumpPhase::JUMP_START;
  unsigned long phase_start_time = 0;
};
JumpState jump_state;

HiBusServo servos(Serial2);

Leg legs[] = { { FL_HIP_ID, FL_KNEE_ID, THIGH_LENGTH, CALF_LENGTH, FL_HIP_OFFSET_LEFT, FL_KNEE_OFFSET_LEFT, true, true },
               { FR_HIP_ID, FR_KNEE_ID, THIGH_LENGTH, CALF_LENGTH, FR_HIP_OFFSET_RIGHT, FR_KNEE_OFFSET_RIGHT, false, true },
               { RL_HIP_ID, RL_KNEE_ID, THIGH_LENGTH, CALF_LENGTH, RL_HIP_OFFSET_LEFT, RL_KNEE_OFFSET_LEFT, true, false },
               { RR_HIP_ID, RR_KNEE_ID, THIGH_LENGTH, CALF_LENGTH, RR_HIP_OFFSET_RIGHT, RR_KNEE_OFFSET_RIGHT, false, false } };
const int NUM_LEGS = sizeof(legs) / sizeof(legs[0]);

/*  ------------------------------------------------------------------------------------------------------  */

// Telemetry Cache
float cached_logic_voltage = 0.0f;
float cached_servo_voltage = 0.0f;
unsigned long last_battery_update_time = 0;
const unsigned long BATTERY_UPDATE_INTERVAL = 5000;  // Update every 5 seconds

void updateBatteryStatus(bool force = false) {
  if (force || millis() - last_battery_update_time > BATTERY_UPDATE_INTERVAL) {
    // 1. Logic Battery (Pin 34)
    uint32_t raw_bat = analogRead(34);
    cached_logic_voltage = (raw_bat / 4095.0f) * 3.3f * 2.0f;

    // 2. Servo Battery (Ask Servo ID 1)
    int16_t servo_vin = servos.getVoltage(1);
    cached_servo_voltage = (servo_vin != -1) ? (servo_vin / 1000.0f) : 0.0f;
    delay(1);
    last_battery_update_time = millis();
  }
}

void drawBatteryStatus() {
#if ENABLE_DISPLAY
  // Clear area (width=55, height=26 for two lines)
  tft.fillRect(tft.width() - 55, tft.height() - 26, 55, 26, TFT_BLACK);

  tft.setTextSize(1);
  tft.setTextDatum(BR_DATUM);

  // Draw Servo Voltage (Top line of pair)
  tft.setTextColor(cached_servo_voltage > 6.0 ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString("S: " + String(cached_servo_voltage, 1) + "V", tft.width(), tft.height() - 13);

  // Draw Logic Voltage (Bottom line of pair)
  // Logic Voltage Colors: Blue > 4.0V (USB), Green > 3.2V (Battery), Red < 3.2V (Low)
  uint16_t logic_color = TFT_RED;
  if (cached_logic_voltage > 4.0) logic_color = TFT_BLUE;
  else if (cached_logic_voltage > 3.2) logic_color = TFT_GREEN;

  tft.setTextColor(logic_color, TFT_BLACK);
  tft.drawString("C: " + String(cached_logic_voltage, 1) + "V", tft.width(), tft.height());

  tft.setTextColor(TFT_WHITE, TFT_BLACK);  // Restore white
#endif
}

void tftMsg(String msg) {
#if ENABLE_DISPLAY
  // Clear the message area (y=70, height=20, width=240)
  // tft.fillRect(0, 70, 240, 20, TFT_BLACK); // Optional: Clear specific area if needed

  // Print Message
  tft.setTextColor(TFT_RED, TFT_BLACK, true);
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);  // Top-Left alignment for message
  tft.setTextWrap(true);
  tft.setCursor(0, 70);
  tft.print(F("                                                  "));  // Poor man's clear
  tft.setCursor(0, 70);
  tft.println(msg);
#endif
  Serial.println(msg);

  // Update Battery Voltages (Bottom Right)
  // Update battery status (non-blocking, interval based)
  updateBatteryStatus();
  drawBatteryStatus();
}

void aboutProgram() {
#if ENABLE_DISPLAY
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.pushImage(0, 5, 240, 64, robonxt_logo_thin);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextDatum(TR_DATUM);
  tft.setTextWrap(true);
  tft.setCursor(0, tft.height() - 25);
  tft.println(F("DEV: robonxt"));
  tft.println(F("VER: " __UPLOADTIME__));
  tft.print(F("PRG: "));
  tft.println(F(__SKETCHNAME__));

  tft.println(F(__SKETCHNAME__));

  drawBatteryStatus();
#endif

  for (int i = 0; i < 10; i++) {
    Serial.println();
  }
  Serial.println(F("DEV: robonxt"));
  Serial.println(F("VER: " __UPLOADTIME__));
  Serial.print(F("PRG: "));
  Serial.println(F(__SKETCHNAME__));
}

void displaySetup() {
#if ENABLE_DISPLAY
  delay(10);
  tft.init();
  delay(10);
  aboutProgram();
  delay(10);
#endif
}

// === Linear Interpolation (lerp) function ===
// Helper function to interpolate between two values (defined for non-ESP32 builds).
#ifndef ESP32
float lerp(float a, float b, float t) {
  return a + t * (b - a);
}
#endif

// Helper to scale spread based on height
// Theory: At low heights (crouch), wide spread forces unconnected kinematics.
// We fade spread to 0 as we approach MIN_Z.
float getStanceScale(float z) {
  if (z >= SPREAD_THRESHOLD_Z) return 1.0f;
  if (z <= LEG_MIN_Z) return 0.0f;
  return (z - LEG_MIN_Z) / (SPREAD_THRESHOLD_Z - LEG_MIN_Z);
}

bool sendLegCommands(Leg &leg, float hipAngle, float kneeAngle, unsigned long duration, bool commit) {
  // Map to Servos
  float servoHip, servoKnee;
  if (leg.is_left) {
    servoHip = -hipAngle + leg.hip_offset;
    servoKnee = kneeAngle + leg.knee_offset;
  } else {
    servoHip = hipAngle + leg.hip_offset;
    servoKnee = -kneeAngle + leg.knee_offset;
  }

  // Constrain
  servoHip = constrain(servoHip, -120.0f, 120.0f);
  servoKnee = constrain(servoKnee, -120.0f, 120.0f);

  // Filter Small Movements
  if (MOVE_ONLY_WHEN_NEEDED && abs(servoHip - leg.lastHipPos) < 1.0f && abs(servoKnee - leg.lastKneePos) < 1.0f) {
    return false;
  }

  if (commit) {
    servos.moveTo(leg.hip_id, servoHip, duration);
    servos.moveTo(leg.knee_id, servoKnee, duration);
  } else {
    servos.moveWaitDegrees(leg.hip_id, servoHip, duration);
    servos.moveWaitDegrees(leg.knee_id, servoKnee, duration);
  }

  leg.lastHipPos = servoHip;
  leg.lastKneePos = servoKnee;
  return true;
}

// Returns true if the servo was actually moved
// commit: If true, sends command immediately. If false, sends 'Wait' command (Sync Write queue).
bool moveLegIfNeeded(Leg &leg, float targetZ, float targetY, unsigned long duration = DEFAULT_MOVE_TIME, bool commit = true) {
  // Apply global CoG
  float local_Y = targetY + GLOBAL_COG_Y_OFFSET;

  // Apply Dynamic Stance Spread
  float spreadScale = getStanceScale(targetZ);
  float current_spread = GLOBAL_STANCE_SPREAD_Y * spreadScale;

  if (leg.is_front) local_Y += current_spread;
  else local_Y -= current_spread;

  // Hardware Direction Fix (V3 Logic): Positive Y = Forward
  float final_solver_Y = local_Y;

  // Account for Foot Radius
  // The IK Solver assumes link lengths touch ground at Z.
  // Physical leg has an additional FOOT_RADIUS height.
  // We subtract this radius so the axle is placed higher, making the total distance (axle + radius) equal directly to targetZ.
  float final_solver_Z = targetZ - FOOT_RADIUS;

  // Solve SimpleIK
  float hipAngle, kneeAngle;
  bool solved = leg.simple_ik.solve(final_solver_Y, final_solver_Z, leg.knee_bends_positive, hipAngle, kneeAngle);
  if (!solved) return false;

  return sendLegCommands(leg, hipAngle, kneeAngle, duration, commit);
}

// Move all legs to specified position (z = height, y = forward/back)
void moveAllLegs(float targetZ, float targetY, unsigned long duration = DEFAULT_MOVE_TIME) {
  for (int i = 0; i < NUM_LEGS; i++) {
    moveLegIfNeeded(legs[i], targetZ, targetY, duration, false);  // Commit = false
  }
  servos.startMove();
}

// ===================================================================================
// Geometric IK helper used by geometric behaviors (e.g., SpinV2, JumpV2)
// ===================================================================================

// Wrapper for SimpleIK that handles coordinate mapping and servo movement
bool moveLegGeometric(Leg &leg, float targetZ, float targetY, unsigned long duration = DEFAULT_MOVE_TIME) {
  // 1. MAPPING M: GLOBAL MAPPINGS to LOCAL LEG FRAME
  // Global Convention: Positive Y = Forward
  // Geometric Solver:  Positive Y = Forward (Distance away from hip)

  // Apply Global Offsets
  float local_Y = targetY + GLOBAL_COG_Y_OFFSET;

  // Apply Stance Logic (Front vs Rear)
  // We want Positive Y to mean "Forward relative to the robot"
  // For Front Legs: Forward is +Y.
  // For Rear Legs:  Forward is +Y.
  // However, the 'stance spread' pushes legs APART.
  // Front Legs push Forward (+Spread).
  // Rear Legs push Backward (-Spread).
  if (leg.is_front) {
    local_Y += GLOBAL_STANCE_SPREAD_Y;
  } else {
    local_Y -= GLOBAL_STANCE_SPREAD_Y;
  }

  // HARDWARE FIX:
  // If your particular servos/legs require inverted Inputs generally, do it here.
  // Based on user feedback: "walking BACKWARDS".
  // So we assume Positive Local Y = Forward physical request.
  float final_solver_Y = local_Y;

  // 2. SOLVE IK
  float hipAngle, kneeAngle;
  bool solved = leg.simple_ik.solve(final_solver_Y, targetZ, leg.knee_bends_positive, hipAngle, kneeAngle);

  if (!solved) return false;

  // 3. MAP TO SERVO COMMANDS
  return sendLegCommands(leg, hipAngle, kneeAngle, duration, true);
}

void moveAllLegsGeometric(float targetZ, float targetY, unsigned long duration = DEFAULT_MOVE_TIME) {
  for (int i = 0; i < NUM_LEGS; i++) {
    moveLegGeometric(legs[i], targetZ, targetY, duration);
  }
}

// Helper to parse space-separated float arguments
int parseSpaceSeparatedFloats(const String &args, float *out, int maxCount) {
  int valIdx = 0;
  int start = 0;
  int len = args.length();

  while (start < len && valIdx < maxCount) {
    int spaceIdx = args.indexOf(' ', start);
    if (spaceIdx == -1) spaceIdx = len;

    String token = args.substring(start, spaceIdx);
    token.trim();
    if (token.length() > 0) {
      out[valIdx] = token.toFloat();
      valIdx++;
    }
    start = spaceIdx + 1;
  }
  return valIdx;
}

// ---- Command Parser ----
String serial_buffer;

// Helper to convert ControllerState to string
const char *controllerStateToString(ControllerState s) {
  switch (s) {
    case ControllerState::STATE_IDLE:
      return "IDLE";
    case ControllerState::DEMO:
      return "DEMO";
    case ControllerState::TROT_ADS_V1:
      return "TROT_ADS_V1";
    case ControllerState::WALK_V4:
      return "WALK_V4 (Bezier)";
    case ControllerState::WALK_V5:
      return "WALK_V5 (Joy)";
    case ControllerState::WALK_V6:
      return "WALK_V6 (Natural)";
    case ControllerState::SPIN_V2:
      return "SPIN_V2 (Geometric)";
    case ControllerState::JUMP_V1:
      return "JUMP_V1";
    case ControllerState::JUMP_V2:
      return "JUMP_V2 (Geometric)";
    case ControllerState::STATE_DISABLED:
      return "DISABLED";
    default:
      return "UNKNOWN";
  }
}

// Command handler function type
typedef void (*CommandHandler)(const String &);

// Command structure
struct Command {
  const char *name;
  CommandHandler handler;
  bool requiresArgs;
};

// Forward declarations for external commands
void handleWalkV4(const String &args);
void updateWalkV4();
void handleWalkV5(const String &args);
void updateWalkV5();
void handleWalkV6(const String &args);
void updateWalkV6();
void handleSpinV2(const String &args);
void handleJumpV1(const String &args);
void handleJumpV2(const String &args);
void handleTrotAdsV1(const String &args);
void handleDemo(const String &args);

// External Motion Updates
void updateDemo();
void updateJumpV1();
void updateJumpV2();
void updateSpinV2();
void updateTrotAdsV1();
void updateBodyManipulation();  // Idle Logic

// External Enter Modes
void enterWalkV4Mode(float speed);
void enterWalkV5Mode();
void enterWalkV6Mode();  // Added for V6

// Internal Helper Forward Declarations
void enterDemoMode();
void enterTrotAdsV1Mode();
void enterSpinV2Mode(int dir);
void enterIdleMode();
void disableAllServos();
void enableAllServos();
void showHelp();
void processPs4Buttons();

// Command handlers
void handleStop(const String &) {
  enterIdleMode();
}
void handleDisable(const String &) {
  disableAllServos();
}
void handleEnable(const String &) {
  enableAllServos();
}
void handleHelp(const String &) {
  showHelp();
}
void handleDown(const String &) {
  Serial.println("Laying down...");
  moveAllLegs(LEG_MIN_Z, 0);
}
void handleUp(const String &) {
  Serial.println("Getting up...");
  moveAllLegs(LEG_MID_Z, 0);
}

// Rock body in place by moving feet in +Y/-Y while keeping height at LEG_MID_Z.
// Negative Y moves feet backward (body forward), positive Y moves feet forward (body back).
void rockBody(float targetY) {
  if (state != ControllerState::STATE_IDLE) return;
  moveAllLegs(LEG_MID_Z, targetY, 500);
}

void handleForward(const String &) {
  rockBody(-30.0f);
}

void handleBack(const String &) {
  rockBody(30.0f);
}

void enterIdleMode() {
  tftMsg("[Controller] Entering STATE_IDLE mode. Halting all movement.");
  servos.broadcastStop();
  state = ControllerState::STATE_IDLE;
}

void disableAllServos() {
  tftMsg("[Controller] Disabling all servos");
  servos.motorOff(BROADCAST_ID);
  state = ControllerState::STATE_DISABLED;
}

void enableAllServos() {
  tftMsg("[Controller] Enabling all servos");

  servos.motorOn(BROADCAST_ID);
  delay(10);  // Wait for servos to wake

  // Update internal state with actual positions to avoid jumps
  // This ensures the next move command starts from where the servos ARE, not where we thought they were.
  for (int i = 0; i < NUM_LEGS; ++i) {
    int16_t h = servos.getPosition(legs[i].hip_id);
    int16_t k = servos.getPosition(legs[i].knee_id);
    if (h != -1) legs[i].lastHipPos = (float)h;
    if (k != -1) legs[i].lastKneePos = (float)k;
  }

  state = ControllerState::STATE_IDLE;
  softStartServos();
}

void softStartServos() {
  tftMsg("[Controller] Soft-starting servos...");
  moveAllLegs(LEG_MID_Z, 0, 2000);  // Take 2 seconds to stand up
}

#if ENABLE_BUTTONS
void handler(Button2 &btn) {
  if (btn == button_1) {
    if (state == ControllerState::STATE_IDLE) {
      enterWalkV4Mode(1.0f);
    } else {
      enterIdleMode();
    }
  } else if (btn == button_2) {
    // Toggle SPIN_V2 direction: if not spinning, start right; if spinning,  } else if (btn == button_2) {
    if (state != ControllerState::SPIN_V2) {
      enterSpinV2Mode(1);  // Turn Right
    } else {
      enterSpinV2Mode(turn_in_place_state.direction * -1);  // Toggle Direction
    }
  } else {
    ;  // this is a catch all just in case
  }
}

void longhandler(Button2 &btn) {
  unsigned int timePressed = btn.wasPressedFor();
  if (btn == button_1 && timePressed > LONG_PRESS_TIME) {
    disableAllServos();
  } else if (btn == button_2 && timePressed > LONG_PRESS_TIME) {
    enableAllServos();
  } else {
    ;  // this is a catch all just in case
  }
}

void doublehandler(Button2 &btn) {
  if (btn == button_1) {
    enterDemoMode();
  } else if (btn == button_2) {
    enterTrotAdsV1Mode();
  } else {
    ;  // this is a catch all just in case
  }
}
#endif

void showHelp() {
  Serial.println("\n=== VitaMotion Engine - Command Reference ===\n");
  Serial.println("Basic Commands:");
  Serial.println("\thelp, ?    - Display this help message");
  Serial.println("\tstatus     - Show state, servo positions, voltages");
  Serial.println("\tstop       - Stop all current movements");
  Serial.println("\tdisable    - Disable all servos");
  Serial.println("\tenable     - Enable all servos\n");

  Serial.println("Movement Commands:");
  Serial.println("\tdemo         - Enter demo mode");
  Serial.println("\tup           - Stand up (medium height)");
  Serial.println("\tdown         - Lay down (min height)");
  Serial.println("\tforward      - Rock forward");
  Serial.println("\tback         - Rock backward");
  Serial.println("\tads          - Alternating diagonal step trot");
  Serial.println("\twalkv4 [f] [h] [s] [l]  - Bezier trot: speed factor f (-1..1), base height h, stride s, lift l");
  Serial.println("\twalkv5 <y> <x> [h] [l] [s] - Diff-drive trot V5 (Stability): y, x, height, lift, stride");
  Serial.println("\twalkv6 <y> <x> [h] [l] [s] - Diff-drive trot V6 (Natural): y, x, height, lift, stride");
  Serial.println("\tspin [left|right]       - Spin in place (Geometric). Default: right");
  Serial.println("\tjump                    - Perform jump V1 sequence");
  Serial.println("\tjumpv2                  - Perform jump V2 (Geometric)\n");

  Serial.println("Button Shortcuts:");
  Serial.println("\tBTN1 click       - Toggle WALK V4 (Bezier trot) on/off");
  Serial.println("\tBTN1 doubleclick - Enter DEMO mode");
  Serial.println("\tBTN1 longpress   - Disable all servos");
  Serial.println("\tBTN2 click       - Toggle SPIN V2 (reverse direction if already spinning)");
  Serial.println("\tBTN2 doubleclick - Enter ADS trot mode");
  Serial.println("\tBTN2 longpress   - Enable all servos\n");

  Serial.println("================================================\n");
}

// Command handlers

// Command handlers moved to their respective modules:
// handleDemo -> Demo_motion.ino
// handleTrotAdsV1 -> ADS_motion.ino
// handleJumpV1/V2 -> Jump_motion.ino
// handleSpinV1/V2 -> Spin_motion.ino

// Status command remains here as it accesses global state directly
void handleStatus(const String &) {
  Serial.print("[Status] Mode: ");
  Serial.println(controllerStateToString(state));

  // T-Display Battery Voltage Reading (Pin 34 is typical for TTGO T-Display)
  // Formula: Vbat = (ADC / 4095) * 3.3V * 2 (Voltage divider 100k/100k) + calibration
  updateBatteryStatus(true);
  Serial.print("Controller Battery (T-Display): ");
  Serial.print(cached_logic_voltage, 2);
  Serial.println(" V");

  Serial.println("Servo positions and voltages:");
  for (int id = 1; id <= 8; ++id) {
    int16_t pos_raw = servos.getPosition(id);
    int16_t vin_raw = servos.getVoltage(id);
    int8_t temp = servos.getTemperature(id);
    bool is_on = servos.isMotorOn(id);
    float vinV = vin_raw / 1000.0f;

    Serial.print("  Servo ");
    Serial.print(id);
    Serial.print(": ");
    Serial.print(is_on ? "ENABLED " : "DISABLED");
    Serial.print(", pos=");
    Serial.print(pos_raw != -1 ? String(pos_raw) : "ERROR");
    Serial.print(", vin=");
    Serial.print(vin_raw != -1 ? String(vinV, 2) : "ERROR");
    Serial.print(", temp=");
    Serial.print(temp != -1 ? String(temp) : "ERROR");
    Serial.println();
  }
}

const Command commands[] = {
  { "demo", handleDemo, false },
  { "stop", handleStop, false },
  { "disable", handleDisable, false },
  { "enable", handleEnable, false },
  { "help", handleHelp, false },
  { "?", handleHelp, false },
  { "down", handleDown, false },
  { "up", handleUp, false },
  { "forward", handleForward, false },
  { "back", handleBack, false },
  { "ads", handleTrotAdsV1, false },
  { "jump", handleJumpV1, false },
  { "walkv4", handleWalkV4, true },
  { "walkv5", handleWalkV5, true },
  { "walkv6", handleWalkV6, true },
  { "spin", handleSpinV2, true },  // Spin command now uses V2 (Geometric) by default
  { "jumpv2", handleJumpV2, false },
  { "status", handleStatus, false }
};

const int NUM_COMMANDS = sizeof(commands) / sizeof(Command);

void parseCommand(const String &cmd) {
  String command = cmd;
  command.trim();
  command.toLowerCase();
  Serial.println("Received: " + command);

  // Split command and arguments
  int space_pos = command.indexOf(' ');
  String cmd_name = space_pos == -1 ? command : command.substring(0, space_pos);
  String args = space_pos == -1 ? "" : command.substring(space_pos + 1);

  // Find and execute command
  bool found = false;
  for (int i = 0; i < NUM_COMMANDS; i++) {
    if (cmd_name == commands[i].name) {
      found = true;
      if (commands[i].requiresArgs && args.length() == 0) {
        Serial.print("Usage: ");
        Serial.print(commands[i].name);
        Serial.println(" <args>");
      } else {
        commands[i].handler(args);
      }
      break;
    }
  }

  if (!found) {
    Serial.println("Unknown command: " + command);
    Serial.println("Type 'help' for available commands.");
  }
}

// Buttons
void buttonSetup() {
#if ENABLE_BUTTONS
  button_1.begin(BUTTON_PIN_1, INPUT);
  button_1.setClickHandler(handler);
  button_1.setLongClickHandler(longhandler);
  button_1.setDoubleClickHandler(doublehandler);

  button_2.begin(BUTTON_PIN_2);
  button_2.setClickHandler(handler);
  button_2.setLongClickHandler(longhandler);
  button_2.setDoubleClickHandler(doublehandler);
#endif
}

void buttonLoop() {
#if ENABLE_BUTTONS
  button_1.loop();
  button_2.loop();
#endif
}


// PS4 Buttons
static bool ps4_prevCross = false;
static bool ps4_prevCircle = false;
static bool ps4_prevTriangle = false;
static bool ps4_prevSquare = false;
static bool ps4_prevL3 = false;
static bool ps4_poseIsUp = false;

// Feedback State
unsigned long error_vibrate_until = 0;

void triggerErrorFeedback() {
  error_vibrate_until = millis() + 300;  // Vibrate for 300ms
}

static bool ps4HasActiveMotionInput() {
#if ENABLE_PS4
  if (!g_ps4Input.connected) return false;

  float magWalk = g_ps4Input.walkX * g_ps4Input.walkX + g_ps4Input.walkY * g_ps4Input.walkY;
  if (magWalk > (JOYSTICK_DEADZONE * JOYSTICK_DEADZONE)) return true;

  if (abs(g_ps4Input.lxRaw) > 10 || abs(g_ps4Input.lyRaw) > 10 || abs(g_ps4Input.rxRaw) > 10 || abs(g_ps4Input.ryRaw) > 10) {
    return true;
  }

  if (g_ps4Input.l2Raw > 10 || g_ps4Input.r2Raw > 10) return true;

  if (g_ps4Input.dpadUp || g_ps4Input.dpadDown || g_ps4Input.dpadLeft || g_ps4Input.dpadRight) return true;

  return false;
#else
  return false;
#endif
}

void updateControllerFeedback() {
#if ENABLE_PS4
  if (!g_ps4Input.connected) return;

  // Throttle updates to prevent crashing the Bluetooth stack (PS4_L2CAP buffer overflow)
  static unsigned long last_feedback_time = 0;
  if (millis() - last_feedback_time < 200) return;  // limit to 5Hz
  last_feedback_time = millis();

  uint8_t r = 0, g = 0, b = 0;
  uint8_t smallRumble = 0, largeRumble = 0;

  // Handle Vibration
  if (millis() < error_vibrate_until) {
    smallRumble = 255;
    largeRumble = 255;
  }

  // Handle Colors based on State
  if (state == ControllerState::STATE_DISABLED) {
    // Flashing Red
    if ((millis() / 500) % 2 == 0) {
      r = 255;
    }
  } else if (state == ControllerState::STATE_IDLE || state == ControllerState::STATE_MANIPULATION) {
    // Blue
    b = 255;
  } else if (state == ControllerState::WALK_V4 || state == ControllerState::WALK_V5 || state == ControllerState::WALK_V6) {
    // Green
    g = 255;
  } else {
    // Other (Yellow)
    r = 255;
    g = 255;
  }

  setControllerFeedback(r, g, b, smallRumble, largeRumble);
#endif
}

void processPs4Buttons() {
#if ENABLE_PS4
  if (!g_ps4Input.connected) {
    ps4_prevCross = false;
    ps4_prevCircle = false;
    ps4_prevTriangle = false;
    ps4_prevSquare = false;
    return;
  }

  bool cross = g_ps4Input.cross;
  bool circle = g_ps4Input.circle;
  bool triangle = g_ps4Input.triangle;
  bool square = g_ps4Input.square;
  bool l3 = g_ps4Input.l3;

  bool crossEdge = cross && !ps4_prevCross;
  bool circleEdge = circle && !ps4_prevCircle;
  bool triangleEdge = triangle && !ps4_prevTriangle;
  bool squareEdge = square && !ps4_prevSquare;
  bool l3Edge = l3 && !ps4_prevL3;

  ps4_prevCross = cross;
  ps4_prevCircle = circle;
  ps4_prevTriangle = triangle;
  ps4_prevSquare = square;
  ps4_prevL3 = l3;

  if (triangleEdge) {
    disableAllServos();
    return;
  }

  if (squareEdge) {
    enableAllServos();
  }

  if (circleEdge) {
    if (state != ControllerState::STATE_DISABLED) {
      enterIdleMode();
      if (ps4_poseIsUp) {
        moveAllLegs(LEG_MIN_Z, 0);
      } else {
        moveAllLegs(LEG_MID_Z, 0);
      }
      ps4_poseIsUp = !ps4_poseIsUp;
    }
  }

  if (crossEdge) {
    if (state == ControllerState::STATE_DISABLED) {
      triggerErrorFeedback();
      return;
    }
    if (state == DEFAULT_WALK_MODE) {
      enterIdleMode();
    } else {
      // Enter the default walk mode
      switch (DEFAULT_WALK_MODE) {
        case ControllerState::WALK_V4: enterWalkV4Mode(1.0f); break;
        case ControllerState::WALK_V5: enterWalkV5Mode(); break;
        case ControllerState::WALK_V6: enterWalkV6Mode(); break;
        default: break;
      }
    }
  }

  // L3 Button: Toggle between IDLE and BODY MANIPULATION
  if (l3Edge) {
    if (state == ControllerState::STATE_IDLE) {
      tftMsg("Mode: Body Manipulation");
      state = ControllerState::STATE_MANIPULATION;
    } else if (state == ControllerState::STATE_MANIPULATION) {
      enterIdleMode();
    }
  }
#endif
}

// ---- Setup ----
void setup() {
  // Initialize Debug Serial

  delay(100);
  Serial.begin(SERIAL_BAUDRATE);
  delay(100);

  // Set up SKETCHNAME!
  char a[] = __FILE__;
  byte b = sizeof(a);
  while ((b > 0) && (a[b] != '\\')) {
    b--;
  }
  __SKETCHNAME__ = &a[++b];
  ////////////////////////

  delay(200);

  // Initial battery update
  updateBatteryStatus(true);

  // tftMsg("\n=== Main Controller ===");
  showHelp();
  displaySetup();
  buttonSetup();
#if ENABLE_PS4
  ps4Setup();
#endif


  // Initialize Servo Serial (Serial2)
#ifdef ESP32
  Serial2.begin(SERIAL_BAUDRATE, SERIAL_8N1, 32, 33);
#else
  // For other boards (Generic Serial2 usage)
  Serial2.begin(SERIAL_BAUDRATE);
#endif


  enterIdleMode();
}

// ---- Main Loop ----
void loop() {
  // Handle button inputs
  buttonLoop();

#if ENABLE_PS4
  ps4Loop();
#endif

  processPs4Buttons();

  // Check for serial input
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serial_buffer.length() > 0) {
        parseCommand(serial_buffer);
        serial_buffer = "";
      }
    } else if (isPrintable(c)) {
      serial_buffer += c;
    }
  }
  if (state == ControllerState::STATE_DISABLED && ps4HasActiveMotionInput()) {
    static unsigned long last_disabled_input_feedback = 0;
    unsigned long now = millis();
    if (now - last_disabled_input_feedback > 400) {
      triggerErrorFeedback();
      last_disabled_input_feedback = now;
    }
  }
  // --- UNIVERSAL UNITREE CONTROL OVERRIDE ---
#ifdef USE_UNITREE_CONTROLS
  // Check if we are in a motion state that supports override (Walks, Idle)
  bool canOverride = (state == ControllerState::WALK_V4 || state == ControllerState::WALK_V5 || state == ControllerState::WALK_V6 || state == ControllerState::STATE_IDLE);

  if (canOverride && g_ps4Input.connected && g_ps4Input.l1) {
    updateBodyManipulation();
    return;  // Skip standard state processing
  }
#endif

  switch (state) {
    case ControllerState::STATE_DISABLED:
      // Serial2.flush()
    case ControllerState::STATE_IDLE:
      // Passive
      break;
    case ControllerState::STATE_MANIPULATION:
      updateBodyManipulation();
      break;
    case ControllerState::DEMO:
      updateDemo();
      break;
    case ControllerState::JUMP_V1:
      updateJumpV1();
      break;
    case ControllerState::WALK_V4:
      updateWalkV4();
      break;
    case ControllerState::WALK_V5:
      updateWalkV5();
      break;
    case ControllerState::WALK_V6:
      updateWalkV6();
      break;
    case ControllerState::SPIN_V2:
      updateSpinV2();
      break;
    case ControllerState::TROT_ADS_V1:
      updateTrotAdsV1();
      break;
    case ControllerState::JUMP_V2:
      updateJumpV2();
      break;
  }

  updateControllerFeedback();
}

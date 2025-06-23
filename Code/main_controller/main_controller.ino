/*
 * VitaMotion Engine - Main Controller
 * 
 * Created by robonxt
 * Assisted by AI
 * Date: 2025-06-21
 * 
 * Changelog:
 * 2025-06-21: Initial commit
 */


#include <HiBusServo.h>
#include <FABRIK2D.h>
#include <Adafruit_NeoPixel.h>

// ---- RGB LED Configuration ----
#define LED_PIN 23
#define BRIGHTNESS 50
const uint8_t LED_TARGET_FPS = 15;                           // Default FPS for all LED animations
const uint16_t MS_PER_FRAME = 1000 / LED_TARGET_FPS;         // Frame duration in milliseconds
Adafruit_NeoPixel pixels(1, LED_PIN, NEO_GRB + NEO_KHZ800);  // Single LED

// LED Colors (as HSV for smoother transitions)
const uint32_t COLOR_IDLE = pixels.ColorHSV(43690, 255, 100);        // Blue
const uint32_t COLOR_MOVING = pixels.ColorHSV(21845, 255, 200);      // Green
const uint32_t COLOR_WALKING = pixels.ColorHSV(18204, 255, 200);     // Light Green
const uint32_t COLOR_CALIBRATING = pixels.ColorHSV(5461, 255, 200);  // Orange
const uint32_t COLOR_ERROR = pixels.ColorHSV(0, 255, 200);           // Red
const uint32_t COLOR_RAINBOW[] = {
  pixels.Color(255, 0, 0),    // Red
  pixels.Color(255, 127, 0),  // Orange
  pixels.Color(255, 255, 0),  // Yellow
  pixels.Color(0, 255, 0),    // Green
  pixels.Color(0, 0, 255),    // Blue
  pixels.Color(75, 0, 130),   // Indigo
  pixels.Color(148, 0, 211)   // Violet
};

// Animation state
struct AnimationState {
  unsigned long lastLEDUpdate = 0;
  unsigned long lastBreathUpdate = 0;
  int rainbowHue = 0;
  int breathBrightness = 20;  // Start at minimum brightness
  int breathDirection = 10;   // Increase step for smoother breathing
  bool blinkState = false;
  unsigned long lastStateChange = 0;
};

AnimationState animState;

// ---- State Machine ----
enum class ControllerState { IDLE,
                             MOVING,
                             WALKING,
                             CALIBRATING,
                             DISABLED };
ControllerState state = ControllerState::IDLE;

// ---- Leg Struct Definition ----
struct Leg {
  uint8_t hip_id;
  uint8_t knee_id;
  int lengths[2];
  Fabrik2D fabrik;
  float hip_offset;
  float knee_offset;
  bool is_left;

  Leg(uint8_t hip, uint8_t knee, int l1, int l2, float hipOff, float kneeOff, bool left)
    : hip_id(hip), knee_id(knee), lengths{ l1, l2 }, fabrik(3, lengths), hip_offset(hipOff), knee_offset(kneeOff), is_left(left) {}
};

// ---- Configuration ----
const int MOVE_TIME = 1000;
const int PAUSE_TIME = MOVE_TIME + 10;

const int FL_HIP_ID = 1;
const int FL_KNEE_ID = 2;
const int FR_HIP_ID = 3;
const int FR_KNEE_ID = 4;
const int RL_HIP_ID = 5;
const int RL_KNEE_ID = 6;
const int RR_HIP_ID = 7;
const int RR_KNEE_ID = 8;

const int THIGH_LENGTH = 100;
const int CALF_LENGTH = 100;

const float FL_HIP_OFFSET_LEFT = 0.24f;      // From Front-Left Hip ID:1
const float FL_KNEE_OFFSET_LEFT = 41.04f;    // From Front-Left Knee ID:2
const float FR_HIP_OFFSET_RIGHT = 0.48f;     // From Front-Right Hip ID:3 (positive for right side)
const float FR_KNEE_OFFSET_RIGHT = -45.84f;  // From Front-Right Knee ID:4
const float RL_HIP_OFFSET_LEFT = 0.0f;       // From Rear-Left Hip ID:5 (0.00 from calibration)
const float RL_KNEE_OFFSET_LEFT = 37.92f;    // From Rear-Left Knee ID:6
const float RR_HIP_OFFSET_RIGHT = 0.0f;      // From Rear-Right Hip ID:7 (0.00 from calibration)
const float RR_KNEE_OFFSET_RIGHT = -52.32f;  // From Rear-Right Knee ID:8

// Position limits (mm)
const float MIN_Z = 55.0f;   // Minimum height (mm)
const float MAX_Z = 180.0f;  // Maximum height (mm)
const float MED_Z = 140.0f;  // Medium height for standing (mm)
const float MIN_Y = -50.0f;  // Maximum backward movement (mm)
const float MAX_Y = 50.0f;   // Maximum forward movement (mm)

// Gait parameters
const int DEFAULT_STEPS = 10;             // Default number of steps for walking
const float STRIDE_LENGTH = 60.0f;        // How far forward/backward to step (mm)
const float STANDING_HEIGHT = 170.0f;     // Standing height (mm) - lower is higher
const float LIFT_HEIGHT = 40.0f;          // How high to lift leg (mm)
const float STEP_BACK_DISTANCE = 30.0f;   // How far back to push off (mm)
const unsigned long MOVE_DURATION = 400;  // Time for each movement phase (ms)
const float MIN_ANGLE_CHANGE = 5.0f;      // Minimum angle change (degrees) before moving servo

// Walking state
enum class WalkPhase {
  LIFT_SWING_LEG,  // Lift and swing one diagonal pair
  PLANT_PUSH_LEG,  // Plant and push with the other diagonal pair
  COMPLETE         // Walking complete
};

struct LegState {
  float target_z = STANDING_HEIGHT;
  float target_y = 0;
  float current_z = STANDING_HEIGHT;
  float current_y = 0;
};

struct WalkingState {
  bool is_walking = false;
  int total_steps = 0;
  int steps_taken = 0;
  unsigned long phase_start_time = 0;
  WalkPhase phase = WalkPhase::LIFT_SWING_LEG;
  bool diagonal = false;  // false = FL+RR, true = FR+RL
  LegState legs[4];       // One state per leg
} walk_state;

HiBusServo servos;

Leg legs[] = {
  { FL_HIP_ID, FL_KNEE_ID, THIGH_LENGTH, CALF_LENGTH, FL_HIP_OFFSET_LEFT, FL_KNEE_OFFSET_LEFT, true },
  { FR_HIP_ID, FR_KNEE_ID, THIGH_LENGTH, CALF_LENGTH, FR_HIP_OFFSET_RIGHT, FR_KNEE_OFFSET_RIGHT, false },
  { RL_HIP_ID, RL_KNEE_ID, THIGH_LENGTH, CALF_LENGTH, RL_HIP_OFFSET_LEFT, RL_KNEE_OFFSET_LEFT, true },
  { RR_HIP_ID, RR_KNEE_ID, THIGH_LENGTH, CALF_LENGTH, RR_HIP_OFFSET_RIGHT, RR_KNEE_OFFSET_RIGHT, false }
};
const int NUM_LEGS = sizeof(legs) / sizeof(legs[0]);

// ---- Helper: Move One Leg ----
// Returns true if the servo was actually moved
bool move_leg_if_needed(Leg& leg, float targetZ, float targetY) {
  // Calculate IK
  leg.fabrik.setTolerance(0.5);
  bool solved = leg.fabrik.solve(targetZ, targetY, leg.lengths);
  if (!solved) return false;

  // Get raw angles from IK solver
  float hipAngle = leg.fabrik.getAngle(0) * RAD_TO_DEG;
  float kneeAngle = leg.fabrik.getAngle(1) * RAD_TO_DEG;

  // Ensure knee bends forward (negative angle from IK solver)
  if (kneeAngle > 0) {
    kneeAngle = -kneeAngle;
    hipAngle = -hipAngle;  // Flip hip angle to maintain valid IK solution
  }

  // Calculate servo angles
  float servoHip, servoKnee;
  if (leg.is_left) {
    servoHip = -hipAngle + leg.hip_offset;
    servoKnee = kneeAngle + leg.knee_offset;
  } else {
    servoHip = hipAngle + leg.hip_offset;
    servoKnee = -kneeAngle + leg.knee_offset;
  }

  // Constrain angles to servo limits
  servoHip = constrain(servoHip, -120.0f, 120.0f);
  servoKnee = constrain(servoKnee, -120.0f, 120.0f);

  // Track target positions
  static float lastHipPos[NUM_LEGS * 2] = { 0 };
  static float lastKneePos[NUM_LEGS * 2] = { 0 };

  // Only move if the change is significant
  bool moved = false;
  if (abs(servoHip - lastHipPos[leg.hip_id - 1]) >= MIN_ANGLE_CHANGE || abs(servoKnee - lastKneePos[leg.knee_id - 1]) >= MIN_ANGLE_CHANGE) {
    servos.moveTo(leg.hip_id, servoHip, MOVE_TIME);
    servos.moveTo(leg.knee_id, servoKnee, MOVE_TIME);
    lastHipPos[leg.hip_id - 1] = servoHip;
    lastKneePos[leg.knee_id - 1] = servoKnee;
    moved = true;

    // Debug output
    Serial.print("Moved Leg ");
    Serial.print(leg.hip_id);
    Serial.print("(");
    Serial.print(leg.is_left ? "L" : "R");
    Serial.print(") ");
    Serial.print("Target(");
    Serial.print(targetZ, 2);
    Serial.print(",");
    Serial.print(targetY, 2);
    Serial.print(") ServoH:");
    Serial.print(servoHip, 1);
    Serial.print("° K:");
    Serial.print(servoKnee, 1);
    Serial.println("°");
  }

  return moved;
}

// Move all legs to specified position (z = height, y = forward/back)
void move_all_legs(float targetZ, float targetY) {
  bool moved = false;
  for (int i = 0; i < NUM_LEGS; i++) {
    moved |= move_leg_if_needed(legs[i], targetZ, targetY);
  }
  if (moved) {
    Serial.print("Moving all legs to Z=");
    Serial.print(targetZ);
    Serial.print(", Y=");
    Serial.println(targetY);
  }
}

// ---- Help Function ----
void show_help() {
  Serial.println("Available commands:");
  Serial.println("  help, ?      - Show this help message");
  Serial.println("  go          - Start moving mode");
  Serial.println("  stop        - Stop current action (moving/walking/calibrating)");
  Serial.println("  disable     - Disable all servos");
  Serial.println("  calibrate   - Enter calibration mode");
  Serial.println("  cancel      - Cancel current operation");
  Serial.println("  save        - Save calibration (in calibration mode)");
  Serial.println("  walk [N]    - Walk N steps (default: 1)");
}

// ---- Command Parser ----
String input_buffer;

void parse_command(const String& cmd);
void enter_moving_mode();
void enter_idle_mode();
void enter_calibration_mode();
void cancel_calibration();
void save_calibration();
void calibration_loop();
void disable_all_servos();

// ---- Calibration State ----
int calibration_current_leg = 0;
bool calibration_waiting = false;
bool calibration_active = false;
bool calibration_save_requested = false;

// ---- LED Effects ----
void handle_leds() {
  static unsigned long lastUpdate = 0;
  static unsigned long lastStateChange = 0;
  unsigned long currentMillis = millis();

  // Update at 60fps (16ms) for smooth animations
  if (currentMillis - lastUpdate < 16) return;
  lastUpdate = currentMillis;

  // Clear all pixels
  pixels.clear();

  // Handle global animations (override state if needed)
  if (calibration_active) {
    // Blinking orange for calibration
    if (currentMillis % 500 < 250) {
      pixels.fill(COLOR_CALIBRATING);
    }
    pixels.show();
    return;
  }

  // Handle state-based animations
  switch (state) {
    case ControllerState::IDLE:
      {
        // Smooth breathing blue effect
        if (currentMillis - animState.lastBreathUpdate > MS_PER_FRAME) {
          animState.lastBreathUpdate = currentMillis;

          // Update brightness
          animState.breathBrightness += animState.breathDirection;

          // Reverse direction at limits
          if (animState.breathBrightness >= 255) {
            animState.breathBrightness = 255;
            animState.breathDirection = -animState.breathDirection;
          } else if (animState.breathBrightness <= 20) {
            animState.breathBrightness = 20;
            animState.breathDirection = -animState.breathDirection;
          }
        }

        // Apply breathing effect to RGB LED
        pixels.setPixelColor(0, pixels.Color(0, 0, animState.breathBrightness));
        break;
      }

    case ControllerState::MOVING:
      {
        // Rainbow cycle effect
        if (currentMillis - animState.lastLEDUpdate > MS_PER_FRAME) {
          animState.lastLEDUpdate = currentMillis;
          animState.rainbowHue = (animState.rainbowHue + 1) % 65535;
          pixels.setPixelColor(0, pixels.gamma32(pixels.ColorHSV(animState.rainbowHue, 255, 200)));
        }
        break;
      }

    case ControllerState::WALKING:
      {
        // Pulsing green effect for walking
        if (currentMillis - animState.lastBreathUpdate > MS_PER_FRAME) {
          animState.lastBreathUpdate = currentMillis;

          // Update brightness
          animState.breathBrightness += animState.breathDirection * 2;

          // Reverse direction at limits
          if (animState.breathBrightness >= 200) {
            animState.breathBrightness = 200;
            animState.breathDirection = -animState.breathDirection;
          } else if (animState.breathBrightness <= 50) {
            animState.breathBrightness = 50;
            animState.breathDirection = -animState.breathDirection;
          }

          // Apply pulsing green to the LED
          pixels.setPixelColor(0, pixels.Color(0, animState.breathBrightness, 0));
        }
        break;
      }

    case ControllerState::CALIBRATING:
      {
        // Handled above as a global state
        break;
      }

    case ControllerState::DISABLED:
      {
        // Error flash pattern for disabled state (1 second cycle = 500ms on, 500ms off)
        if (currentMillis - animState.lastLEDUpdate > 500) {  // Toggle every 500ms
          animState.lastLEDUpdate = currentMillis;
          animState.blinkState = !animState.blinkState;
        }
        pixels.setPixelColor(0, animState.blinkState ? COLOR_ERROR : 0);
        break;
      }
  }

  pixels.show();
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  servos.begin(Serial2);

  // Initialize RGB LED
  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  pixels.clear();
  pixels.show();

  // Test LED sequence (single LED)
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));  // Red
  pixels.show();
  delay(300);
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));  // Green
  pixels.show();
  delay(300);
  pixels.setPixelColor(0, pixels.Color(0, 0, 255));  // Blue
  pixels.show();
  delay(300);
  pixels.clear();
  pixels.show();

  Serial.println("\n=== Multi-Leg Controller ===");
  show_help();
  enter_idle_mode();
}

// ---- Main Loop ----
void loop() {
  // Handle LED animations
  handle_leds();

  // Handle serial input
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (input_buffer.length() > 0) {
        parse_command(input_buffer);
        input_buffer = "";
      }
    } else if (isPrintable(c)) {
      input_buffer += c;
    }
  }
  switch (state) {
    case ControllerState::DISABLED:
      break;
    case ControllerState::IDLE:
      break;
    case ControllerState::MOVING:
      // Existing moving mode behavior
      static bool toggle = false;
      static unsigned long lastMoveTime = 0;
      static float pose1Z = 180, pose1Y = 0;
      static float pose2Z = 55, pose2Y = 0;
      if (millis() - lastMoveTime > PAUSE_TIME + 1000) {
        for (int i = 0; i < NUM_LEGS; ++i) {
          move_leg_if_needed(legs[i], toggle ? pose1Z : pose2Z, toggle ? pose1Y : pose2Y);
        }
        toggle = !toggle;
        lastMoveTime = millis();
      }
      break;
    case ControllerState::WALKING:
      {
        if (!walk_state.is_walking) break;

        unsigned long current_time = millis();
        unsigned long elapsed = current_time - walk_state.phase_start_time;

        // Only process the next phase if enough time has passed
        if (elapsed < MOVE_DURATION) break;

        // Update phase start time for the next movement
        walk_state.phase_start_time = current_time;

        // Handle current phase
        switch (walk_state.phase) {
          case WalkPhase::LIFT_SWING_LEG:
            {
              // Move swing legs up and forward, support legs back
              bool moved = false;
              for (int i = 0; i < NUM_LEGS; i++) {
                bool is_swing_leg = ((i == 0 || i == 3) && !walk_state.diagonal) || ((i == 1 || i == 2) && walk_state.diagonal);

                if (is_swing_leg) {
                  // Swing leg: lift and move forward
                  moved |= move_leg_if_needed(legs[i],
                                              STANDING_HEIGHT - LIFT_HEIGHT,      // Higher Z = lower position
                                              STEP_BACK_DISTANCE + STRIDE_LENGTH  // Move forward
                  );
                } else {
                  // Support leg: push back
                  moved |= move_leg_if_needed(legs[i],
                                              STANDING_HEIGHT,     // Stay at standing height
                                              -STEP_BACK_DISTANCE  // Push back
                  );
                }
              }

              if (moved) {
                // Only move to next phase if we actually moved something
                walk_state.phase = WalkPhase::PLANT_PUSH_LEG;
                Serial.println("Phase: Plant & Push");
              }
              break;
            }

          case WalkPhase::PLANT_PUSH_LEG:
            {
              // Switch diagonals and prepare for next step
              walk_state.diagonal = !walk_state.diagonal;

              // Move all legs to neutral position first
              bool moved = false;
              for (int i = 0; i < NUM_LEGS; i++) {
                moved |= move_leg_if_needed(legs[i], STANDING_HEIGHT, 0);
              }

              if (moved) {
                // Check if we've completed all steps
                if (walk_state.diagonal) {
                  walk_state.steps_taken++;
                  if (walk_state.steps_taken >= walk_state.total_steps) {
                    walk_state.phase = WalkPhase::COMPLETE;
                    Serial.println("Walking complete");
                  } else {
                    walk_state.phase = WalkPhase::LIFT_SWING_LEG;
                    Serial.println("Phase: Next Step");
                  }
                } else {
                  walk_state.phase = WalkPhase::LIFT_SWING_LEG;
                  Serial.println("Phase: Next Step");
                }
              }
              break;
            }

          case WalkPhase::COMPLETE:
            walk_state.is_walking = false;
            state = ControllerState::IDLE;
            break;
        }
        break;
      }
    case ControllerState::CALIBRATING:
      calibration_loop();
      break;
  }
}

void parse_command(const String& cmd) {
  String command = cmd;
  command.trim();
  command.toLowerCase();

  if (command == "go") {
    enter_moving_mode();
  } else if (command == "stop") {
    if (state == ControllerState::MOVING || state == ControllerState::WALKING) {
      enter_idle_mode();
    } else if (state == ControllerState::CALIBRATING) {
      cancel_calibration();
    }
  } else if (command == "disable") {
    disable_all_servos();
  } else if (command == "calibrate") {
    enter_calibration_mode();
  } else if (command == "cancel") {
    if (state == ControllerState::CALIBRATING) {
      cancel_calibration();
    }
  } else if (command == "save") {
    if (state == ControllerState::CALIBRATING) {
      calibration_save_requested = true;
    } else {
      Serial.println("Not in calibration mode");
    }
  } else if (command == "help" || command == "?") {
    show_help();
  } else if (command == "down" || command == "lay down" || command == "stay down") {
    Serial.println("Laying down...");
    move_all_legs(MIN_Z, 0);  // Move to min height, centered
  } else if (command == "up" || command == "get up" || command == "stand") {
    Serial.println("Getting up...");
    move_all_legs(MED_Z, 0);  // Move to medium height, centered
  } else if (command == "forward") {
    Serial.println("Rocking forward...");
    move_all_legs(MED_Z, MAX_Y);
  } else if (command == "back") {
    Serial.println("Rocking backward...");
    move_all_legs(MED_Z, MIN_Y);
  } else if (command.startsWith("walk")) {
    int steps = DEFAULT_STEPS;
    // Parse number of steps if provided
    int space_pos = command.indexOf(' ');
    if (space_pos != -1) {
      String steps_str = command.substring(space_pos + 1);
      steps = steps_str.toInt();
      if (steps <= 0) steps = DEFAULT_STEPS;
    }

    Serial.print("Starting to walk for ");
    Serial.print(steps);
    Serial.println(" steps...");

    // Initialize walking state
    walk_state.is_walking = true;
    walk_state.total_steps = steps;
    walk_state.steps_taken = 0;
    walk_state.phase_start_time = millis();
    walk_state.phase = WalkPhase::LIFT_SWING_LEG;

    // Start walking state
    state = ControllerState::WALKING;
  } else {
    Serial.println("Unknown command: " + command);
  }
}

void enter_moving_mode() {
  Serial.println("[Controller] Entering MOVING mode");
  state = ControllerState::MOVING;
  // Reset any walking state if we're entering moving mode
  walk_state.is_walking = false;
  // Reset animation states
  animState.rainbowHue = 0;
  animState.lastLEDUpdate = 0;
}

void enter_idle_mode() {
  if (state != ControllerState::IDLE) {
    Serial.println("[Controller] Entering IDLE mode");
    state = ControllerState::IDLE;
    // Initialize breathing effect
    animState.breathBrightness = 20;
    animState.breathDirection = 5;
    animState.lastBreathUpdate = 0;
  }
}

void enter_calibration_mode() {
  Serial.println("[Controller] Entering CALIBRATING mode");
  state = ControllerState::CALIBRATING;
  calibration_current_leg = 0;
  calibration_waiting = false;
  calibration_active = true;
  calibration_save_requested = false;
  animState.blinkState = false;
  animState.lastLEDUpdate = 0;
}

void cancel_calibration() {
  Serial.println("[Controller] Calibration canceled.");
  calibration_active = false;
  // Don't force IDLE state, let the previous state continue
  if (state == ControllerState::CALIBRATING) {
    state = ControllerState::IDLE;
  }
}

void save_calibration() {
  Serial.println("[Controller] Calibration saved. Returning to IDLE.");
  calibration_save_requested = true;
  calibration_active = false;
  state = ControllerState::IDLE;
}

void disable_all_servos() {
  Serial.println("[Controller] Disabling all servos");

  // Disable servos
  for (int i = 0; i < NUM_LEGS; ++i) {
    servos.unloadServo(legs[i].hip_id);
    servos.unloadServo(legs[i].knee_id);
  }

  // Set state to DISABLED which will handle the LED pattern
  state = ControllerState::DISABLED;
}

void calibration_loop() {
  if (!calibration_active) return;
  if (calibration_current_leg >= NUM_LEGS) {
    Serial.println("\nAll legs calibrated. Type 'save' to persist or 'cancel' to discard.");
    calibration_active = false;
    return;
  }
  if (!calibration_waiting) {
    Serial.print("\n--- Leg ");
    Serial.print(calibration_current_leg);
    Serial.print(" (Hip ID: ");
    Serial.print(legs[calibration_current_leg].hip_id);
    Serial.print(", Knee ID: ");
    Serial.print(legs[calibration_current_leg].knee_id);
    Serial.println(") ---");
    Serial.println("Move this leg to its mechanical 'straight' pose, then type any character and press Enter.");
    calibration_waiting = true;
  }
  if (Serial.available() > 0) {
    Serial.read();
    // Read servo positions (raw values 0-1000)
    int hipRawPos = servos.readPosition(legs[calibration_current_leg].hip_id);
    int kneeRawPos = servos.readPosition(legs[calibration_current_leg].knee_id);

    // Convert raw position (0-1000) to degrees (-120 to 120)
    float hipAngle = 0, kneeAngle = 0;
    bool hipOk = (hipRawPos != -1);
    bool kneeOk = (kneeRawPos != -1);

    if (hipOk) {
      hipAngle = (hipRawPos / 1000.0) * 240.0 - 120.0;
    }
    if (kneeOk) {
      kneeAngle = (kneeRawPos / 1000.0) * 240.0 - 120.0;
    }
    Serial.print("[ Leg ");
    Serial.print(calibration_current_leg);
    Serial.print(" ] ");
    if (hipOk) {
      Serial.print("Hip Servo (ID ");
      Serial.print(legs[calibration_current_leg].hip_id);
      Serial.print(") angle: ");
      Serial.print(hipAngle, 2);
      Serial.print(" deg");
    } else {
      Serial.print("Hip Servo: Read Failed");
    }
    Serial.print(" | ");
    if (kneeOk) {
      Serial.print("Knee Servo (ID ");
      Serial.print(legs[calibration_current_leg].knee_id);
      Serial.print(") angle: ");
      Serial.print(kneeAngle, 2);
      Serial.print(" deg");
    } else {
      Serial.print("Knee Servo: Read Failed");
    }
    Serial.println();
    calibration_current_leg++;
    calibration_waiting = false;
    delay(500);
  }
}

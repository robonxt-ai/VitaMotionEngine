/*
 * WalkV5_motion.ino
 * Implements Walk V5 (Differential Bezier Gait with Stability).
 * 
 * Features:
 * 1. Differential Steering: Left/Right legs can have different stride lengths to turn.
 * 2. Bezier Trajectory: Inherits smooth V4 lift/land.
 * 3. Open-Loop Stability:
 *    - Leans into turns (Roll compensation).
 *    - Shifts COG forward/back based on acceleration (Pitch compensation).
 */

#include <Arduino.h>
#include "SimpleIK.h"
#include "Ps4Input.h"

// ---- Externs from main_controller_v7.ino ----
extern ControllerState state;
extern void tftMsg(String msg);
extern Leg legs[];
extern HiBusServo servos;
extern Point2D getCubicBezierPoint(float t, Point2D p0, Point2D p1, Point2D p2, Point2D p3);
extern float GLOBAL_SPEED_MULTIPLIER;
extern float GLOBAL_COG_Y_OFFSET;
extern bool moveLegIfNeeded(Leg &leg, float targetZ, float targetY, unsigned long duration, bool commit);
extern int parseSpaceSeparatedFloats(const String &args, float *out, int maxCount);

#ifndef ESP32
extern float lerp(float a, float b, float t);
#endif

// Helper from JoystickHelper.ino
extern void processCommonWalkInput(float &in_x, float &in_y, float &in_pitch, float &in_roll,
                                   float &base_height, float &lift, float &stride,
                                   float h_min, float h_max, float l_min, float l_max, float s_min, float s_max,
                                   float step_lift, float step_stride);

// ---- V5 Configuration ----
const long WALKV5_UPDATE_INTERVAL_MS = 20;
// Default values (used if not overridden)
const float WALKV5_DEFAULT_CYCLE = 600.0f;
const float WALKV5_DEFAULT_STRIDE = 50.0f;
const float WALKV5_DEFAULT_LIFT = 15.0f;
const float WALKV5_DEFAULT_HEIGHT = 140.0f;
const float WALKV5_BODY_Y_OFFSET = 0.0f;

const float WALKV5_MIN_HEIGHT = 100.0f;
const float WALKV5_MAX_HEIGHT = 170.0f;

const float WALKV5_MIN_LIFT = 7.5f;
const float WALKV5_MAX_LIFT = 60.0f;
const float WALKV5_MIN_STRIDE = 10.0f;
const float WALKV5_MAX_STRIDE = 60.0f;

const float WALKV5_STEP_LIFT = 2.5f;
const float WALKV5_STEP_STRIDE = 2.5f;

// Stability Tunables (auto-lean from movement)
float V5_LEAN_MULTIPLIER = 10.0f;   // mm of height diff per 1.0 turning factor
float V5_PITCH_MULTIPLIER = 15.0f;  // mm of COG shift per 1.0 speed factor

// Manual pitch/roll tuning (right stick override)
const float WALKV5_PITCH_SCALE = 25.0f;  // mm height diff for manual pitch
const float WALKV5_ROLL_SCALE = 20.0f;   // mm height diff for manual roll

struct WalkV5State {
  unsigned long start_time = 0;
  unsigned long last_update_time = 0;

  // Joystick Inputs
  float input_y = 0.0f;      // Forward/Back (-1.0 to 1.0)
  float input_x = 0.0f;      // Left/Right (-1.0 to 1.0)
  float input_pitch = 0.0f;  // Right stick Y (manual pitch)
  float input_roll = 0.0f;   // Right stick X (manual roll)

  // Filtered Inputs
  float filtered_x = 0.0f;
  float filtered_y = 0.0f;

  // Dynamic Parameters
  float base_height = WALKV5_DEFAULT_HEIGHT;
  float lift_height = WALKV5_DEFAULT_LIFT;
  float max_stride = WALKV5_DEFAULT_STRIDE;
  float cycle_time = WALKV5_DEFAULT_CYCLE;
};
WalkV5State walkv5_state;


// Calculate leg position for V5 Bezier trot with stability + manual offsets
void calculateBezierTrotLegV5(Leg &leg, float phase, float stride_scale, float roll_offset,
                              float pitch_offset, float effective_lift) {
  float y_pos = 0;
  float z_pos = walkv5_state.base_height;

  // Apply roll (left/right height difference)
  if (leg.is_left) z_pos += roll_offset;
  else z_pos -= roll_offset;

  // Apply pitch (front/rear height difference)
  if (leg.is_front) z_pos += pitch_offset;
  else z_pos -= pitch_offset;

  // Stride for this leg
  float leg_stride = walkv5_state.max_stride * stride_scale;

  if (phase < 0.5f) {
    // --- SWING PHASE ---
    float t = phase / 0.5f;

    Point2D p0 = { -leg_stride, 0.0f };
    Point2D p3 = { leg_stride, 0.0f };
    Point2D p1 = { -leg_stride * 0.6f, -effective_lift * 1.5f };
    Point2D p2 = { leg_stride * 0.6f, -effective_lift * 1.5f };

    Point2D step = getCubicBezierPoint(t, p0, p1, p2, p3);

    y_pos = step.y;
    z_pos += step.z;

  } else {
    // --- STANCE PHASE ---
    float t = (phase - 0.5f) / 0.5f;
    y_pos = lerp(leg_stride, -leg_stride, t);
  }

  // COG shift from movement speed
  float cog_offset = walkv5_state.filtered_y * V5_PITCH_MULTIPLIER;
  float final_y = y_pos - cog_offset + WALKV5_BODY_Y_OFFSET;

  moveLegIfNeeded(leg, z_pos, final_y, WALKV5_UPDATE_INTERVAL_MS, false);
}

void updateWalkV5() {
  unsigned long current_time = millis();
  if (current_time - walkv5_state.last_update_time < WALKV5_UPDATE_INTERVAL_MS) return;
  walkv5_state.last_update_time = current_time;

  processCommonWalkInput(walkv5_state.input_x, walkv5_state.input_y,
                         walkv5_state.input_pitch, walkv5_state.input_roll,
                         walkv5_state.base_height, walkv5_state.lift_height, walkv5_state.max_stride,
                         WALKV5_MIN_HEIGHT, WALKV5_MAX_HEIGHT,
                         WALKV5_MIN_LIFT, WALKV5_MAX_LIFT,
                         WALKV5_MIN_STRIDE, WALKV5_MAX_STRIDE,
                         WALKV5_STEP_LIFT, WALKV5_STEP_STRIDE);

  // Use pre-filtered inputs from JoystickHelper
  walkv5_state.filtered_x = walkv5_state.input_x;
  walkv5_state.filtered_y = walkv5_state.input_y;

  float effective_cycle = walkv5_state.cycle_time;
  if (GLOBAL_SPEED_MULTIPLIER > 0.1) effective_cycle /= GLOBAL_SPEED_MULTIPLIER;

  float time_in_cycle = fmod(current_time - walkv5_state.start_time, effective_cycle);
  float main_phase = time_in_cycle / effective_cycle;

  // Differential drive mixing
  float left_mix = constrain(walkv5_state.filtered_y + walkv5_state.filtered_x, -1.0f, 1.0f);
  float right_mix = constrain(walkv5_state.filtered_y - walkv5_state.filtered_x, -1.0f, 1.0f);

  // Dynamic lift scaling (0 at stop, full at 20%+ speed)
  float speed_mag = max(abs(left_mix), abs(right_mix));
  float current_lift = walkv5_state.lift_height * constrain(speed_mag * 5.0f, 0.0f, 1.0f);

  // Roll: auto-lean from turning + manual from right stick
  float auto_roll = walkv5_state.filtered_x * V5_LEAN_MULTIPLIER;
  float manual_roll = walkv5_state.input_roll * WALKV5_ROLL_SCALE;
  float total_roll = auto_roll + manual_roll;

  // Pitch: manual from right stick
  float manual_pitch = walkv5_state.input_pitch * WALKV5_PITCH_SCALE;

  // Execute legs (diagonal pairs) - pass same roll/pitch to all, function handles left/right/front/rear
  calculateBezierTrotLegV5(legs[LEG_FR], main_phase, right_mix, total_roll, manual_pitch, current_lift);
  calculateBezierTrotLegV5(legs[LEG_RL], main_phase, left_mix, total_roll, manual_pitch, current_lift);

  float pair2_phase = fmod(main_phase + 0.5f, 1.0f);
  calculateBezierTrotLegV5(legs[LEG_FL], pair2_phase, left_mix, total_roll, manual_pitch, current_lift);
  calculateBezierTrotLegV5(legs[LEG_RR], pair2_phase, right_mix, total_roll, manual_pitch, current_lift);

  servos.startMove();
}

void enterWalkV5Mode() {
  state = ControllerState::WALK_V5;
  walkv5_state.start_time = millis();
  walkv5_state.last_update_time = millis();
  walkv5_state.input_x = 0;
  walkv5_state.input_y = 0;
  walkv5_state.input_pitch = 0;
  walkv5_state.input_roll = 0;
  walkv5_state.base_height = WALKV5_DEFAULT_HEIGHT;
  walkv5_state.lift_height = WALKV5_DEFAULT_LIFT;
  walkv5_state.max_stride = WALKV5_DEFAULT_STRIDE;
  walkv5_state.cycle_time = WALKV5_DEFAULT_CYCLE;
  tftMsg("Mode: WALK V5 (Diff Drive)");
}

// Command: walkv5 <y> <x> [height] [lift] [stride]
// y: forward/back speed (-1.0 to 1.0)
// x: turn left/right (-1.0 to 1.0)
void handleWalkV5(const String &args) {
  if (args.length() > 0) {
    float values[5] = { 0.0f, 0.0f, WALKV5_DEFAULT_HEIGHT, WALKV5_DEFAULT_LIFT, WALKV5_DEFAULT_STRIDE };
    int count = parseSpaceSeparatedFloats(args, values, 5);

    walkv5_state.input_y = constrain(values[0], -1.0, 1.0);
    walkv5_state.input_x = constrain(values[1], -1.0, 1.0);
    if (count > 2) walkv5_state.base_height = values[2];
    if (count > 3) walkv5_state.lift_height = values[3];
    if (count > 4) walkv5_state.max_stride = values[4];
  }

  if (state != ControllerState::WALK_V5) {
    enterWalkV5Mode();
  }
}

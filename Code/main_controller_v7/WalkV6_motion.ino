/*
 * WalkV6_motion.ino
 * Implements Walk V6 (Natural Geometric Gait).
 * 
 * Goals:
 * 1. Natural Feel: Uses pure V4 Bezier geometry (no artificial tilting).
 * 2. Control: Supports V5-style differential steering (Joy Y/X) + Dynamic Params.
 * 3. Architecture: Clean separation of steering mixing and leg trajectory.
 */

#include <Arduino.h>
#include "SimpleIK.h"

// ---- Externs from main_controller_v7.ino ----
extern ControllerState state;
extern void tftMsg(String msg);
extern Leg legs[];
extern HiBusServo servos;
extern Point2D getCubicBezierPoint(float t, Point2D p0, Point2D p1, Point2D p2, Point2D p3);
extern float GLOBAL_SPEED_MULTIPLIER;
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

extern void updateBodyManipulation();

// ---- V6 Configuration ----
const long WALKV6_UPDATE_INTERVAL_MS = 20;

// Default values
const float WALKV6_DEFAULT_CYCLE = 600.0f;
const float WALKV6_DEFAULT_STRIDE = 50.0f;
const float WALKV6_DEFAULT_LIFT = 15.0f;
const float WALKV6_DEFAULT_HEIGHT = 140.0f;
const float WALKV6_BODY_Y_OFFSET = 0.0f;

// Limits for Joystick Control
const float WALKV6_MIN_HEIGHT = 90.0f;
const float WALKV6_MAX_HEIGHT = 170.0f;

const float WALKV6_MIN_LIFT = 7.5f;
const float WALKV6_MAX_LIFT = 30.0f;
const float WALKV6_MIN_STRIDE = 10.0f;
const float WALKV6_MAX_STRIDE = 60.0f;

const float WALKV6_STEP_LIFT = 2.5f;
const float WALKV6_STEP_STRIDE = 2.5f;

// Pitch/Roll tuning (right stick body tilt during walk)
const float WALKV6_PITCH_SCALE = 25.0f;  // mm height diff for pitch
const float WALKV6_ROLL_SCALE = 20.0f;   // mm height diff for roll

struct WalkV6State {
  unsigned long start_time = 0;
  unsigned long last_update_time = 0;

  // Joystick Inputs
  float input_y = 0.0f;      // Forward/Back (-1.0 to 1.0)
  float input_x = 0.0f;      // Left/Right (-1.0 to 1.0)
  float input_pitch = 0.0f;  // Right stick Y (body pitch)
  float input_roll = 0.0f;   // Right stick X (body roll)

  // Dynamic Parameters
  float base_height = WALKV6_DEFAULT_HEIGHT;
  float lift_height = WALKV6_DEFAULT_LIFT;
  float max_stride = WALKV6_DEFAULT_STRIDE;
  float cycle_time = WALKV6_DEFAULT_CYCLE;
};
WalkV6State walkv6_state;

// Calculate leg position for V6 Bezier trot with pitch/roll offsets
void calculateBezierTrotLegV6(Leg &leg, float phase, float stride_scale, float effective_lift,
                              float pitch_offset, float roll_offset) {
  float y_pos = 0;
  float z_pos = walkv6_state.base_height;

  // Apply pitch (front/rear height difference)
  if (leg.is_front) z_pos += pitch_offset;
  else z_pos -= pitch_offset;

  // Apply roll (left/right height difference)
  if (leg.is_left) z_pos += roll_offset;
  else z_pos -= roll_offset;

  float leg_stride = walkv6_state.max_stride * stride_scale;

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

  float final_y = y_pos + WALKV6_BODY_Y_OFFSET;

  moveLegIfNeeded(leg, z_pos, final_y, WALKV6_UPDATE_INTERVAL_MS, false);
}

void updateWalkV6() {
  unsigned long current_time = millis();
  if (current_time - walkv6_state.last_update_time < WALKV6_UPDATE_INTERVAL_MS) return;
  walkv6_state.last_update_time = current_time;


  processCommonWalkInput(walkv6_state.input_x, walkv6_state.input_y,
                         walkv6_state.input_pitch, walkv6_state.input_roll,
                         walkv6_state.base_height, walkv6_state.lift_height, walkv6_state.max_stride,
                         WALKV6_MIN_HEIGHT, WALKV6_MAX_HEIGHT,
                         WALKV6_MIN_LIFT, WALKV6_MAX_LIFT,
                         WALKV6_MIN_STRIDE, WALKV6_MAX_STRIDE,
                         WALKV6_STEP_LIFT, WALKV6_STEP_STRIDE);

  float effective_cycle = walkv6_state.cycle_time;
  if (GLOBAL_SPEED_MULTIPLIER > 0.1) effective_cycle /= GLOBAL_SPEED_MULTIPLIER;

  float time_in_cycle = fmod(current_time - walkv6_state.start_time, effective_cycle);
  float main_phase = time_in_cycle / effective_cycle;

  // Differential drive mixing
  float left_mix = constrain(walkv6_state.input_y + walkv6_state.input_x, -1.0f, 1.0f);
  float right_mix = constrain(walkv6_state.input_y - walkv6_state.input_x, -1.0f, 1.0f);

  // Dynamic lift scaling (0 at stop, full at 20%+ speed)
  float speed_mag = max(abs(left_mix), abs(right_mix));
  float current_lift = walkv6_state.lift_height * constrain(speed_mag * 5.0f, 0.0f, 1.0f);

  // Calculate pitch/roll offsets from right stick
  float pitch_offset = walkv6_state.input_pitch * WALKV6_PITCH_SCALE;
  float roll_offset = walkv6_state.input_roll * WALKV6_ROLL_SCALE;

  // Execute legs (diagonal pairs)
  calculateBezierTrotLegV6(legs[LEG_FR], main_phase, right_mix, current_lift, pitch_offset, roll_offset);
  calculateBezierTrotLegV6(legs[LEG_RL], main_phase, left_mix, current_lift, pitch_offset, roll_offset);

  float pair2_phase = fmod(main_phase + 0.5f, 1.0f);
  calculateBezierTrotLegV6(legs[LEG_FL], pair2_phase, left_mix, current_lift, pitch_offset, roll_offset);
  calculateBezierTrotLegV6(legs[LEG_RR], pair2_phase, right_mix, current_lift, pitch_offset, roll_offset);

  servos.startMove();
}

void enterWalkV6Mode() {
  state = ControllerState::WALK_V6;
  walkv6_state.start_time = millis();
  walkv6_state.last_update_time = millis();
  walkv6_state.input_x = 0;
  walkv6_state.input_y = 0;
  walkv6_state.input_pitch = 0;
  walkv6_state.input_roll = 0;
  walkv6_state.base_height = WALKV6_DEFAULT_HEIGHT;
  walkv6_state.lift_height = WALKV6_DEFAULT_LIFT;
  walkv6_state.max_stride = WALKV6_DEFAULT_STRIDE;
  walkv6_state.cycle_time = WALKV6_DEFAULT_CYCLE;

  tftMsg("Mode: WALK V6 (Natural)");
}

// Command: walkv6 <y> <x> [height] [lift] [stride]
void handleWalkV6(const String &args) {
  if (args.length() > 0) {
    float values[5] = { 0.0f, 0.0f, WALKV6_DEFAULT_HEIGHT, WALKV6_DEFAULT_LIFT, WALKV6_DEFAULT_STRIDE };
    int count = parseSpaceSeparatedFloats(args, values, 5);

    walkv6_state.input_y = constrain(values[0], -1.0f, 1.0f);
    walkv6_state.input_x = constrain(values[1], -1.0f, 1.0f);
    if (count > 2) walkv6_state.base_height = values[2];
    if (count > 3) walkv6_state.lift_height = values[3];
    if (count > 4) walkv6_state.max_stride = values[4];
  }

  if (state != ControllerState::WALK_V6) {
    enterWalkV6Mode();
  }
}

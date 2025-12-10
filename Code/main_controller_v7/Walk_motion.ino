/*
 * V4_motion.ino
 * Implements Walk V4 (Bezier Curve Gait) for smoother, more organic motion.
 * 
 * Uses Cubic Bezier curves for the swing phase to define a trajectory that:
 * 1. Lifts sharply (P1)
 * 2. Glides forward (Peak)`
 * 3. Lands gently (P2 -> P3) with near-zero vertical velocity
 */

#include <Arduino.h>
#include "SimpleIK.h"

// Externs from main_controller_v7.ino
extern ControllerState state;
extern void tftMsg(String msg);
extern Leg legs[];
extern HiBusServo servos;
extern float GLOBAL_SPEED_MULTIPLIER;
extern bool moveLegIfNeeded(Leg &leg, float targetZ, float targetY, unsigned long duration, bool commit);
extern int parseSpaceSeparatedFloats(const String &args, float *out, int maxCount);
#ifndef ESP32
extern float lerp(float a, float b, float t);
#endif

// ---- V4 Configuration ----
const long WALKV4_UPDATE_INTERVAL_MS = 20;  // 50Hz
float WALKV4_CYCLE_TIME = 600.0f;           // slightly faster natural trot
float WALKV4_STRIDE_LENGTH = 40.0f;
float WALKV4_LIFT_HEIGHT = 10.0f;
float WALKV4_BASE_HEIGHT = 150.0f;
const float WALKV4_BODY_Y_OFFSET = 0.0f;  // Local body Y offset for V4

struct WalkV4State {
  unsigned long start_time = 0;
  unsigned long last_update_time = 0;
  float forward_factor = 1.0f;
};
WalkV4State walkv4_state;

// Cubic Bezier calculation
Point2D getCubicBezierPoint(float t, Point2D p0, Point2D p1, Point2D p2, Point2D p3) {
  float u = 1.0f - t;
  float tt = t * t;
  float uu = u * u;
  float uuu = uu * u;
  float ttt = tt * t;

  Point2D result;
  result.y = (uuu * p0.y) + (3 * uu * t * p1.y) + (3 * u * tt * p2.y) + (ttt * p3.y);
  result.z = (uuu * p0.z) + (3 * uu * t * p1.z) + (3 * u * tt * p2.z) + (ttt * p3.z);

  return result;
}

// Calculate leg position for V4 Bezier trot
void calculateBezierTrotLeg(Leg &leg, float phase, float forward_factor) {
  float y_pos = 0;
  float z_pos = WALKV4_BASE_HEIGHT;

  if (phase < 0.5f) {
    // Swing phase
    float t = phase / 0.5f;
    Point2D p0 = { -WALKV4_STRIDE_LENGTH, 0.0f };
    Point2D p3 = { WALKV4_STRIDE_LENGTH, 0.0f };
    Point2D p1 = { -WALKV4_STRIDE_LENGTH * 0.6f, -WALKV4_LIFT_HEIGHT * 1.5f };
    Point2D p2 = { WALKV4_STRIDE_LENGTH * 0.6f, -WALKV4_LIFT_HEIGHT * 1.5f };
    Point2D step = getCubicBezierPoint(t, p0, p1, p2, p3);
    y_pos = step.y;
    z_pos = WALKV4_BASE_HEIGHT + step.z;
  } else {
    // Stance phase
    float t = (phase - 0.5f) / 0.5f;
    y_pos = lerp(WALKV4_STRIDE_LENGTH, -WALKV4_STRIDE_LENGTH, t);
    z_pos = WALKV4_BASE_HEIGHT;
  }

  y_pos *= forward_factor;
  float final_y = y_pos + WALKV4_BODY_Y_OFFSET;

  moveLegIfNeeded(leg, z_pos, final_y, WALKV4_UPDATE_INTERVAL_MS, false);
}

void updateWalkV4() {
  unsigned long current_time = millis();
  if (current_time - walkv4_state.last_update_time < WALKV4_UPDATE_INTERVAL_MS) return;
  walkv4_state.last_update_time = current_time;

  float effective_cycle = WALKV4_CYCLE_TIME;
  if (GLOBAL_SPEED_MULTIPLIER > 0.1) effective_cycle /= GLOBAL_SPEED_MULTIPLIER;

  float time_in_cycle = fmod(current_time - walkv4_state.start_time, effective_cycle);
  float main_phase = time_in_cycle / effective_cycle;

  // Pair 1 (FR, RL) - Phase 0.0
  calculateBezierTrotLeg(legs[LEG_FR], main_phase, walkv4_state.forward_factor);
  calculateBezierTrotLeg(legs[LEG_RL], main_phase, walkv4_state.forward_factor);

  // Pair 2 (FL, RR) - Phase 0.5
  float pair2_phase = fmod(main_phase + 0.5f, 1.0f);
  calculateBezierTrotLeg(legs[LEG_FL], pair2_phase, walkv4_state.forward_factor);
  calculateBezierTrotLeg(legs[LEG_RR], pair2_phase, walkv4_state.forward_factor);

  servos.startMove();
}

void enterWalkV4Mode(float forward_scale) {
  state = ControllerState::WALK_V4;
  walkv4_state.forward_factor = forward_scale;
  walkv4_state.start_time = millis();
  walkv4_state.last_update_time = millis();
  tftMsg("Mode: WALK V4 (Bezier)");
}

void handleWalkV4(const String &args) {
  float factor = 1.0;

  if (args.length() > 0) {
    float values[4] = { 1.0f, WALKV4_BASE_HEIGHT, WALKV4_STRIDE_LENGTH, WALKV4_LIFT_HEIGHT };
    int count = parseSpaceSeparatedFloats(args, values, 4);

    factor = constrain(values[0], -1.0, 1.0);
    if (count > 1) WALKV4_BASE_HEIGHT = values[1];
    if (count > 2) WALKV4_STRIDE_LENGTH = values[2];
    if (count > 3) WALKV4_LIFT_HEIGHT = values[3];
  }

  enterWalkV4Mode(factor);
}

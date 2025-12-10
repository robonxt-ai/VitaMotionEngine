#include <Arduino.h>
#include "Ps4Input.h"

// --------------------------------------------------------
// Modular Joystick Input Helper
// --------------------------------------------------------
// Universal joystick filtering for all modes.
// Provides:
// - processCommonWalkInput(): For walk modes (LX/LY + RX/RY pitch/roll + L2/R2 height + D-Pad tuning)
// - processCommonManipulationInput(): For body manipulation (all 4 axes + triggers)

extern const float JOYSTICK_DEADZONE;

extern void triggerErrorFeedback();

// --------------------------------------------------------
// Universal Smoothing Constants
// --------------------------------------------------------
static const float STICK_ALPHA = 0.1f;           // Standard stick smoothing
static const float HEIGHT_ALPHA = 0.05f;         // Slower filter for height/triggers
static const float CARDINAL_SNAP_ANGLE = 20.0f;  // Degrees from cardinal to snap (0-45)

static bool common_prevDpadUp = false;
static bool common_prevDpadDown = false;
static bool common_prevDpadLeft = false;
static bool common_prevDpadRight = false;

void processCommonWalkInput(float &in_x, float &in_y, float &in_pitch, float &in_roll,
                            float &base_height, float &lift, float &stride,
                            float h_min, float h_max, float l_min, float l_max, float s_min, float s_max,
                            float step_lift, float step_stride) {
#if defined(ENABLE_PS4) || defined(ENABLE_PS4_V7)
  if (!g_ps4Input.connected) return;

  // Left stick: movement inputs with deadzone
  in_y = g_ps4Input.walkY;
  in_x = g_ps4Input.walkX;
  if (abs(in_y) < JOYSTICK_DEADZONE) in_y = 0.0f;
  if (abs(in_x) < JOYSTICK_DEADZONE) in_x = 0.0f;

  // Cardinal direction snapping (makes it easier to walk straight or turn in place)
  float mag = sqrtf(in_x * in_x + in_y * in_y);
  if (mag > JOYSTICK_DEADZONE) {
    float angle = atan2f(in_x, in_y) * 180.0f / PI;  // -180 to 180, 0 = forward
    float snap_range = CARDINAL_SNAP_ANGLE;

    // Snap to cardinal directions if within range
    if (abs(angle) < snap_range) {
      in_x = 0.0f;  // Snap to forward
    } else if (abs(angle - 180.0f) < snap_range || abs(angle + 180.0f) < snap_range) {
      in_x = 0.0f;  // Snap to backward
    } else if (abs(angle - 90.0f) < snap_range) {
      in_y = 0.0f;  // Snap to right
    } else if (abs(angle + 90.0f) < snap_range) {
      in_y = 0.0f;  // Snap to left
    }
  }

  // Right stick: pitch/roll inputs with deadzone
  float raw_ry = g_ps4Input.ryRaw / 128.0f;  // Pitch (forward/back tilt)
  float raw_rx = g_ps4Input.rxRaw / 128.0f;  // Roll (left/right tilt)
  if (abs(raw_ry) < JOYSTICK_DEADZONE) raw_ry = 0.0f;
  if (abs(raw_rx) < JOYSTICK_DEADZONE) raw_rx = 0.0f;

  // Smoothing filters (reset if inactive > 200ms)
  static float smoothed_x = 0.0f, smoothed_y = 0.0f;
  static float smoothed_pitch = 0.0f, smoothed_roll = 0.0f;
  static unsigned long last_call_time = 0;
  if (millis() - last_call_time > 200) {
    smoothed_x = smoothed_y = smoothed_pitch = smoothed_roll = 0.0f;
  }
  last_call_time = millis();

  smoothed_x += (in_x - smoothed_x) * STICK_ALPHA;
  smoothed_y += (in_y - smoothed_y) * STICK_ALPHA;
  smoothed_pitch += (raw_ry - smoothed_pitch) * STICK_ALPHA;
  smoothed_roll += (raw_rx - smoothed_roll) * STICK_ALPHA;

  if (abs(smoothed_x) < 0.001f) smoothed_x = 0.0f;
  if (abs(smoothed_y) < 0.001f) smoothed_y = 0.0f;
  if (abs(smoothed_pitch) < 0.001f) smoothed_pitch = 0.0f;
  if (abs(smoothed_roll) < 0.001f) smoothed_roll = 0.0f;

  in_x = smoothed_x;
  in_y = smoothed_y;
  in_pitch = smoothed_pitch;
  in_roll = smoothed_roll;

  // Height adjustment (L2 = crouch, R2 = extend)
  float neutral_h = (h_min + h_max) * 0.5f;
  float val_l2 = g_ps4Input.l2Raw / 255.0f;
  float val_r2 = g_ps4Input.r2Raw / 255.0f;
  float target_h_mod = (val_r2 - val_l2) * (h_max - h_min) * 0.5f;

  static float smoothed_h_mod = 0.0f;
  smoothed_h_mod = smoothed_h_mod + (target_h_mod - smoothed_h_mod) * HEIGHT_ALPHA;

  float h = neutral_h + smoothed_h_mod;

  if (h < h_min) h = h_min;
  if (h > h_max) h = h_max;

  base_height = h;

  // D-Pad tuning (lift/stride)
  bool upNow = g_ps4Input.dpadUp;
  bool downNow = g_ps4Input.dpadDown;
  bool leftNow = g_ps4Input.dpadLeft;
  bool rightNow = g_ps4Input.dpadRight;

  bool upEdge = upNow && !common_prevDpadUp;
  bool downEdge = downNow && !common_prevDpadDown;
  bool leftEdge = leftNow && !common_prevDpadLeft;
  bool rightEdge = rightNow && !common_prevDpadRight;

  common_prevDpadUp = upNow;
  common_prevDpadDown = downNow;
  common_prevDpadLeft = leftNow;
  common_prevDpadRight = rightNow;

  if (upEdge) lift += step_lift;
  if (downEdge) lift -= step_lift;
  float prevLift = lift;
  lift = constrain(lift, l_min, l_max);
  if ((upEdge && lift >= l_max) || (downEdge && lift <= l_min) || (lift != prevLift && (lift == l_min || lift == l_max))) {
    triggerErrorFeedback();
  }

  if (rightEdge) stride += step_stride;
  if (leftEdge) stride -= step_stride;
  float prevStride = stride;
  stride = constrain(stride, s_min, s_max);
  if ((rightEdge && stride >= s_max) || (leftEdge && stride <= s_min) || (stride != prevStride && (stride == s_min || stride == s_max))) {
    triggerErrorFeedback();
  }

#endif
}

// --------------------------------------------------------
// Universal Body Manipulation Input Processing
// --------------------------------------------------------
// Processes all 4 stick axes + triggers with deadzone and smoothing.
// Used by Idle/Body Manipulation mode.

void processCommonManipulationInput(float &out_lx, float &out_ly, float &out_rx, float &out_ry,
                                    float &out_l2, float &out_r2) {
#if defined(ENABLE_PS4) || defined(ENABLE_PS4_V7)
  if (!g_ps4Input.connected) return;

  // Read and normalize raw inputs
  float raw_lx = g_ps4Input.lxRaw / 128.0f;
  float raw_ly = g_ps4Input.lyRaw / 128.0f;
  float raw_rx = g_ps4Input.rxRaw / 128.0f;
  float raw_ry = g_ps4Input.ryRaw / 128.0f;
  float raw_l2 = g_ps4Input.l2Raw / 255.0f;
  float raw_r2 = g_ps4Input.r2Raw / 255.0f;

  // Apply deadzone
  if (abs(raw_lx) < JOYSTICK_DEADZONE) raw_lx = 0.0f;
  if (abs(raw_ly) < JOYSTICK_DEADZONE) raw_ly = 0.0f;
  if (abs(raw_rx) < JOYSTICK_DEADZONE) raw_rx = 0.0f;
  if (abs(raw_ry) < JOYSTICK_DEADZONE) raw_ry = 0.0f;

  // Smoothing filters (reset if inactive > 200ms)
  static float smoothed_lx = 0.0f, smoothed_ly = 0.0f;
  static float smoothed_rx = 0.0f, smoothed_ry = 0.0f;
  static float smoothed_l2 = 0.0f, smoothed_r2 = 0.0f;
  static unsigned long last_manip_call_time = 0;

  if (millis() - last_manip_call_time > 200) {
    smoothed_lx = smoothed_ly = smoothed_rx = smoothed_ry = smoothed_l2 = smoothed_r2 = 0.0f;
  }
  last_manip_call_time = millis();

  smoothed_lx += (raw_lx - smoothed_lx) * STICK_ALPHA;
  smoothed_ly += (raw_ly - smoothed_ly) * STICK_ALPHA;
  smoothed_rx += (raw_rx - smoothed_rx) * STICK_ALPHA;
  smoothed_ry += (raw_ry - smoothed_ry) * STICK_ALPHA;
  smoothed_l2 += (raw_l2 - smoothed_l2) * STICK_ALPHA;
  smoothed_r2 += (raw_r2 - smoothed_r2) * STICK_ALPHA;

  if (abs(smoothed_lx) < 0.001f) smoothed_lx = 0.0f;
  if (abs(smoothed_ly) < 0.001f) smoothed_ly = 0.0f;
  if (abs(smoothed_rx) < 0.001f) smoothed_rx = 0.0f;
  if (abs(smoothed_ry) < 0.001f) smoothed_ry = 0.0f;

  out_lx = smoothed_lx;
  out_ly = smoothed_ly;
  out_rx = smoothed_rx;
  out_ry = smoothed_ry;
  out_l2 = smoothed_l2;
  out_r2 = smoothed_r2;
#endif
}

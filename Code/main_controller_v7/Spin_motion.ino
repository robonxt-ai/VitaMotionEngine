// Spin_motion.ino
// Contains the SpinV2 (geometric spin) behavior update loop.

#include <Arduino.h>
#include "SimpleIK.h"

// Externs from main_controller_v7.ino
extern ControllerState state;
extern GaitTurnState turn_in_place_state;
extern Leg legs[];
extern const int NUM_LEGS;
extern float GLOBAL_SPEED_MULTIPLIER;
extern bool moveLegIfNeeded(Leg &leg, float targetZ, float targetY, unsigned long duration, bool commit);
extern bool moveLegGeometric(Leg &leg, float targetZ, float targetY, unsigned long duration);
extern const int LEG_FL;
extern const int LEG_FR;
extern const int LEG_RL;
extern const int LEG_RR;
extern void tftMsg(String msg);
#ifndef ESP32
extern float lerp(float a, float b, float t);
#endif

struct GaitKeyframe {
  float y;
  float z;
};

const float TURN_Y_OFFSET = 10.0f;
const float TURN_LEG_DOWN_HEIGHT = 150.0f;
const float TURN_LEG_UP_HEIGHT = 140.0f;
const GaitKeyframe TURN_IN_PLACE_SEQUENCE[] = {
  { -TURN_Y_OFFSET, TURN_LEG_DOWN_HEIGHT },
  { -TURN_Y_OFFSET, TURN_LEG_UP_HEIGHT },
  { TURN_Y_OFFSET, TURN_LEG_UP_HEIGHT },
  { TURN_Y_OFFSET, TURN_LEG_DOWN_HEIGHT },
};
const int TURN_IN_PLACE_NUM_KEYFRAMES = sizeof(TURN_IN_PLACE_SEQUENCE) / sizeof(TURN_IN_PLACE_SEQUENCE[0]);
const long spinGeoUpdateIntervalMs = 50;

void enterSpinV2Mode(int dir) {
  state = ControllerState::SPIN_V2;
  turn_in_place_state.direction = dir;
  turn_in_place_state.start_time = millis();
  tftMsg("Mode: SPIN V2 (Geometric)");
}

void updateSpinV2() {
  if (state != ControllerState::SPIN_V2) return;

  unsigned long current_time = millis();
  if (current_time - turn_in_place_state.last_update_time < spinGeoUpdateIntervalMs * 1.5) return;
  turn_in_place_state.last_update_time = current_time;

  int num_frames = TURN_IN_PLACE_NUM_KEYFRAMES;
  long total_cycle_time = num_frames * spinGeoUpdateIntervalMs * 1.5 * (1.0f / GLOBAL_SPEED_MULTIPLIER);

  for (int i = 0; i < NUM_LEGS; i++) {
    int leg_offset_idx = (i == 0 || i == 3) ? 0 : num_frames / 2;

    unsigned long elapsed = current_time - turn_in_place_state.start_time;
    float current_frame_float = (float)(elapsed % total_cycle_time) / total_cycle_time * num_frames;
    current_frame_float = fmod(current_frame_float + leg_offset_idx, num_frames);

    int frame_idx_a = (int)current_frame_float;
    int frame_idx_b = (frame_idx_a + 1) % num_frames;
    float t = current_frame_float - frame_idx_a;

    float targetY = lerp(TURN_IN_PLACE_SEQUENCE[frame_idx_a].y, TURN_IN_PLACE_SEQUENCE[frame_idx_b].y, t);
    float targetZ = lerp(TURN_IN_PLACE_SEQUENCE[frame_idx_a].z, TURN_IN_PLACE_SEQUENCE[frame_idx_b].z, t);

    // Direction: LEFT = left legs back, RIGHT = left legs forward
    float direction_mult = (turn_in_place_state.direction == -1)
                             ? (legs[i].is_left ? -1.0f : 1.0f)
                             : (legs[i].is_left ? 1.0f : -1.0f);

    moveLegGeometric(legs[i], targetZ, targetY * direction_mult, spinGeoUpdateIntervalMs);
  }
}

void handleSpinV2(const String &args) {
  int dir = 1;
  if (args == "left" || args == "l") dir = -1;
  enterSpinV2Mode(dir);
}

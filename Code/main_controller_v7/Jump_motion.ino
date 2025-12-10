// Jump_motion.ino
// Contains the JUMP_V1 and JUMP_V2 behavior update loops.

#include <Arduino.h>
#include "SimpleIK.h"

// Externs from main_controller_v7.ino
extern ControllerState state;
extern JumpState jump_state;
extern Leg legs[];
extern HiBusServo servos;
extern const int NUM_LEGS;
extern const float LEG_MIN_Z;
extern const float LEG_MID_Z;
extern const float LEG_MAX_Z;
extern void tftMsg(String msg);
extern bool moveLegIfNeeded(Leg &leg, float targetZ, float targetY, unsigned long duration, bool commit);
extern void moveAllLegs(float targetZ, float targetY, unsigned long duration);
extern void moveAllLegsGeometric(float targetZ, float targetY, unsigned long duration);
extern void enterIdleMode();

void enterJumpV1Mode() {
  state = ControllerState::JUMP_V1;
  jump_state.phase = JumpPhase::JUMP_START;
  tftMsg("Entering JUMPING state.");
}

void enterJumpV2Mode() {
  state = ControllerState::JUMP_V2;
  jump_state.phase = JumpPhase::JUMP_START;
  jump_state.phase_start_time = millis();
  tftMsg("Mode: JUMP V2 (Geometric)");
}

void updateJumpV1() {
  if (state != ControllerState::JUMP_V1) return;

  unsigned long current_time = millis();

  switch (jump_state.phase) {
    case JumpPhase::JUMP_START:
      tftMsg("Jumping: crouch...");
      moveAllLegs(100, 40, 300);  // crouch
      jump_state.phase = JumpPhase::JUMP_EXTEND;
      jump_state.phase_start_time = current_time;
      break;

    case JumpPhase::JUMP_EXTEND:
      if (current_time - jump_state.phase_start_time >= 400) {
        tftMsg("Jumping: Jump!");
        for (int i = 0; i < NUM_LEGS; i++) moveLegIfNeeded(legs[i], LEG_MAX_Z, -40.0f, 150, false);
        servos.startMove();
        jump_state.phase_start_time = millis();
        jump_state.phase = JumpPhase::JUMP_RESET;
      }
      break;

    case JumpPhase::JUMP_RESET:
      if (current_time - jump_state.phase_start_time >= 600) {
        tftMsg("Jumping: Resetting.");
        moveAllLegs(LEG_MID_Z, 0, 100);
        jump_state.phase = JumpPhase::JUMP_FINISH;
        jump_state.phase_start_time = current_time;
      }
      break;

    case JumpPhase::JUMP_FINISH:
      if (current_time - jump_state.phase_start_time >= 600) {
        tftMsg("Jump complete.");
        enterIdleMode();
      }
      break;
  }
}

void updateJumpV2() {
  if (state != ControllerState::JUMP_V2) return;

  unsigned long current_time = millis();

  switch (jump_state.phase) {
    case JumpPhase::JUMP_START:
      if (current_time - jump_state.phase_start_time > 1000) {
        tftMsg("JUMP V2: Launch!");
        jump_state.phase = JumpPhase::JUMP_EXTEND;
        jump_state.phase_start_time = current_time;
        moveAllLegsGeometric(LEG_MAX_Z, 0.0f, 50);
      } else if (current_time - jump_state.phase_start_time < 100) {
        tftMsg("JUMP V2: Crouching...");
        moveAllLegsGeometric(LEG_MIN_Z, 0.0f, 1000);
      }
      break;

    case JumpPhase::JUMP_EXTEND:
      if (current_time - jump_state.phase_start_time > 150) {
        tftMsg("JUMP V2: Retract");
        jump_state.phase = JumpPhase::JUMP_RESET;
        jump_state.phase_start_time = current_time;
        moveAllLegsGeometric(LEG_MIN_Z, 0.0f, 100);
      }
      break;

    case JumpPhase::JUMP_RESET:
      if (current_time - jump_state.phase_start_time > 300) {
        tftMsg("JUMP V2: Finish");
        jump_state.phase = JumpPhase::JUMP_FINISH;
        jump_state.phase_start_time = current_time;
        moveAllLegsGeometric(LEG_MID_Z, 0.0f, 500);
      }
      break;

    case JumpPhase::JUMP_FINISH:
      if (current_time - jump_state.phase_start_time > 600) {
        state = ControllerState::STATE_IDLE;
      }
      break;
  }
}

void handleJumpV1(const String &) {
  enterJumpV1Mode();
}

void handleJumpV2(const String &args) {
  (void)args;  // args currently unused
  enterJumpV2Mode();
}

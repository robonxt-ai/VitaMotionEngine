// Demo_motion.ino
// Contains the DEMO behavior update loop.

#include <Arduino.h>
#include "SimpleIK.h"

// Externs from main_controller_v7.ino
extern ControllerState state;
extern Leg legs[];
extern const int NUM_LEGS;
extern const float LEG_MIN_Z;
extern const float LEG_MID_Z;
extern const unsigned long DEFAULT_PAUSE_TIME;
extern bool moveLegIfNeeded(Leg &leg, float targetZ, float targetY, unsigned long duration, bool commit);
extern void tftMsg(String msg);

void enterDemoMode() {
  tftMsg("[Controller] Entering DEMO mode");
  state = ControllerState::DEMO;
}

void updateDemo() {
  // Immediately exit if state has changed
  if (state != ControllerState::DEMO) return;

  static bool toggle = false;
  static unsigned long lastMoveTime = 0;
  static float pose1Z = LEG_MID_Z, pose1Y = 0;
  static float pose2Z = LEG_MIN_Z, pose2Y = 0;
  if (millis() - lastMoveTime > DEFAULT_PAUSE_TIME + 1000) {
    for (int i = 0; i < NUM_LEGS; ++i) {
      moveLegIfNeeded(legs[i], toggle ? pose1Z : pose2Z, toggle ? pose1Y : pose2Y);
    }
    toggle = !toggle;
    lastMoveTime = millis();
  }
}

void handleDemo(const String &) {
  enterDemoMode();
}

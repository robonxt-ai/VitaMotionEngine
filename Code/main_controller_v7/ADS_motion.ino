// ADS_motion.ino
// Contains the Alternating Diagonal Step (ADS) behavior update loop.

#include <Arduino.h>
#include "SimpleIK.h"

// Externs from main_controller_v7.ino
extern ControllerState state;
extern Leg legs[];
extern const int NUM_LEGS;
extern bool moveLegIfNeeded(Leg &leg, float targetZ, float targetY, unsigned long duration, bool commit);
extern const int LEG_FL;
extern const int LEG_RR;
extern void tftMsg(String msg);

void enterTrotAdsV1Mode() {
  tftMsg("[Controller] Entering TROT_ADS_V1 mode");
  state = ControllerState::TROT_ADS_V1;
}

// ADS / trot gait parameters (local to ADS behavior)
const unsigned long TROT_MOVE_TIME = 100;                       // Time for each movement phase (ms)
const unsigned long TROT_MOVE_DURATION = TROT_MOVE_TIME + 100;  // Time to pause between movements (ms)
const float TROT_LIFT_HEIGHT = 20.0f;                           // How high to lift leg (mm)
extern const float LEG_MID_Z;                                   // From main, used to define standing height

void updateTrotAdsV1() {
  if (state != ControllerState::TROT_ADS_V1) return;

  static bool trot_toggle = false;
  static unsigned long lastTrotTime = 0;
  if (millis() - lastTrotTime > TROT_MOVE_DURATION) {
    for (int i = 0; i < NUM_LEGS; ++i) {
      bool up = ((i == LEG_FL || i == LEG_RR) ? trot_toggle : !trot_toggle);
      float z = up ? (LEG_MID_Z - TROT_LIFT_HEIGHT) : LEG_MID_Z;
      moveLegIfNeeded(legs[i], z, 0, TROT_MOVE_TIME, false);
    }
    servos.startMove();  // Sync Write commit
    trot_toggle = !trot_toggle;
    lastTrotTime = millis();
  }
}

void handleTrotAdsV1(const String &) {
  enterTrotAdsV1Mode();
}

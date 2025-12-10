#include <Arduino.h>
#include "SimpleIK.h"
#include "Ps4Input.h"

extern ControllerState state;
extern Leg legs[];
extern const int NUM_LEGS;
extern HiBusServo servos;
extern const float LEG_MID_Z;
extern const float LEG_MAX_Z;
extern const float LEG_MIN_Z;

extern bool moveLegIfNeeded(Leg &leg, float targetZ, float targetY, unsigned long duration, bool commit);

// Universal input processing from JoystickHelper.ino
extern void processCommonManipulationInput(float &out_lx, float &out_ly, float &out_rx, float &out_ry,
                                           float &out_l2, float &out_r2);

// Tuning Parameters
const float IDLE_SHIFT_SCALE = 50.0f;  // Max mm shift
const float IDLE_PITCH_SCALE = 40.0f;  // Max height diff for pitch
const float IDLE_ROLL_SCALE = 30.0f;   // Max height diff for roll

void updateBodyManipulation() {
#if ENABLE_PS4
	if (!g_ps4Input.connected) return;

	// Get filtered inputs from universal helper (deadzone + smoothing applied)
	float filter_ly = 0.0f, filter_rx = 0.0f, filter_ry = 0.0f;
	float filter_l2 = 0.0f, filter_r2 = 0.0f;
	float unused_lx = 0.0f;  // LX reserved for future 3DOF
	processCommonManipulationInput(unused_lx, filter_ly, filter_rx, filter_ry, filter_l2, filter_r2);

	// Calculate body manipulation offsets
	float leg_common_y = filter_ly * IDLE_SHIFT_SCALE;
	float pitch_offset = filter_ry * IDLE_PITCH_SCALE;
	float roll_offset = filter_rx * IDLE_ROLL_SCALE;

	// Height from triggers (L2 = crouch, R2 = extend)
	float height_mod = 0.0f;
	if (filter_l2 > 0.01f) {
		height_mod -= filter_l2 * (LEG_MID_Z - LEG_MIN_Z);
	}
	if (filter_r2 > 0.01f) {
		height_mod += filter_r2 * (LEG_MAX_Z - LEG_MID_Z);
	}

	for (int i = 0; i < NUM_LEGS; i++) {
		float z = LEG_MID_Z + height_mod;
		if (legs[i].is_front) z += pitch_offset;
		else z -= pitch_offset;
		if (legs[i].is_left) z += roll_offset;
		else z -= roll_offset;
		moveLegIfNeeded(legs[i], z, leg_common_y, 20, false);
	}

	servos.startMove();
#endif
}

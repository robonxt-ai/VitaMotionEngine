/*
 * VitaMotion Engine - Main Controller
 *
 * Created by robonxt
 * Assisted by AI
 * Last modified: 2025-09-23
 *
 * Changelog:
 * 2025-06-21: Initial commit
 * 2025-06-27: Integrated Python-style keyframe trot gait (by AI)
 * 2025-06-29: updated to use v0.03 of HiBusServo library
 * 2025-07-01: Cleaned up code
 * 2025-07-02: Cleaned up more code, change to command pattern
 * 2025-07-03: V4, added display and button support
 * 2025-09-23: V5, cleanup and going to start working on WIFI or Bluetooth, hide some warnings (BTSerial warning and TFT_eSPI warning)
 *
 * TODO:
 * - [COMPLETED] Create basic movements and initial setup
 * - Switch to Raspberry Pi Pico W for wireless connectivity (Bluetooth, Wifi) or ESP32 (most likely will be using ESP32)
 * - Add Bluetooth or Wifi connectivity for remote control
 * - Add camera for object recognition and navigation
 * - Add obstacle avoidance using lidar or ultrasonic sensors
 * - Add AI-powered path planning and navigation
 * - Add AI-powered behavior generation
 */

#include <FABRIK2D.h>
#include <HiBusServo.h>

#define IS_DEBUGGING_ENABLED false
#define MOVE_ONLY_WHEN_NEEDED false // Set to true to move only when needed, false to always move
#define SERIAL_BAUDRATE 115200      // SERIAL = Serial communication

#define __UPLOADTIME__ __DATE__ " " __TIME__ // When was the program uploaded?
char *__SKETCHNAME__;                        // What is the name of the sketch?

//// For TFT stuff ////
#ifndef TOUCH_CS
#define TOUCH_CS -1
#endif
#include "robonxt_logos.h"
#include <TFT_eSPI.h>
#include "DroidPadInput.h"
TFT_eSPI tft = TFT_eSPI(135, 240);

//// For Button stuff ////
#include <Button2.h>
#define BUTTON_PIN_1 35
#define BUTTON_PIN_2 0
#define LONG_PRESS_TIME 1000
Button2 button_1;
Button2 button_2;

// ---- State Machine ----
enum class ControllerState
{
    STATE_IDLE,
    STATE_DISABLED,
    DEMO,
    ALTERNATING_DIAGONAL_STEP,
    WALKV1,
    WALKV2,
    TURNV1,
    TURN_IN_PLACE,
    JUMPING
};

ControllerState state = ControllerState::STATE_IDLE;

// ---- Leg Struct Definition ----
struct Leg
{
    int hip_id;
    int knee_id;
    int lengths[2];
    Fabrik2D fabrik;
    float hip_offset;
    float knee_offset;
    bool is_left;
    bool is_front;  // True for front legs (FL, FR), false for rear legs (RL, RR)
    float lastHipPos;
    float lastKneePos;

    Leg(int hip, int knee, int l1, int l2, float hipOff, float kneeOff, bool left, bool front)
        : hip_id(hip), knee_id(knee), lengths{l1, l2}, fabrik(3, lengths), hip_offset(hipOff), knee_offset(kneeOff),
          is_left(left), is_front(front), lastHipPos(0), lastKneePos(0)
    {
    }
};

// ---- Configuration ----
const int FL_HIP_ID = 1;
const int FL_KNEE_ID = 2;
const int FR_HIP_ID = 3;
const int FR_KNEE_ID = 4;
const int RL_HIP_ID = 5;
const int RL_KNEE_ID = 6;
const int RR_HIP_ID = 7;
const int RR_KNEE_ID = 8;

// Leg Indices in 'legs' array
const int LEG_FL = 0;
const int LEG_FR = 1;
const int LEG_RL = 2;
const int LEG_RR = 3;

const int THIGH_LENGTH = 100;
const int CALF_LENGTH = 100;

const float FL_HIP_OFFSET_LEFT = -0.48f;
const float FL_KNEE_OFFSET_LEFT = 41.76f;
const float FR_HIP_OFFSET_RIGHT = 1.44f;
const float FR_KNEE_OFFSET_RIGHT = -46.80f;
const float RL_HIP_OFFSET_LEFT = 0.96f;
const float RL_KNEE_OFFSET_LEFT = 42.48f;
const float RR_HIP_OFFSET_RIGHT = -0.24f;
const float RR_KNEE_OFFSET_RIGHT = -52.08f;

// Position limits (mm)
const float LEG_MIN_X = -50.0f;  // Maximum left movement (mm) (not implemented yet)
const float LEG_MAX_X = 50.0f;   // Maximum right movement (mm) (not implemented yet)
const float LEG_MIN_Y = -100.0f; // Maximum backward movement (mm)
const float LEG_MAX_Y = 100.0f;  // Maximum forward movement (mm)
const float LEG_MIN_Z = 55.0f;   // Minimum height (mm)
const float LEG_MID_Z = 140.0f;  // Medium height for standing (mm)
const float LEG_MAX_Z = 180.0f;  // Maximum height (mm)

// Default movement parameters
const unsigned long DEFAULT_MOVE_TIME = 1000;
const unsigned long DEFAULT_PAUSE_TIME = DEFAULT_MOVE_TIME + 100;

// Trot parameters
const unsigned long TROT_MOVE_TIME = 100;                      // Time for each movement phase (ms)
const unsigned long TROT_MOVE_DURATION = TROT_MOVE_TIME + 100; // Time to pause between movements (ms)
const float TROT_LIFT_HEIGHT = 20.0f;                          // How high to lift leg (mm)
const float TROT_STANDING_HEIGHT = LEG_MID_Z;                  // Standing height (mm) - lower is higher

// WalkV1 Gait Parameters
struct GaitKeyframe
{
    float y; // Mapped 'x' (forward/backward)
    float z; // Mapped 'y' (height)
};

const float WALKV1_SPEED = 3.75f;          // Speed multiplier (lower is slower)
const long WALKV1_UPDATE_INTERVAL_MS = 50; // How often to update the leg positions for smooth motion (lower is faster)
const float WALKV1_BODY_Y_OFFSET = 0.0f;   // Tunable value for stability
const float WALKV1_STRIDE = 30.0;          // originally 10.0
const float WALKV1_PUSH_BACK = 20.0;       // originally 3.5
const float WALKV1_LEG_DOWN_HEIGHT = 150;  // originally 150
const float WALKV1_LEG_UP_HEIGHT = WALKV1_LEG_DOWN_HEIGHT - 12; // originally 120

// Global stance tuning (mm)
// GLOBAL_STANCE_SPREAD_Y moves front feet forward and rear feet backward to widen the support polygon
float GLOBAL_STANCE_SPREAD_Y = 15.0f;  // Positive = Forward

// Runtime speed multiplier for walking/turning (1.0 = default). Adjusted via controller buttons/sliders.
float GLOBAL_SPEED_MULTIPLIER = 1.0f;

// Global COG compensation (mm) - shifts all feet forward to move body backward relative to feet
// Positive value moves feet forward, shifting body backward over back-heavy COG
const float GLOBAL_COG_Y_OFFSET = 15.0f;

const GaitKeyframe WALKV1_GAIT_SEQUENCE[] = {
    {-WALKV1_STRIDE, WALKV1_LEG_DOWN_HEIGHT},   // [0] Foot back (Negative = Backward)
    {-WALKV1_STRIDE, WALKV1_LEG_UP_HEIGHT},     // [1] Foot back
    {WALKV1_STRIDE, WALKV1_LEG_UP_HEIGHT},      // [2] Foot forward (Positive = Forward)
    {WALKV1_STRIDE, WALKV1_LEG_DOWN_HEIGHT},    // [3] Foot forward
    {WALKV1_PUSH_BACK, WALKV1_LEG_DOWN_HEIGHT}, // [4] Pushing back
    {-WALKV1_PUSH_BACK, WALKV1_LEG_DOWN_HEIGHT} // [5] Pushing back
};
const int WALKV1_GAIT_NUM_KEYFRAMES = sizeof(WALKV1_GAIT_SEQUENCE) / sizeof(WALKV1_GAIT_SEQUENCE[0]);

// WalkV2 Gait Parameters (with software compliance for smooth motion)
const float WALKV2_SPEED = 3.75f;
const long WALKV2_UPDATE_INTERVAL_MS = 50;
const float WALKV2_STRIDE = 30.0;
const float WALKV2_PUSH_BACK = 20.0;
const float WALKV2_LEG_DOWN_HEIGHT = 150;
const float WALKV2_LEG_UP_HEIGHT = 140; // 10mm lift
// Intermediate heights for compliance (smooth transitions)
const float WALKV2_LEG_MID1_HEIGHT = 147; // 3mm step
const float WALKV2_LEG_MID2_HEIGHT = 144; // 3mm step
const GaitKeyframe WALKV2_GAIT_SEQUENCE[] = {
    {-WALKV2_STRIDE, WALKV2_LEG_DOWN_HEIGHT},   // [0] Foot back
    {-WALKV2_STRIDE, WALKV2_LEG_MID1_HEIGHT},   // [1] Starting to lift
    {-WALKV2_STRIDE, WALKV2_LEG_MID2_HEIGHT},   // [2] Lifting more
    {-WALKV2_STRIDE, WALKV2_LEG_UP_HEIGHT},     // [3] Foot back, fully lifted
    {WALKV2_STRIDE, WALKV2_LEG_UP_HEIGHT},      // [4] Foot forward
    {WALKV2_STRIDE, WALKV2_LEG_MID2_HEIGHT},    // [5] Starting to lower
    {WALKV2_STRIDE, WALKV2_LEG_MID1_HEIGHT},    // [6] Lowering more
    {WALKV2_STRIDE, WALKV2_LEG_DOWN_HEIGHT},    // [7] Foot forward
    {WALKV2_PUSH_BACK, WALKV2_LEG_DOWN_HEIGHT}, // [8] Pushing back
    {-WALKV2_PUSH_BACK, WALKV2_LEG_DOWN_HEIGHT} // [9] Pushing back
};
const int WALKV2_GAIT_NUM_KEYFRAMES = sizeof(WALKV2_GAIT_SEQUENCE) / sizeof(WALKV2_GAIT_SEQUENCE[0]);

// Turn-In-Place Gait Parameters (Stationary Pivot)
const float TURN_Y_OFFSET = 10.0f;  // Minimal Y motion for stability
const float TURN_SPEED_SCALE = 0.8f; // Slower than walking for stability
const float TURN_LEG_DOWN_HEIGHT = 150; // Same as walk
const float TURN_LEG_UP_HEIGHT = 140;   // Only 10mm lift for stability
const GaitKeyframe TURN_IN_PLACE_SEQUENCE[] = {
    {-TURN_Y_OFFSET, TURN_LEG_DOWN_HEIGHT},  // Planted, slight back
    {-TURN_Y_OFFSET, TURN_LEG_UP_HEIGHT},    // Lifted, slight back
    {TURN_Y_OFFSET, TURN_LEG_UP_HEIGHT},     // Lifted, slight forward
    {TURN_Y_OFFSET, TURN_LEG_DOWN_HEIGHT},   // Planted, slight forward
};
const int TURN_IN_PLACE_NUM_KEYFRAMES = sizeof(TURN_IN_PLACE_SEQUENCE) / sizeof(TURN_IN_PLACE_SEQUENCE[0]);

struct WalkV1State
{
    unsigned long start_time = 0;
    unsigned long last_update_time = 0;
    float forward_factor = 1.0f; // Scales the forward motion. 1.0 = normal, -1.0 = backward, 0.0 = in place
};
WalkV1State walkv1_state;
WalkV1State walkv2_state; // Reuse same struct

struct GaitTurnState
{
    unsigned long start_time = 0;
    unsigned long last_update_time = 0;
    int direction = 1; // 1 = right, -1 = left
};
GaitTurnState turnv1_state;
GaitTurnState turn_in_place_state; // Reuse same struct since data needs are identical

enum class JumpPhase
{
    JUMP_START,
    JUMP_EXTEND,
    JUMP_RESET,
    JUMP_FINISH
};

struct JumpState
{
    JumpPhase phase = JumpPhase::JUMP_START;
    unsigned long phase_start_time = 0;
};
JumpState jump_state;

HiBusServo servos(Serial2);

Leg legs[] = {{FL_HIP_ID, FL_KNEE_ID, THIGH_LENGTH, CALF_LENGTH, FL_HIP_OFFSET_LEFT, FL_KNEE_OFFSET_LEFT, true, true},
              {FR_HIP_ID, FR_KNEE_ID, THIGH_LENGTH, CALF_LENGTH, FR_HIP_OFFSET_RIGHT, FR_KNEE_OFFSET_RIGHT, false, true},
              {RL_HIP_ID, RL_KNEE_ID, THIGH_LENGTH, CALF_LENGTH, RL_HIP_OFFSET_LEFT, RL_KNEE_OFFSET_LEFT, true, false},
              {RR_HIP_ID, RR_KNEE_ID, THIGH_LENGTH, CALF_LENGTH, RR_HIP_OFFSET_RIGHT, RR_KNEE_OFFSET_RIGHT, false, false}};
const int NUM_LEGS = sizeof(legs) / sizeof(legs[0]);

/*  ------------------------------------------------------------------------------------------------------  */
void tftMsg(String msg)
{
    tft.setTextColor(TFT_RED, TFT_BLACK, true);
    tft.setTextSize(1);
    tft.setTextDatum(TR_DATUM);
    tft.setTextWrap(true);
    tft.setCursor(0, 70);
    tft.print(F("                                                  "));
    tft.setCursor(0, 70);
    tft.println(msg);
    Serial.println(msg);
}

void about_program()
{
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.pushImage(0, 5, 240, 64, robonxt_logo_thin);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(TR_DATUM);
    tft.setTextWrap(true);
    tft.setCursor(0, tft.height() - 25);
    tft.println(F("DEV: robonxt"));
    tft.println(F("VER: " __UPLOADTIME__));
    tft.print(F("PRG: "));
    tft.println(F(__SKETCHNAME__));

    for (int i = 0; i < 10; i++)
    {
        Serial.println();
    }
    Serial.println(F("DEV: robonxt"));
    Serial.println(F("VER: " __UPLOADTIME__));
    Serial.print(F("PRG: "));
    Serial.println(F(__SKETCHNAME__));
}

void handler(Button2 &btn)
{
    if (btn == button_1)
    {
        if (state == ControllerState::STATE_IDLE)
        {
            enter_walkv1_mode(1.0f);
        }
        else
        {
            enter_idle_mode();
        }
    }
    else if (btn == button_2)
    {
        if (state != ControllerState::TURNV1)
        {
            enter_turn_mode(1);
        }
        else
        {
            enter_turn_mode(-1);
        }
    }
    else
    {
        ; // this is a catch all just in case
    }
}

void longhandler(Button2 &btn)
{
    unsigned int timePressed = btn.wasPressedFor();
    if (btn == button_1 && timePressed > LONG_PRESS_TIME)
    {
        disable_all_servos();
    }
    else if (btn == button_2 && timePressed > LONG_PRESS_TIME)
    {
        enable_all_servos();
    }
    else
    {
        ; // this is a catch all just in case
    }
}

void doublehandler(Button2 &btn)
{
    if (btn == button_1)
    {
        enter_demo_mode();
    }
    else if (btn == button_2)
    {
        enter_alternating_diagonal_step_mode();
    }
    else
    {
        ; // this is a catch all just in case
    }
}

void display_setup()
{
    // init tft!
    tft.init();
    about_program();
}

void button_setup()
{
    button_1.begin(BUTTON_PIN_1, INPUT);
    button_1.setClickHandler(handler);
    button_1.setLongClickHandler(longhandler);
    button_1.setDoubleClickHandler(doublehandler);

    button_2.begin(BUTTON_PIN_2);
    button_2.setClickHandler(handler);
    button_2.setLongClickHandler(longhandler);
    button_2.setDoubleClickHandler(doublehandler);
}

void button_loop()
{
    button_1.loop();
    button_2.loop();
}

// === Linear Interpolation (lerp) function ===
// Helper function to interpolate between two values.
// This is the core of the smooth keyframe animation.
// ESP32 has this function already, other platforms need this.
#ifndef ESP32
float lerp(float a, float b, float t)
{
    return a + t * (b - a);
}
#endif

// Returns true if the servo was actually moved
bool move_leg_if_needed(Leg &leg, float targetZ, float targetY, unsigned long duration = DEFAULT_MOVE_TIME)
{
    // Apply global COG compensation
    float adjusted_Y = targetY + GLOBAL_COG_Y_OFFSET;
    
    // Apply stance spread: front legs move forward, rear legs move backward
    if (leg.is_front) {
        adjusted_Y += GLOBAL_STANCE_SPREAD_Y;  // Positive = forward
    } else {
        adjusted_Y -= GLOBAL_STANCE_SPREAD_Y;  // Negative = backward
    }
    
    // INVERT Y FOR HARDWARE: The hardware uses Negative=Forward. 
    // We invert it here so everything above uses Positive=Forward.
    float hardware_Y = -adjusted_Y;

    // Calculate IK
    leg.fabrik.setTolerance(0.5);
    bool solved = leg.fabrik.solve(targetZ, hardware_Y, leg.lengths);
    if (!solved)
        return false;

    // Get raw angles from IK solver
    float hipAngle = leg.fabrik.getAngle(0) * RAD_TO_DEG;
    float kneeAngle = leg.fabrik.getAngle(1) * RAD_TO_DEG;

    // Ensure knee bends forward (negative angle from IK solver)
    if (kneeAngle > 0)
    {
        kneeAngle = -kneeAngle;
        hipAngle = -hipAngle; // Flip hip angle to maintain valid IK solution
    }

    // Calculate servo angles
    float servoHip, servoKnee;
    if (leg.is_left)
    {
        servoHip = -hipAngle + leg.hip_offset;
        servoKnee = kneeAngle + leg.knee_offset;
    }
    else
    {
        servoHip = hipAngle + leg.hip_offset;
        servoKnee = -kneeAngle + leg.knee_offset;
    }

    // Constrain angles to servo limits
    servoHip = constrain(servoHip, -120.0f, 120.0f);
    servoKnee = constrain(servoKnee, -120.0f, 120.0f);

    bool moved = false;
    if (MOVE_ONLY_WHEN_NEEDED)
    {
        // Only move if the change is significant
        if (abs(servoHip - leg.lastHipPos) >= 1.0f || abs(servoKnee - leg.lastKneePos) >= 1.0f)
        {
            servos.moveTo(leg.hip_id, servoHip, duration);
            servos.moveTo(leg.knee_id, servoKnee, duration);
            leg.lastHipPos = servoHip;
            leg.lastKneePos = servoKnee;
            moved = true;
        }
    }
    else
    {
        servos.moveTo(leg.hip_id, servoHip, duration);
        servos.moveTo(leg.knee_id, servoKnee, duration);
        leg.lastHipPos = servoHip;
        leg.lastKneePos = servoKnee;
        moved = true;
    }

    return moved;
}

// Move all legs to specified position (z = height, y = forward/back)
void move_all_legs(float targetZ, float targetY, unsigned long duration = DEFAULT_MOVE_TIME)
{
    for (int i = 0; i < NUM_LEGS; i++)
    {
        move_leg_if_needed(legs[i], targetZ, targetY, duration);
    }
}

void show_help()
{
    Serial.println("\n=== VitaMotion Engine - Command Reference ===\n");
    Serial.println("Basic Commands:");
    Serial.println("\thelp, ?    - Display this help message");
    Serial.println("\tstatus     - Show state, servo positions, voltages");
    Serial.println("\tstop       - Stop all current movements");
    Serial.println("\tdisable    - Disable all servos");
    Serial.println("\tenable     - Enable all servos\n");
    Serial.println("Movement Commands:");
    Serial.println("\tdemo       - Enter demo mode");
    Serial.println("\tup         - Stand up (medium height)");
    Serial.println("\tdown       - Lay down (min height)");
    Serial.println("\tforward    - Rock forward");
    Serial.println("\tback       - Rock backward");
    Serial.println("\tads        - Start alternating diagonal step");
    Serial.println("\twalkv1 [f] - Walk (f: -1.0 to 1.0, default 1.0)");
    Serial.println("\tturn left/right   - Turn left/right");
    Serial.println("\tjump       - Perform jump sequence\n");
    Serial.println("================================================\n");
}

// ---- Command Parser ----
String serial_buffer;
String bt_buffer;

// Helper to convert ControllerState to string
const char *controllerStateToString(ControllerState s)
{
    switch (s)
    {
    case ControllerState::STATE_IDLE:
        return "IDLE";
    case ControllerState::DEMO:
        return "DEMO";
    case ControllerState::ALTERNATING_DIAGONAL_STEP:
        return "ALTERNATING_DIAGONAL_STEP";
    case ControllerState::WALKV1:
        return "WALKV1";
    case ControllerState::WALKV2:
        return "WALKV2";
    case ControllerState::TURNV1:
        return "TURNV1";
    case ControllerState::TURN_IN_PLACE:
        return "TURN_IN_PLACE";
    case ControllerState::JUMPING:
        return "JUMPING";
    case ControllerState::STATE_DISABLED:
        return "DISABLED";
    default:
        return "UNKNOWN";
    }
}

// Command handler function type
typedef void (*CommandHandler)(const String &);

// Command structure
struct Command
{
    const char *name;
    CommandHandler handler;
    bool requiresArgs;
};

// Command handlers
void handleDemo(const String &)
{
    enter_demo_mode();
}
void handleStop(const String &)
{
    enter_idle_mode();
}
void handleDisable(const String &)
{
    disable_all_servos();
}
void handleEnable(const String &)
{
    enable_all_servos();
}
void handleHelp(const String &)
{
    show_help();
}
void handleDown(const String &)
{
    Serial.println("Laying down...");
    move_all_legs(LEG_MIN_Z, 0);
}
void handleUp(const String &)
{
    Serial.println("Getting up...");
    move_all_legs(LEG_MID_Z, 0);
}
void handleForward(const String &)
{
    Serial.println("Leaning forward...");
    move_all_legs(LEG_MID_Z, LEG_MAX_Y);
}
void handleBack(const String &)
{
    Serial.println("Leaning backward...");
    move_all_legs(LEG_MID_Z, LEG_MIN_Y);
}
void handleADS(const String &)
{
    enter_alternating_diagonal_step_mode();
}
void handleJump(const String &)
{
    state = ControllerState::JUMPING;
    jump_state.phase = JumpPhase::JUMP_START;
    Serial.println("Entering JUMPING state.");
}
void handleWalkV1(const String &args)
{
    float factor = 1.0;
    if (args.length() > 0)
    {
        factor = args.toFloat();
        factor = constrain(factor, -1.0, 1.0);
    }
    enter_walkv1_mode(factor);
}
void handleWalkV2(const String &args)
{
    float factor = 1.0;
    if (args.length() > 0)
    {
        factor = args.toFloat();
        factor = constrain(factor, -1.0, 1.0);
    }
    enter_walkv2_mode(factor);
}
void handleTurn(const String &args)
{
    if (args == "left")
    {
        enter_turn_mode(-1);
    }
    else if (args == "right")
    {
        enter_turn_mode(1);
    }
    else
    {
        Serial.println("Usage: turn <left|right>");
    }
}
void handleTurnInPlace(const String &args)
{
    if (args == "left")
    {
        enter_turn_in_place_mode(-1);
    }
    else if (args == "right")
    {
        enter_turn_in_place_mode(1);
    }
    else
    {
        Serial.println("Usage: spin <left|right>");
    }
}
void handleStatus(const String &)
{
    Serial.print("[Status] Mode: ");
    Serial.println(controllerStateToString(state));
    Serial.println("Servo positions and voltages:");
    for (int id = 1; id <= 8; ++id)
    {
        int16_t pos_raw = servos.getPosition(id);
        int16_t vin_raw = servos.getVoltage(id);
        int8_t temp = servos.getTemperature(id);
        bool is_on = servos.isMotorOn(id);
        float vinV = vin_raw / 1000.0f;

        Serial.print("  Servo ");
        Serial.print(id);
        Serial.print(": ");
        Serial.print(is_on ? "ENABLED " : "DISABLED");
        Serial.print(", pos=");
        Serial.print(pos_raw != -1 ? String(pos_raw) : "ERROR");
        Serial.print(", vin=");
        Serial.print(vin_raw != -1 ? String(vinV, 2) : "ERROR");
        Serial.print(", temp=");
        Serial.print(temp != -1 ? String(temp) : "ERROR");
        Serial.println();
        delay(3);
    }
}

// Command table
const Command commands[] = {
    {"demo", handleDemo, false},     {"stop", handleStop, false}, {"disable", handleDisable, false},
    {"enable", handleEnable, false}, {"help", handleHelp, false}, {"?", handleHelp, false},
    {"down", handleDown, false},     {"up", handleUp, false},     {"forward", handleForward, false},
    {"back", handleBack, false},     {"ads", handleADS, false},   {"jump", handleJump, false},
    {"walkv1", handleWalkV1, true},  {"walkv2", handleWalkV2, true}, {"turn", handleTurn, true},  {"spin", handleTurnInPlace, true},
    {"status", handleStatus, false}};

const int NUM_COMMANDS = sizeof(commands) / sizeof(Command);

void parse_command(const String &cmd)
{
    String command = cmd;
    command.trim();
    command.toLowerCase();
    Serial.println("Received: " + command);

    // Delegate to modular DroidPad parser first
    if (dp_tryHandleCommand(command))
    {
        return;
    }

    // All DroidPad input handled in module

    // Split command and arguments
    int space_pos = command.indexOf(' ');
    String cmd_name = space_pos == -1 ? command : command.substring(0, space_pos);
    String args = space_pos == -1 ? "" : command.substring(space_pos + 1);

    // Find and execute command
    bool found = false;
    for (int i = 0; i < NUM_COMMANDS; i++)
    {
        if (cmd_name == commands[i].name)
        {
            found = true;
            if (commands[i].requiresArgs && args.length() == 0)
            {
                Serial.print("Usage: ");
                Serial.print(commands[i].name);
                Serial.println(" <args>");
            }
            else
            {
                commands[i].handler(args);
            }
            break;
        }
    }

    if (!found)
    {
        Serial.println("Unknown command: " + command);
        Serial.println("Type 'help' for available commands.");
    }
}

void enter_walkv1_mode(float forward_factor)
{
    tftMsg("[Controller] Entering WALKV1 mode with forward factor: " + String(forward_factor, 2));
    walkv1_state.start_time = millis();
    walkv1_state.last_update_time = walkv1_state.start_time;
    walkv1_state.forward_factor = forward_factor;
    state = ControllerState::WALKV1;
}

void enter_walkv2_mode(float forward_factor)
{
    tftMsg("[Controller] Entering WALKV2 mode (with compliance) - forward factor: " + String(forward_factor, 2));
    walkv2_state.start_time = millis();
    walkv2_state.last_update_time = walkv2_state.start_time;
    walkv2_state.forward_factor = forward_factor;
    state = ControllerState::WALKV2;
}

void enter_turn_mode(int direction)
{
    tftMsg("[Controller] Entering TURNV1 mode: " + String(direction == 1 ? "RIGHT" : "LEFT"));
    turnv1_state.start_time = millis();
    turnv1_state.last_update_time = turnv1_state.start_time;
    turnv1_state.direction = direction;
    state = ControllerState::TURNV1;
}

void enter_turn_in_place_mode(int direction)
{
    tftMsg("[Controller] Entering TURN_IN_PLACE mode: " + String(direction == 1 ? "RIGHT" : "LEFT"));
    turn_in_place_state.start_time = millis();
    turn_in_place_state.last_update_time = turn_in_place_state.start_time;
    turn_in_place_state.direction = direction;
    state = ControllerState::TURN_IN_PLACE;
}

void enter_alternating_diagonal_step_mode()
{
    tftMsg("[Controller] Entering ALTERNATING_DIAGONAL_STEP mode");
    state = ControllerState::ALTERNATING_DIAGONAL_STEP;
}

void enter_demo_mode()
{
    tftMsg("[Controller] Entering DEMO mode");
    state = ControllerState::DEMO;
}

void enter_idle_mode()
{
    tftMsg("[Controller] Entering STATE_IDLE mode. Halting all movement.");
    state = ControllerState::STATE_IDLE;
}

void disable_all_servos()
{
    tftMsg("[Controller] Disabling all servos");

    for (int i = 0; i < NUM_LEGS; ++i)
    {
        servos.motorOff(legs[i].hip_id);
        delay(1);
        servos.motorOff(legs[i].knee_id);
        delay(1);
    }

    state = ControllerState::STATE_DISABLED;
}

void soft_start_servos()
{
    tftMsg("[Controller] Soft-starting servos...");
    move_all_legs(LEG_MID_Z, 0, 2000); // Take 2 seconds to stand up
    delay(2000); // Wait for it to finish
}

void update_demo()
{
    // Immediately exit if state has changed
    if (state != ControllerState::DEMO) return;

    static bool toggle = false;
    static unsigned long lastMoveTime = 0;
    static float pose1Z = LEG_MID_Z, pose1Y = 0;
    static float pose2Z = LEG_MIN_Z, pose2Y = 0;
    if (millis() - lastMoveTime > DEFAULT_PAUSE_TIME + 1000)
    {
        for (int i = 0; i < NUM_LEGS; ++i)
        {
            move_leg_if_needed(legs[i], toggle ? pose1Z : pose2Z, toggle ? pose1Y : pose2Y);
        }
        toggle = !toggle;
        lastMoveTime = millis();
    }
}

void update_jump()
{
    if (state != ControllerState::JUMPING) return;
    
    unsigned long current_time = millis();

    switch (jump_state.phase)
    {
    case JumpPhase::JUMP_START:
        tftMsg("Jumping: crouch...");
        move_all_legs(100, 40, 300); // crouch
        jump_state.phase = JumpPhase::JUMP_EXTEND;
        jump_state.phase_start_time = current_time;
        break;

    case JumpPhase::JUMP_EXTEND:
        // Wait for crouch (200ms) + pause (100ms)
        if (current_time - jump_state.phase_start_time >= 400)
        {
            tftMsg("Jumping: Jump!");
            move_all_legs(LEG_MAX_Z, 50, 100); // jump!
            jump_state.phase = JumpPhase::JUMP_RESET;
            jump_state.phase_start_time = current_time;
        }
        break;

    case JumpPhase::JUMP_RESET:
        // Wait for jump (100ms) + cooldown (50ms)
        if (current_time - jump_state.phase_start_time >= 600)
        {
            tftMsg("Jumping: Resetting to stand.");
            move_all_legs(LEG_MID_Z, 0, 100);
            jump_state.phase = JumpPhase::JUMP_FINISH;
            jump_state.phase_start_time = current_time;
        }
        break;

    case JumpPhase::JUMP_FINISH:
        // Wait for reset to stand (500ms)
        if (current_time - jump_state.phase_start_time >= 600)
        {
            tftMsg("Jump complete. Returning to STATE_IDLE.");
            state = ControllerState::STATE_IDLE;
        }
        break;
    }
}

void update_walkv1()
{
    if (state != ControllerState::WALKV1) return;

    unsigned long current_time = millis();

    // Update at 20Hz (50ms)
    if (current_time - walkv1_state.last_update_time < WALKV1_UPDATE_INTERVAL_MS)
    {
        return;
    }
    walkv1_state.last_update_time = current_time;

    float time_since_start_sec = (current_time - walkv1_state.start_time) / 1000.0f;
    float elapsed = time_since_start_sec * WALKV1_SPEED * GLOBAL_SPEED_MULTIPLIER;

    int index1 = floor(elapsed);
    float ratio = elapsed - index1;

    // --- Calculate and move Pair 1 (FR, RL) ---
    int pair1_idx1 = index1 % WALKV1_GAIT_NUM_KEYFRAMES;
    int pair1_idx2 = (pair1_idx1 + 1) % WALKV1_GAIT_NUM_KEYFRAMES;

    float p1_y = lerp(WALKV1_GAIT_SEQUENCE[pair1_idx1].y, WALKV1_GAIT_SEQUENCE[pair1_idx2].y, ratio);
    float p1_z = lerp(WALKV1_GAIT_SEQUENCE[pair1_idx1].z, WALKV1_GAIT_SEQUENCE[pair1_idx2].z, ratio);
    p1_y *= walkv1_state.forward_factor;

    float final_p1_y = p1_y + WALKV1_BODY_Y_OFFSET;

    move_leg_if_needed(legs[LEG_FR], p1_z, final_p1_y, WALKV1_UPDATE_INTERVAL_MS); // FR
    move_leg_if_needed(legs[LEG_RL], p1_z, final_p1_y, WALKV1_UPDATE_INTERVAL_MS); // RL

    // --- Calculate and move Pair 2 (FL, RR) ---
    int pair2_idx1 = (index1 + (WALKV1_GAIT_NUM_KEYFRAMES / 2)) % WALKV1_GAIT_NUM_KEYFRAMES;
    int pair2_idx2 = (pair2_idx1 + 1) % WALKV1_GAIT_NUM_KEYFRAMES;

    float p2_y = lerp(WALKV1_GAIT_SEQUENCE[pair2_idx1].y, WALKV1_GAIT_SEQUENCE[pair2_idx2].y, ratio);
    float p2_z = lerp(WALKV1_GAIT_SEQUENCE[pair2_idx1].z, WALKV1_GAIT_SEQUENCE[pair2_idx2].z, ratio);
    p2_y *= walkv1_state.forward_factor;

    float final_p2_y = p2_y + WALKV1_BODY_Y_OFFSET;

    move_leg_if_needed(legs[LEG_FL], p2_z, final_p2_y, WALKV1_UPDATE_INTERVAL_MS); // FL
    move_leg_if_needed(legs[LEG_RR], p2_z, final_p2_y, WALKV1_UPDATE_INTERVAL_MS); // RR
}

void update_walkv2()
{
    if (state != ControllerState::WALKV2)
        return;

    unsigned long current_time = millis();

    // Update at 20Hz (50ms)
    if (current_time - walkv2_state.last_update_time < WALKV2_UPDATE_INTERVAL_MS)
    {
        return;
    }
    walkv2_state.last_update_time = current_time;

    float time_since_start_sec = (current_time - walkv2_state.start_time) / 1000.0f;
    float elapsed = time_since_start_sec * WALKV2_SPEED * GLOBAL_SPEED_MULTIPLIER;

    int index1 = floor(elapsed);
    float ratio = elapsed - index1;

    // --- Calculate and move Pair 1 (FR, RL) ---
    int pair1_idx1 = index1 % WALKV2_GAIT_NUM_KEYFRAMES;
    int pair1_idx2 = (pair1_idx1 + 1) % WALKV2_GAIT_NUM_KEYFRAMES;

    float p1_y = lerp(WALKV2_GAIT_SEQUENCE[pair1_idx1].y, WALKV2_GAIT_SEQUENCE[pair1_idx2].y, ratio);
    float p1_z = lerp(WALKV2_GAIT_SEQUENCE[pair1_idx1].z, WALKV2_GAIT_SEQUENCE[pair1_idx2].z, ratio);

    p1_y *= walkv2_state.forward_factor;

    float final_p1_y = p1_y + WALKV1_BODY_Y_OFFSET;

    move_leg_if_needed(legs[LEG_FR], p1_z, final_p1_y, WALKV2_UPDATE_INTERVAL_MS); // FR
    move_leg_if_needed(legs[LEG_RL], p1_z, final_p1_y, WALKV2_UPDATE_INTERVAL_MS); // RL

    // --- Calculate and move Pair 2 (FL, RR) ---
    int pair2_idx1 = (index1 + (WALKV2_GAIT_NUM_KEYFRAMES / 2)) % WALKV2_GAIT_NUM_KEYFRAMES;
    int pair2_idx2 = (pair2_idx1 + 1) % WALKV2_GAIT_NUM_KEYFRAMES;

    float p2_y = lerp(WALKV2_GAIT_SEQUENCE[pair2_idx1].y, WALKV2_GAIT_SEQUENCE[pair2_idx2].y, ratio);
    float p2_z = lerp(WALKV2_GAIT_SEQUENCE[pair2_idx1].z, WALKV2_GAIT_SEQUENCE[pair2_idx2].z, ratio);

    p2_y *= walkv2_state.forward_factor;

    float final_p2_y = p2_y + WALKV1_BODY_Y_OFFSET;

    move_leg_if_needed(legs[LEG_FL], p2_z, final_p2_y, WALKV2_UPDATE_INTERVAL_MS); // FL
    move_leg_if_needed(legs[LEG_RR], p2_z, final_p2_y, WALKV2_UPDATE_INTERVAL_MS); // RR
}

void update_turnv1()
{
    if (state != ControllerState::TURNV1) return;

    unsigned long current_time = millis();
    if (current_time - turnv1_state.last_update_time < WALKV1_UPDATE_INTERVAL_MS)
    {
        return;
    }
    turnv1_state.last_update_time = current_time;

    float time_since_start_sec = (current_time - turnv1_state.start_time) / 1000.0f;
    float elapsed = time_since_start_sec * WALKV1_SPEED * GLOBAL_SPEED_MULTIPLIER;

    int index1 = floor(elapsed);
    float ratio = elapsed - index1;

    // Base indices into gait keyframes
    int pair1_idx1 = index1 % WALKV1_GAIT_NUM_KEYFRAMES;
    int pair1_idx2 = (pair1_idx1 + 1) % WALKV1_GAIT_NUM_KEYFRAMES;

    float base_y1 = lerp(WALKV1_GAIT_SEQUENCE[pair1_idx1].y, WALKV1_GAIT_SEQUENCE[pair1_idx2].y, ratio);
    float base_z1 = lerp(WALKV1_GAIT_SEQUENCE[pair1_idx1].z, WALKV1_GAIT_SEQUENCE[pair1_idx2].z, ratio);

    int pair2_idx1 = (index1 + (WALKV1_GAIT_NUM_KEYFRAMES / 2)) % WALKV1_GAIT_NUM_KEYFRAMES;
    int pair2_idx2 = (pair2_idx1 + 1) % WALKV1_GAIT_NUM_KEYFRAMES;

    float base_y2 = lerp(WALKV1_GAIT_SEQUENCE[pair2_idx1].y, WALKV1_GAIT_SEQUENCE[pair2_idx2].y, ratio);
    float base_z2 = lerp(WALKV1_GAIT_SEQUENCE[pair2_idx1].z, WALKV1_GAIT_SEQUENCE[pair2_idx2].z, ratio);

    const float INNER_SCALE = 0.3f;                                          
    float scale_left = (turnv1_state.direction == -1) ? INNER_SCALE : 1.0f;  
    float scale_right = (turnv1_state.direction == -1) ? 1.0f : INNER_SCALE; 

    // Pair1: FR (1) right, RL (2) left
    float final_y_FR = -(base_y1 * scale_right) + WALKV1_BODY_Y_OFFSET;
    float final_y_RL = -(base_y1 * scale_left) + WALKV1_BODY_Y_OFFSET;

    move_leg_if_needed(legs[LEG_FR], base_z1, final_y_FR, WALKV1_UPDATE_INTERVAL_MS); // FR
    move_leg_if_needed(legs[LEG_RL], base_z1, final_y_RL, WALKV1_UPDATE_INTERVAL_MS); // RL

    // Pair2: FL (0) left, RR (3) right
    float final_y_FL = -(base_y2 * scale_left) + WALKV1_BODY_Y_OFFSET;
    float final_y_RR = -(base_y2 * scale_right) + WALKV1_BODY_Y_OFFSET;

    move_leg_if_needed(legs[LEG_FL], base_z2, final_y_FL, WALKV1_UPDATE_INTERVAL_MS);
    move_leg_if_needed(legs[LEG_RR], base_z2, final_y_RR, WALKV1_UPDATE_INTERVAL_MS);
}

void update_turn_in_place()
{
    if (state != ControllerState::TURN_IN_PLACE) return;

    unsigned long current_time = millis();
    if (current_time - turn_in_place_state.last_update_time < WALKV1_UPDATE_INTERVAL_MS)
    {
        return;
    }
    turn_in_place_state.last_update_time = current_time;

    float time_since_start_sec = (current_time - turn_in_place_state.start_time) / 1000.0f;
    float elapsed = time_since_start_sec * WALKV1_SPEED * GLOBAL_SPEED_MULTIPLIER * TURN_SPEED_SCALE;

    int index1 = floor(elapsed);
    float ratio = elapsed - index1;

    // Base indices into turn gait keyframes
    int pair1_idx1 = index1 % TURN_IN_PLACE_NUM_KEYFRAMES;
    int pair1_idx2 = (pair1_idx1 + 1) % TURN_IN_PLACE_NUM_KEYFRAMES;

    float base_y1 = lerp(TURN_IN_PLACE_SEQUENCE[pair1_idx1].y, TURN_IN_PLACE_SEQUENCE[pair1_idx2].y, ratio);
    float base_z1 = lerp(TURN_IN_PLACE_SEQUENCE[pair1_idx1].z, TURN_IN_PLACE_SEQUENCE[pair1_idx2].z, ratio);

    int pair2_idx1 = (index1 + (TURN_IN_PLACE_NUM_KEYFRAMES / 2)) % TURN_IN_PLACE_NUM_KEYFRAMES;
    int pair2_idx2 = (pair2_idx1 + 1) % TURN_IN_PLACE_NUM_KEYFRAMES;

    float base_y2 = lerp(TURN_IN_PLACE_SEQUENCE[pair2_idx1].y, TURN_IN_PLACE_SEQUENCE[pair2_idx2].y, ratio);
    float base_z2 = lerp(TURN_IN_PLACE_SEQUENCE[pair2_idx1].z, TURN_IN_PLACE_SEQUENCE[pair2_idx2].z, ratio);

    // Apply direction: opposite Y motion for left vs right legs creates rotation
    // If turning RIGHT (1): Left legs Forward (Scale 1.0), Right legs Backward (Scale -1.0)
    // If turning LEFT (-1): Left legs Backward (Scale -1.0), Right legs Forward (Scale 1.0)
    // NOTE: Positive Y = Forward now.
    float scale_left = (turn_in_place_state.direction == -1) ? -1.0f : 1.0f;
    float scale_right = (turn_in_place_state.direction == -1) ? 1.0f : -1.0f;

    // Pair1: FR (right), RL (left)
    float final_y_FR = (base_y1 * scale_right);
    float final_y_RL = (base_y1 * scale_left);

    move_leg_if_needed(legs[LEG_FR], base_z1, final_y_FR, WALKV1_UPDATE_INTERVAL_MS);
    move_leg_if_needed(legs[LEG_RL], base_z1, final_y_RL, WALKV1_UPDATE_INTERVAL_MS);

    // Pair2: FL (left), RR (right)
    float final_y_FL = (base_y2 * scale_left);
    float final_y_RR = (base_y2 * scale_right);

    move_leg_if_needed(legs[LEG_FL], base_z2, final_y_FL, WALKV1_UPDATE_INTERVAL_MS);
    move_leg_if_needed(legs[LEG_RR], base_z2, final_y_RR, WALKV1_UPDATE_INTERVAL_MS);
}

void update_ads()
{
    if (state != ControllerState::ALTERNATING_DIAGONAL_STEP) return;

    static bool trot_toggle = false;
    static unsigned long lastTrotTime = 0;
    if (millis() - lastTrotTime > TROT_MOVE_DURATION)
    {
        for (int i = 0; i < NUM_LEGS; ++i)
        {
            bool up = ((i == LEG_FL || i == LEG_RR) ? trot_toggle : !trot_toggle);
            float z = up ? (TROT_STANDING_HEIGHT - TROT_LIFT_HEIGHT) : TROT_STANDING_HEIGHT;
            move_leg_if_needed(legs[i], z, 0, TROT_MOVE_TIME);
        }
        trot_toggle = !trot_toggle;
        lastTrotTime = millis();
    }
}

void enable_all_servos()
{
    tftMsg("[Controller] Enabling all servos");

    for (int i = 0; i < NUM_LEGS; ++i)
    {
        servos.motorOn(legs[i].hip_id);
        servos.motorOn(legs[i].knee_id);
    }

    state = ControllerState::STATE_IDLE;
    soft_start_servos();
}

// ---- Setup ----
void setup()
{
#ifdef ESP32
    Serial2.setPins(26, 27); // for ESP boards
#endif
    Serial.begin(SERIAL_BAUDRATE);
    servos.begin(SERIAL_BAUDRATE);
	
    // Set up SKETCHNAME!
    char a[] = __FILE__;
    byte b = sizeof(a);
    while ((b > 0) && (a[b] != '\\'))
        b--;
    __SKETCHNAME__ = &a[++b];
    ////////////////////////

    delay(200);
    tftMsg("\n=== Main Controller ===");
    show_help();
    display_setup();
    button_setup();

}

// ---- Main Loop ----
void loop()
{
    // Handle button inputs
    button_loop();

    // Check for serial input
    while (Serial.available() > 0)
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
            if (serial_buffer.length() > 0)
            {
                parse_command(serial_buffer);
                serial_buffer = "";
            }
        }
        else if (isPrintable(c))
        {
            serial_buffer += c;
        }
    }

#ifdef ESP32
    if (isBluetoothReady)
    {
        while (SerialBT.available() > 0)
        {
            char c = SerialBT.read();
            if (c == '\n' || c == '\r')
            {
                if (bt_buffer.length() > 0)
                {
                    parse_command(bt_buffer);
                    bt_buffer = "";
                }
            }
            else if (isPrintable(c))
            {
                bt_buffer += c;
            }
        }
    }
#endif
    switch (state)
    {
    case ControllerState::STATE_DISABLED:
    case ControllerState::STATE_IDLE:
        break;
    case ControllerState::DEMO:
        update_demo();
        break;
    case ControllerState::JUMPING:
        update_jump();
        break;
    case ControllerState::WALKV1:
        update_walkv1();
        break;
    case ControllerState::WALKV2:
        update_walkv2();
        break;
    case ControllerState::TURNV1:
        update_turnv1();
        break;
    case ControllerState::TURN_IN_PLACE:
        update_turn_in_place();
        break;
    case ControllerState::ALTERNATING_DIAGONAL_STEP:
        update_ads();
        break;
    }
}

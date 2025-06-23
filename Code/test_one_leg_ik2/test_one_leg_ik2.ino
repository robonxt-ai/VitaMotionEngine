#include <HiBusServo.h>
#include <FABRIK2D.h>

// Servo IDs (Right Leg)
const int HIP_SERVO_ID = 7;
const int KNEE_SERVO_ID = 8;

// Constants for leg dimensions (in mm)
const int THIGH_LENGTH = 100;  // Hip to knee length
const int CALF_LENGTH = 100;   // Knee to foot length

int lengths[] = { THIGH_LENGTH, CALF_LENGTH };  // 2DOF leg where hip to knee is THIGH_LENGTH and knee to foot is CALF_LENGTH

const int MOVE_TIME = 1000;              // Movement duration (ms)
const int PAUSE_TIME = MOVE_TIME + 10;  // Pause between moves (ms)

HiBusServo servos(true);
Fabrik2D fabrik2D(3, lengths);  // The 3 means the end effector "joint" (aka feet for us)


// Alternate between two positive (x, y) targets, as in the FABRIK2D example
float targetX1 = 180;
float targetY1 = 0;
float targetX2 = 65;
float targetY2 = 0;
bool toggleTarget = false;


void setup() {
  Serial.begin(115200);
  delay(1000);
  servos.begin(Serial2);
  delay(1000);
  fabrik2D.setTolerance(0.5);
  delay(1000);
  Serial.println("Ready!");
}

// --- Calibrated offsets for "straight" mechanical alignment (from calibration tool)
const float HIP_STRAIGHT_OFFSET_DEG = 3.84f;
const float KNEE_STRAIGHT_OFFSET_DEG = 41.04f;

void loop() {
  // Alternate between two positive (x, y) targets as in the FABRIK2D example
  float targetX = toggleTarget ? targetX1 : targetX2;
  float targetY = toggleTarget ? targetY1 : targetY2;

  // Print the target
  Serial.print("Target: (");
  Serial.print(targetX);
  Serial.print(", ");
  Serial.print(targetY);
  Serial.println(")");

  // Solve IK and print return value
  bool solved = fabrik2D.solve(targetX, targetY, lengths);
  Serial.print("solve() returned: ");
  Serial.println(solved ? "true" : "false");

  // Get angles in degrees
  float hipAngle = fabrik2D.getAngle(0) * RAD_TO_DEG;
  float kneeAngle = fabrik2D.getAngle(1) * RAD_TO_DEG;

  // Apply calibration offsets to match true mechanical "straight" alignment
  float transformedHipAngle = -hipAngle + HIP_STRAIGHT_OFFSET_DEG;
  float transformedKneeAngle = kneeAngle + KNEE_STRAIGHT_OFFSET_DEG;
  // Clamp to [-120, 120] degrees for HiBusServo
  float finalHipServoAngle = min(120.0f, max(-120.0f, transformedHipAngle));
  float finalKneeServoAngle = min(120.0f, max(-120.0f, transformedKneeAngle));
  // Debug: print the actual angles sent to the servos
  Serial.print("Final Hip Servo Angle: ");
  Serial.print(finalHipServoAngle);
  Serial.print(" | Final Knee Servo Angle: ");
  Serial.println(finalKneeServoAngle);
  // Move servos
  servos.moveTo(HIP_SERVO_ID, finalHipServoAngle, MOVE_TIME);
  servos.moveTo(KNEE_SERVO_ID, finalKneeServoAngle, MOVE_TIME);

  // Debug output
  Serial.print("Hip: ");
  Serial.print(hipAngle);
  Serial.print("° Knee: ");
  Serial.print(kneeAngle);
  Serial.print("° Position: (");
  Serial.print(fabrik2D.getX(2));
  Serial.print(", ");
  Serial.print(fabrik2D.getY(2));
  Serial.println(")");

  // Toggle target for next loop
  toggleTarget = !toggleTarget;
  delay(PAUSE_TIME);
  delay(2000);
}

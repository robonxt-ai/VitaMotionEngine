#include <HiBusServo.h>
#define SERIAL_BAUDRATE 115200      // SERIAL = Serial communication

HiBusServo servos(Serial2);

// Servo IDs (update if needed)
const int FL_HIP_ID = 1;
const int FL_KNEE_ID = 2;
const int FR_HIP_ID = 3;
const int FR_KNEE_ID = 4;
const int RL_HIP_ID = 5;
const int RL_KNEE_ID = 6;
const int RR_HIP_ID = 7;
const int RR_KNEE_ID = 8;

// Current leg offsets (update these to match your robot's current values!!!)
const float CURRENT_FL_HIP_OFFSET = 0.24f;
const float CURRENT_FL_KNEE_OFFSET = 41.04f;
const float CURRENT_FR_HIP_OFFSET = 0.48f;
const float CURRENT_FR_KNEE_OFFSET = -45.84f;
const float CURRENT_RL_HIP_OFFSET = 0.0f;
const float CURRENT_RL_KNEE_OFFSET = 37.92f;
const float CURRENT_RR_HIP_OFFSET = 0.0f;
const float CURRENT_RR_KNEE_OFFSET = -52.32f;

void setup() {
  delay(100);
  Serial2.setPins(26, 27);
  Serial.begin(SERIAL_BAUDRATE);
  servos.begin(SERIAL_BAUDRATE);
  delay(3000);

  // Enable torque on all servos so they can be read
  Serial.println("Enabling torque on all servos...");
  for (int id = 1; id <= 8; ++id)
  {
    servos.motorOn(id);
    delay(10); // Small delay between commands
  }
  Serial.println("Servos enabled.");

  Serial.println("\n=== Leg Center Calibration Tool ===");
  Serial.println("Move the hip and knee joints to their true mechanical 'straight' (centered) positions.");
  Serial.println("When ready, type any character and press Enter to read and print the current servo positions.");
  Serial.println("Repeat as needed. Use the reported values as offsets for your main IK program.");

  Serial.println("\nSystem ready.");
  delay(2000); // Add a final delay to allow servos to stabilize before the first command.
}

void handleStatus(const String &) {
  Serial.println("\n=== Servo Calibration Readout ===");
  
  // Read all positions first
  int16_t positions[8];
  int16_t voltages[8];
  int8_t temps[8];
  
  for (int id = 1; id <= 8; ++id) {
    positions[id-1] = servos.getPosition(id);
    voltages[id-1] = servos.getVoltage(id);
    temps[id-1] = servos.getTemperature(id);
    delay(5);
  }
  
  // Helper to print servo info
  auto printServo = [&](const char* label, int id, float currentOffset) {
    int16_t pos = positions[id-1];
    int16_t vin = voltages[id-1];
    int8_t temp = temps[id-1];
    
    if (pos == -1) {
      Serial.print("  ");
      Serial.print(label);
      Serial.println(": ERROR - Could not read position");
      return;
    }
    
    float offset_deg = (pos - 500) * 0.24f;
    float delta = offset_deg - currentOffset;
    
    Serial.print("  ");
    Serial.print(label);
    Serial.print(" (ID ");
    Serial.print(id);
    Serial.print("): ");
    Serial.print(offset_deg, 2);
    Serial.print("° (Δ");
    if (delta > 0) Serial.print("+");
    Serial.print(delta, 2);
    Serial.print("°) [pos=");
    Serial.print(pos);
    Serial.print(", vin=");
    Serial.print(vin / 1000.0f, 2);
    Serial.print("V, temp=");
    Serial.print(temp);
    Serial.println("°C]");
  };
  
  // Grouped by leg
  Serial.println("\n--- Front Left ---");
  printServo("Hip ", FL_HIP_ID, CURRENT_FL_HIP_OFFSET);
  printServo("Knee", FL_KNEE_ID, CURRENT_FL_KNEE_OFFSET);
  
  Serial.println("\n--- Front Right ---");
  printServo("Hip ", FR_HIP_ID, CURRENT_FR_HIP_OFFSET);
  printServo("Knee", FR_KNEE_ID, CURRENT_FR_KNEE_OFFSET);
  
  Serial.println("\n--- Rear Left ---");
  printServo("Hip ", RL_HIP_ID, CURRENT_RL_HIP_OFFSET);
  printServo("Knee", RL_KNEE_ID, CURRENT_RL_KNEE_OFFSET);
  
  Serial.println("\n--- Rear Right ---");
  printServo("Hip ", RR_HIP_ID, CURRENT_RR_HIP_OFFSET);
  printServo("Knee", RR_KNEE_ID, CURRENT_RR_KNEE_OFFSET);
  
  // Copy-paste ready code
  Serial.println("\n=== Copy-Paste Ready Offsets (verify variable names match your controller!) ===");
  Serial.print("const float FL_HIP_OFFSET_LEFT = ");
  Serial.print((positions[FL_HIP_ID-1] - 500) * 0.24f, 2);
  Serial.println("f;");
  
  Serial.print("const float FL_KNEE_OFFSET_LEFT = ");
  Serial.print((positions[FL_KNEE_ID-1] - 500) * 0.24f, 2);
  Serial.println("f;");
  
  Serial.print("const float FR_HIP_OFFSET_RIGHT = ");
  Serial.print((positions[FR_HIP_ID-1] - 500) * 0.24f, 2);
  Serial.println("f;");
  
  Serial.print("const float FR_KNEE_OFFSET_RIGHT = ");
  Serial.print((positions[FR_KNEE_ID-1] - 500) * 0.24f, 2);
  Serial.println("f;");
  
  Serial.print("const float RL_HIP_OFFSET_LEFT = ");
  Serial.print((positions[RL_HIP_ID-1] - 500) * 0.24f, 2);
  Serial.println("f;");
  
  Serial.print("const float RL_KNEE_OFFSET_LEFT = ");
  Serial.print((positions[RL_KNEE_ID-1] - 500) * 0.24f, 2);
  Serial.println("f;");
  
  Serial.print("const float RR_HIP_OFFSET_RIGHT = ");
  Serial.print((positions[RR_HIP_ID-1] - 500) * 0.24f, 2);
  Serial.println("f;");
  
  Serial.print("const float RR_KNEE_OFFSET_RIGHT = ");
  Serial.print((positions[RR_KNEE_ID-1] - 500) * 0.24f, 2);
  Serial.println("f;");
  
  Serial.println("\n=== End of Calibration ===");
}
void loop() {
  if (Serial.available() > 0) {
    Serial.read();  // Clear input

    handleStatus("");
  }
  delay(10);
}

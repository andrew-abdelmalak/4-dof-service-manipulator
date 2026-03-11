/*
 * Arduino Serial Receiver for Joint Control + Gripper
 * 
 * Accepts commands:
 * 1. "J1,J2,J3,J4" - Joint angles in degrees
 * 2. "GRIPPER_OPEN" - Opens the gripper
 * 3. "GRIPPER_CLOSE" - Closes the gripper
 * 
 * Flow:
 * 1. Waits for command
 * 2. Parses and executes
 * 3. Sends "READY" back to request next command
 */

#include <Servo.h>

// ============================================
// CONFIGURATION
// ============================================
const int SERVO_PINS[5] = {3, 5, 6, 9, 10};  // Joints 1-4 + Gripper (Joint 5)
Servo servos[5];

// ============================================
// GRIPPER SAFETY CONFIGURATION
// ============================================
// IMPORTANT: Gripper motor must NOT be fully closed to prevent overload!
// 
// Gripper Joint Angles (degrees):
//   - GRIPPER_OPEN = 90.0°    → Servo Position: 180° (fully open)
//   - GRIPPER_CLOSED = 15.0°  → Servo Position: 105° (TIGHTER grip for better hold)
// 
// Adjusted to 15.0° for tighter grip while maintaining motor safety
const float GRIPPER_OPEN = 90.0;    // Fully open → Servo: 180°
const float GRIPPER_CLOSED = 30.0;  // TIGHTER closing position → Servo: 105°

// Maximum safe closing limit for gripper (safety check)
const float GRIPPER_MIN_SAFE = 30.0;  // Minimum angle to prevent motor overload

// Buffer for incoming serial data
String inputString = "";
bool stringComplete = false;

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200); // Fast baud rate for smooth streaming
  while (!Serial) { }

  // Attach Servos
  for (int i = 0; i < 5; i++) {
    servos[i].attach(SERVO_PINS[i]);
  }

  // Move to a safe "home" position initially (optional)
  // jointToServo(0, 0); ... etc

  Serial.println("ARDUINO_CONNECTED"); // Handshake signal
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  // 1. Process data if a new line has arrived
  if (stringComplete) {
    parseAndMove(inputString);
    
    // Clear buffer for next command
    inputString = "";
    stringComplete = false;
    
    // Tell laptop we are ready for the next line
    Serial.println("READY");
  }
}

// ============================================
// SERIAL EVENT (Interrupt-like)
// ============================================
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. 
  This runs between each time loop() runs.
*/
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // Check for newline character which marks end of command
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

// ============================================
// PARSING AND MOVEMENT LOGIC
// ============================================
void parseAndMove(String data) {
  // Check for special gripper commands
  if (data == "GRIPPER_OPEN") {
    Serial.println(">>> Opening gripper...");
    int servoPos = jointToServo(GRIPPER_OPEN, 4);
    servos[4].write(servoPos);
    delay(1000);  // Wait for gripper to open
    return;
  }
  
  if (data == "GRIPPER_CLOSE") {
    Serial.println(">>> Closing gripper to TIGHT position...");
    Serial.print("    Joint angle: ");
    Serial.print(GRIPPER_CLOSED);
    Serial.print("° -> Servo: ");
    int servoPos = jointToServo(GRIPPER_CLOSED, 4);
    Serial.print(servoPos);
    Serial.println("°");
    servos[4].write(servoPos);
    delay(1000);  // Wait for gripper to close
    return;
  }
  
  // Expected format: "val1,val2,val3,val4"
  // We use standard C string parsing (strtok) for efficiency
  
  char buf[data.length() + 1];
  data.toCharArray(buf, sizeof(buf));
  
  float angles[4];
  char *ptr = strtok(buf, ",");
  int index = 0;
  
  while (ptr != NULL && index < 4) {
    angles[index] = atof(ptr); // Convert string to float
    ptr = strtok(NULL, ",");
    index++;
  }

  // If we didn't get 4 angles, ignore this bad packet
  if (index < 4) return;

  // Apply Calibration and Move Servos (Joints 1-4 only)
  for (int i = 0; i < 4; i++) {
    int servoPos = jointToServo(angles[i], i);
    servos[i].write(servoPos);
  }
}

// ============================================
// CALIBRATION FUNCTION WITH GRIPPER SAFETY
// ============================================
int jointToServo(float joint_deg, int joint_index) {
  float servo_deg;
  
  switch(joint_index) {
    case 0:  // Joint 1: Offset +90
      servo_deg = joint_deg + 90.0;
      break;
    case 1:  // Joint 2: Inverted + 110 offset
      servo_deg = -1.0 * joint_deg + 110.0;
      break;
    case 2:  // Joint 3: Inverted + 90 offset
      servo_deg = -1.0 * joint_deg + 90.0;
      break;
    case 3:  // Joint 4: Inverted + 110 offset
      servo_deg = -1.0 * joint_deg + 110.0;
      break;
    case 4:  // Joint 5 (Gripper): Direct mapping + 90, with SAFETY CHECK
      // SAFETY: Prevent gripper from closing too much
      if (joint_deg < GRIPPER_MIN_SAFE) {
        Serial.print("!!! WARNING: Gripper angle ");
        Serial.print(joint_deg);
        Serial.print("° is below safe minimum ");
        Serial.print(GRIPPER_MIN_SAFE);
        Serial.println("°. Clamping to safe value!");
        joint_deg = GRIPPER_MIN_SAFE;
      }
      servo_deg = joint_deg + 90.0;
      break;
    default:
      servo_deg = 90.0;
  }
  
  // Clamp to valid servo range
  if (servo_deg < 0) servo_deg = 0;
  if (servo_deg > 180) servo_deg = 180;
  
  return (int)servo_deg;
}
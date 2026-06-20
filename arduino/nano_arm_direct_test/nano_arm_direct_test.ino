#include <Servo.h>

// Direct Nano arm pose test.
// Upload from Arduino IDE to verify the arm without Pi/ROS/serial control.
//
// Wiring expected:
// Servo 1 -> D2
// Servo 2 -> D3
// Servo 3 -> D4
// Servo 4 -> D5
// Servo 5 -> D6
// Servo 6 -> D7
//
// Important:
// - Use external servo power
// - Connect external power GND to Nano GND
// - Do not power the arm servos from Nano USB alone

static const uint8_t SERVO_COUNT = 6;
static const uint8_t SERVO_PINS[SERVO_COUNT] = {2, 3, 4, 5, 6, 7};
static const uint16_t SERIAL_BAUD = 9600;

// Smooth motion settings
static const int STEP_DELAY_MS = 12;
static const unsigned long HOLD_MS = 1800;
static const unsigned long PRINT_INTERVAL_MS = 400;

// Full-arm test poses.
// Change these if your arm needs safer angles.
static const int STRAIGHT_POSE[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};
static const int LEFT_POSE[SERVO_COUNT] = {140, 110, 70, 90, 90, 90};
static const int RIGHT_POSE[SERVO_COUNT] = {40, 110, 70, 90, 90, 90};

Servo servos[SERVO_COUNT];
int currentAngles[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};

void printAngles(const char *prefix) {
  Serial.print(prefix);
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    Serial.print(' ');
    Serial.print(currentAngles[i]);
  }
  Serial.println();
}

void writeCurrentPose() {
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    servos[i].write(currentAngles[i]);
  }
}

void copyPose(const int sourcePose[SERVO_COUNT], int targetPose[SERVO_COUNT]) {
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    targetPose[i] = sourcePose[i];
  }
}

void moveToPose(const int targetPose[SERVO_COUNT]) {
  bool moving = true;
  unsigned long lastPrintMs = 0;

  while (moving) {
    moving = false;

    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
      if (currentAngles[i] < targetPose[i]) {
        currentAngles[i]++;
        moving = true;
      } else if (currentAngles[i] > targetPose[i]) {
        currentAngles[i]--;
        moving = true;
      }
      servos[i].write(currentAngles[i]);
    }

    unsigned long nowMs = millis();
    if ((nowMs - lastPrintMs) >= PRINT_INTERVAL_MS) {
      lastPrintMs = nowMs;
      printAngles("POSE");
    }

    delay(STEP_DELAY_MS);
  }

  printAngles("POSE");
}

void holdPose(unsigned long holdMs) {
  delay(holdMs);
}

void runPoseSequence() {
  printAngles("TARGET_STRAIGHT");
  moveToPose(STRAIGHT_POSE);
  holdPose(HOLD_MS);

  printAngles("TARGET_LEFT");
  moveToPose(LEFT_POSE);
  holdPose(HOLD_MS);

  printAngles("TARGET_STRAIGHT");
  moveToPose(STRAIGHT_POSE);
  holdPose(HOLD_MS);

  printAngles("TARGET_RIGHT");
  moveToPose(RIGHT_POSE);
  holdPose(HOLD_MS);

  printAngles("TARGET_STRAIGHT");
  moveToPose(STRAIGHT_POSE);
  holdPose(HOLD_MS);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    servos[i].attach(SERVO_PINS[i]);
  }

  copyPose(STRAIGHT_POSE, currentAngles);
  writeCurrentPose();
  delay(200);
  printAngles("NANO_ARM_DIRECT_READY");
  delay(1000);
}

void loop() {
  runPoseSequence();
}

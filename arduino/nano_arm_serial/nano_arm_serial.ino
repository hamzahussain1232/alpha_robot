#include <Servo.h>

static const uint8_t SERVO_COUNT = 6;
static const uint8_t SERVO_PINS[SERVO_COUNT] = {2, 3, 4, 5, 6, 7};
static const uint16_t SERIAL_BAUD = 9600;

Servo servos[SERVO_COUNT];
int currentAngles[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};

void writeAllServos() {
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    servos[i].write(currentAngles[i]);
  }
}

void printAngles(const char *prefix) {
  Serial.print(prefix);
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    Serial.print(i == 0 ? ' ' : ' ');
    Serial.print(currentAngles[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(20);
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    servos[i].attach(SERVO_PINS[i]);
  }
  delay(200);
  printAngles("NANO_ARM_READY");
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) {
    return;
  }

  char cmd = 0;
  int a0 = 0, a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0;
  int matched = sscanf(
    line.c_str(),
    "%c %d %d %d %d %d %d",
    &cmd, &a0, &a1, &a2, &a3, &a4, &a5
  );

  if ((cmd == 'A' || cmd == 'a') && matched == 7) {
    int values[SERVO_COUNT] = {a0, a1, a2, a3, a4, a5};
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
      values[i] = constrain(values[i], 0, 180);
      currentAngles[i] = values[i];
    }
    writeAllServos();
    printAngles("OK");
    return;
  }

  int index = 0;
  int angle = 0;
  matched = sscanf(line.c_str(), "%c %d %d", &cmd, &index, &angle);
  if ((cmd == 'S' || cmd == 's') && matched == 3) {
    if (index < 1 || index > SERVO_COUNT) {
      Serial.println("ERR");
      return;
    }
    currentAngles[index - 1] = constrain(angle, 0, 180);
    writeAllServos();
    printAngles("OK");
    return;
  }

  if (cmd == 'H' || cmd == 'h') {
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
      currentAngles[i] = 90;
    }
    writeAllServos();
    printAngles("HOME");
    return;
  }

  if (cmd == 'P' || cmd == 'p') {
    printAngles("POS");
    return;
  }

  Serial.println("ERR");
}

// Simple Mega ADK motor wiring test.
// On boot: drive forward for 2 seconds, then stop forever.

const int enFL = 4;
const int inFL1 = 22;
const int inFL2 = 24;

const int enRL = 5;
const int inRL1 = 26;
const int inRL2 = 28;

const int enFR = 6;
const int inFR1 = 23;
const int inFR2 = 25;

const int enRR = 7;
const int inRR1 = 27;
const int inRR2 = 29;

const bool INVERT_FL = true;
const bool INVERT_RL = true;
const bool INVERT_FR = false;
const bool INVERT_RR = false;

const int TEST_PWM = 180;
const unsigned long TEST_DURATION_MS = 2000;

void setup() {
  pinMode(enFL, OUTPUT); pinMode(inFL1, OUTPUT); pinMode(inFL2, OUTPUT);
  pinMode(enRL, OUTPUT); pinMode(inRL1, OUTPUT); pinMode(inRL2, OUTPUT);
  pinMode(enFR, OUTPUT); pinMode(inFR1, OUTPUT); pinMode(inFR2, OUTPUT);
  pinMode(enRR, OUTPUT); pinMode(inRR1, OUTPUT); pinMode(inRR2, OUTPUT);

  stopAll();
  delay(1000);
  driveForward(TEST_PWM);
  delay(TEST_DURATION_MS);
  stopAll();
}

void loop() {
  stopAll();
  delay(1000);
}

void setMotor(int pwm, bool invert, int in1, int in2, int en) {
  int p = invert ? -pwm : pwm;

  if (p > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (p < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  int power = abs(p);
  if (power > 255) {
    power = 255;
  }
  analogWrite(en, power);
}

void driveForward(int pwm) {
  setMotor(pwm, INVERT_FL, inFL1, inFL2, enFL);
  setMotor(pwm, INVERT_RL, inRL1, inRL2, enRL);
  setMotor(pwm, INVERT_FR, inFR1, inFR2, enFR);
  setMotor(pwm, INVERT_RR, inRR1, inRR2, enRR);
}

void stopAll() {
  setMotor(0, false, inFL1, inFL2, enFL);
  setMotor(0, false, inRL1, inRL2, enRL);
  setMotor(0, false, inFR1, inFR2, enFR);
  setMotor(0, false, inRR1, inRR2, enRR);
}

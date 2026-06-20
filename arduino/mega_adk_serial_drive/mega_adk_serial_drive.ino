// L298N 4-motor drive + 4 encoder reporting (Arduino Mega)
// Serial protocol:
//   Command: "M <left_pwm> <right_pwm>\n"  (PWM range -255..255)
//   Telemetry: "E <fl> <fr> <rl> <rr>\n"   (encoder counts)

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

// ==========================================
//              ENCODER PINS
// ==========================================
// A channels must be interrupt-capable pins on Mega: 2,3,18,19
const int encFL_A = 2;
const int encFR_A = 3;
const int encRL_A = 18;
const int encRR_A = 19;

// B channels can be any digital pins
const int encFL_B = 30;
const int encFR_B = 31;
const int encRL_B = 32;
const int encRR_B = 33;

// Invert if counts go backwards
const bool INVERT_ENC_FL = true;
const bool INVERT_ENC_FR = false;
const bool INVERT_ENC_RL = true;
const bool INVERT_ENC_RR = false;

volatile long countFL = 0;
volatile long countFR = 0;
volatile long countRL = 0;
volatile long countRR = 0;

const long SERIAL_BAUD = 115200;
const unsigned long COMMAND_TIMEOUT_MS = 500;
unsigned long last_cmd_ms = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(20);

  pinMode(enFL, OUTPUT); pinMode(inFL1, OUTPUT); pinMode(inFL2, OUTPUT);
  pinMode(enRL, OUTPUT); pinMode(inRL1, OUTPUT); pinMode(inRL2, OUTPUT);
  pinMode(enFR, OUTPUT); pinMode(inFR1, OUTPUT); pinMode(inFR2, OUTPUT);
  pinMode(enRR, OUTPUT); pinMode(inRR1, OUTPUT); pinMode(inRR2, OUTPUT);

  pinMode(encFL_A, INPUT_PULLUP); pinMode(encFL_B, INPUT_PULLUP);
  pinMode(encFR_A, INPUT_PULLUP); pinMode(encFR_B, INPUT_PULLUP);
  pinMode(encRL_A, INPUT_PULLUP); pinMode(encRL_B, INPUT_PULLUP);
  pinMode(encRR_A, INPUT_PULLUP); pinMode(encRR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encFL_A), isrFL, RISING);
  attachInterrupt(digitalPinToInterrupt(encFR_A), isrFR, RISING);
  attachInterrupt(digitalPinToInterrupt(encRL_A), isrRL, RISING);
  attachInterrupt(digitalPinToInterrupt(encRR_A), isrRR, RISING);

  stopAll();
  last_cmd_ms = millis();
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      char cmd;
      int left_pwm = 0;
      int right_pwm = 0;
      int matched = sscanf(line.c_str(), "%c %d %d", &cmd, &left_pwm, &right_pwm);
      if (matched == 3 && (cmd == 'M' || cmd == 'm')) {
        drive(left_pwm, right_pwm);
        last_cmd_ms = millis();
        Serial.print("RX ");
        Serial.print(left_pwm);
        Serial.print(" ");
        Serial.println(right_pwm);
      }
    }
  }

  if (millis() - last_cmd_ms > COMMAND_TIMEOUT_MS) {
    stopAll();
  }

  // Emit encoder counts at ~20 Hz
  static unsigned long last_pub = 0;
  if (millis() - last_pub > 50) {
    last_pub = millis();
    Serial.print("E ");
    Serial.print(countFL); Serial.print(' ');
    Serial.print(countFR); Serial.print(' ');
    Serial.print(countRL); Serial.print(' ');
    Serial.println(countRR);
  }
}

// ==========================================
//            INTERRUPT ISRs
// ==========================================
void isrFL() { countFL += (digitalRead(encFL_B) ^ INVERT_ENC_FL) ? 1 : -1; }
void isrFR() { countFR += (digitalRead(encFR_B) ^ INVERT_ENC_FR) ? 1 : -1; }
void isrRL() { countRL += (digitalRead(encRL_B) ^ INVERT_ENC_RL) ? 1 : -1; }
void isrRR() { countRR += (digitalRead(encRR_B) ^ INVERT_ENC_RR) ? 1 : -1; }

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
  if (power > 255) power = 255;
  analogWrite(en, power);
}

void drive(int left_pwm, int right_pwm) {
  setMotor(left_pwm,  INVERT_FL, inFL1, inFL2, enFL);
  setMotor(left_pwm,  INVERT_RL, inRL1, inRL2, enRL);
  setMotor(right_pwm, INVERT_FR, inFR1, inFR2, enFR);
  setMotor(right_pwm, INVERT_RR, inRR1, inRR2, enRR);
}

void stopAll() {
  setMotor(0, false, inFL1, inFL2, enFL);
  setMotor(0, false, inRL1, inRL2, enRL);
  setMotor(0, false, inFR1, inFR2, enFR);
  setMotor(0, false, inRR1, inRR2, enRR);
}

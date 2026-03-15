// Mega ADK serial drive + 4 encoder reporting
// Serial protocol:
//   Command: "M <left_pwm> <right_pwm>\n"  (PWM range -255..255)
//   Telemetry: "E <fl> <fr> <rl> <rr>\n"   (encoder counts)
//
// Notes:
// - A-channel pins use interrupt-capable pins on Mega: 2,3,18,19
// - B-channel pins can be any digital pins (choose pins not used by motors)
// - Adjust encoder pins, CPR, and invert flags as needed.

// ==========================================
//        L298N #1 (LEFT SIDE MOTORS)
// ==========================================
const int enA_Left = 4;
const int in1_Left = 22; // Front-Left Direction
const int in2_Left = 24; // Front-Left Direction

const int enB_Left = 5;
const int in3_Left = 26; // Rear-Left Direction
const int in4_Left = 28; // Rear-Left Direction

// ==========================================
//        L298N #2 (RIGHT SIDE MOTORS)
// ==========================================
const int enA_Right = 6;
const int in1_Right = 23; // Front-Right Direction
const int in2_Right = 25; // Front-Right Direction

const int enB_Right = 7;
const int in3_Right = 27; // Rear-Right Direction
const int in4_Right = 29; // Rear-Right Direction

// ==========================================
//              ENCODER PINS
// ==========================================
// A channels must be interrupt-capable pins on Mega
const int encFL_A = 2;
const int encFR_A = 3;
const int encRL_A = 18;
const int encRR_A = 19;

// B channels can be any digital pins (avoid motor pins)
const int encFL_B = 30;
const int encFR_B = 31;
const int encRL_B = 32;
const int encRR_B = 33;

// ==========================================
//            ENCODER SETTINGS
// ==========================================
const long ENCODER_CPR = 13730; // adjust if needed
const bool INVERT_FL = false;
const bool INVERT_FR = false;
const bool INVERT_RL = false;
const bool INVERT_RR = false;

volatile long countFL = 0;
volatile long countFR = 0;
volatile long countRL = 0;
volatile long countRR = 0;

// ==========================================
//            SERIAL SETTINGS
// ==========================================
const long SERIAL_BAUD = 115200;
const unsigned long COMMAND_TIMEOUT_MS = 500;
unsigned long last_cmd_ms = 0;

// ==========================================
//            INTERRUPT ISRs
// ==========================================
void isrFL() { countFL += (digitalRead(encFL_B) ^ INVERT_FL) ? 1 : -1; }
void isrFR() { countFR += (digitalRead(encFR_B) ^ INVERT_FR) ? 1 : -1; }
void isrRL() { countRL += (digitalRead(encRL_B) ^ INVERT_RL) ? 1 : -1; }
void isrRR() { countRR += (digitalRead(encRR_B) ^ INVERT_RR) ? 1 : -1; }

// ==========================================
//                SETUP
// ==========================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(5);

  pinMode(enA_Left, OUTPUT); pinMode(in1_Left, OUTPUT); pinMode(in2_Left, OUTPUT);
  pinMode(enB_Left, OUTPUT); pinMode(in3_Left, OUTPUT); pinMode(in4_Left, OUTPUT);
  pinMode(enA_Right, OUTPUT); pinMode(in1_Right, OUTPUT); pinMode(in2_Right, OUTPUT);
  pinMode(enB_Right, OUTPUT); pinMode(in3_Right, OUTPUT); pinMode(in4_Right, OUTPUT);

  pinMode(encFL_A, INPUT_PULLUP); pinMode(encFL_B, INPUT_PULLUP);
  pinMode(encFR_A, INPUT_PULLUP); pinMode(encFR_B, INPUT_PULLUP);
  pinMode(encRL_A, INPUT_PULLUP); pinMode(encRL_B, INPUT_PULLUP);
  pinMode(encRR_A, INPUT_PULLUP); pinMode(encRR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encFL_A), isrFL, RISING);
  attachInterrupt(digitalPinToInterrupt(encFR_A), isrFR, RISING);
  attachInterrupt(digitalPinToInterrupt(encRL_A), isrRL, RISING);
  attachInterrupt(digitalPinToInterrupt(encRR_A), isrRR, RISING);

  stopRobot();
  last_cmd_ms = millis();
}

// ==========================================
//                 LOOP
// ==========================================
void loop() {
  // Read serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'M') {
      int left_pwm = Serial.parseInt();
      int right_pwm = Serial.parseInt();
      applyPwm(left_pwm, right_pwm);
      last_cmd_ms = millis();
    }
  }

  // Safety stop on timeout
  if (millis() - last_cmd_ms > COMMAND_TIMEOUT_MS) {
    stopRobot();
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
//           MOTOR CONTROL HELPERS
// ==========================================
void setSide(int pwm, int in1, int in2, int in3, int in4, int enA, int enB) {
  if (pwm > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  }

  int power = abs(pwm);
  if (power > 255) power = 255;
  analogWrite(enA, power);
  analogWrite(enB, power);
}

void applyPwm(int left_pwm, int right_pwm) {
  setSide(left_pwm, in1_Left, in2_Left, in3_Left, in4_Left, enA_Left, enB_Left);
  setSide(right_pwm, in1_Right, in2_Right, in3_Right, in4_Right, enA_Right, enB_Right);
}

void stopRobot() {
  applyPwm(0, 0);
}

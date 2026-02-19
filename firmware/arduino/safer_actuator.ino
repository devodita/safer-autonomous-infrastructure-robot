/**
 * safer_actuator.ino
 * -------------------
 * Arduino firmware for the SAFER robot actuator board.
 * Receives serial PWM frames from the ROS motor_controller node
 * and drives two DC motors via an L298N H-bridge.
 *
 * Frame format (5 bytes):
 *   [0xAA] [left_pwm] [right_pwm] [checksum] [0x55]
 *   checksum = (left_pwm + right_pwm) & 0xFF
 *   PWM value 127 = stop, 0 = full reverse, 255 = full forward
 *
 * Pin mapping (configurable below):
 *   Left motor:  IN1=2, IN2=3, ENA=9
 *   Right motor: IN3=4, IN4=5, ENB=10
 *   E-STOP input (active LOW): pin 7
 */

// ── Pin definitions ───────────────────────────────────────────────────────────
const int LEFT_IN1  = 2;
const int LEFT_IN2  = 3;
const int LEFT_ENA  = 9;    // PWM-capable

const int RIGHT_IN3 = 4;
const int RIGHT_IN4 = 5;
const int RIGHT_ENB = 10;   // PWM-capable

const int ESTOP_PIN = 7;    // Active LOW — connect to normally-closed E-STOP button

// ── Serial frame constants ───────────────────────────────────────────────────
const byte FRAME_START = 0xAA;
const byte FRAME_END   = 0x55;
const int  FRAME_SIZE  = 5;

// ── State ─────────────────────────────────────────────────────────────────────
byte   frame_buf[FRAME_SIZE];
int    frame_idx    = 0;
bool   estop_active = false;

unsigned long last_frame_ms   = 0;
const unsigned long CMD_TIMEOUT_MS = 500;  // Halt if no frame within 500 ms

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(LEFT_ENA,  OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);
  pinMode(RIGHT_ENB, OUTPUT);
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  haltMotors();
  Serial.begin(115200);
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
  // Hardware E-STOP check
  if (digitalRead(ESTOP_PIN) == LOW) {
    if (!estop_active) {
      estop_active = true;
      haltMotors();
    }
  } else {
    estop_active = false;
  }

  // Software watchdog — halt if no command within timeout
  if (millis() - last_frame_ms > CMD_TIMEOUT_MS) {
    haltMotors();
  }

  // Read serial bytes
  while (Serial.available() > 0) {
    byte b = Serial.read();

    if (b == FRAME_START) {
      frame_idx = 0;
    }

    if (frame_idx < FRAME_SIZE) {
      frame_buf[frame_idx++] = b;
    }

    if (frame_idx == FRAME_SIZE) {
      parseFrame();
      frame_idx = 0;
    }
  }
}

// ── Frame parser ──────────────────────────────────────────────────────────────
void parseFrame() {
  if (frame_buf[0] != FRAME_START || frame_buf[4] != FRAME_END) return;

  byte left_pwm  = frame_buf[1];
  byte right_pwm = frame_buf[2];
  byte checksum  = frame_buf[3];

  if (((left_pwm + right_pwm) & 0xFF) != checksum) return;  // Reject corrupt frames

  last_frame_ms = millis();

  if (!estop_active) {
    applyMotor(LEFT_IN1,  LEFT_IN2,  LEFT_ENA,  left_pwm);
    applyMotor(RIGHT_IN3, RIGHT_IN4, RIGHT_ENB, right_pwm);
  }
}

// ── Motor drive helper ────────────────────────────────────────────────────────
// pwm_byte: 127=stop, 0=full reverse, 255=full forward
void applyMotor(int in1, int in2, int en, byte pwm_byte) {
  int speed = (int)pwm_byte - 127;   // -127 … +128

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, map(speed, 0, 128, 0, 255));
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, map(-speed, 0, 127, 0, 255));
  } else {
    // Brake
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(en, 0);
  }
}

void haltMotors() {
  digitalWrite(LEFT_IN1,  LOW);
  digitalWrite(LEFT_IN2,  LOW);
  analogWrite(LEFT_ENA,   0);
  digitalWrite(RIGHT_IN3, LOW);
  digitalWrite(RIGHT_IN4, LOW);
  analogWrite(RIGHT_ENB,  0);
}

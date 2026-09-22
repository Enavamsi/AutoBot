/*
 * Encoder + Motor Test (Serial Monitor control)
 * ------------------------------------------------
 * Standalone sketch - no micro-ROS, no IMU. Just drives the motors from
 * single-character commands typed into the Serial Monitor and prints live
 * encoder tick counts so you can sanity-check wiring/direction/counts
 * before plugging this into the full ROS node.
 *
 * Open Serial Monitor at 115200 baud, line ending = "Newline" (or none,
 * either works since we only read the first char of each line).
 *
 * Commands:
 *   w  - forward
 *   s  - backward
 *   a  - turn left in place (left wheel back, right wheel fwd)
 *   d  - turn right in place (left wheel fwd, right wheel back)
 *   x  - stop
 *   +  - increase speed by 10 (max 255)
 *   -  - decrease speed by 10 (min 0)
 *   r  - reset tick counters to 0
 *
 * Speed defaults to 255 (max PWM) as requested. Ticks/sec is printed too
 * so you can eyeball whether both wheels spin at roughly the same rate
 * for the same PWM - if not, that's a sign your KP/KI gains will need to
 * differ between the two sides later.
 */

#include <Arduino.h>

// ==========================================================
// MOTOR DRIVER PINS (same as main node)
// ==========================================================
#define ENA 25 // LEFT MOTOR
#define IN1 26
#define IN2 27

#define ENB 33   //RIGHT MOTOR
#define IN3 13
#define IN4 32

#define PWM_CH_LEFT  0
#define PWM_CH_RIGHT 1
#define PWM_FREQ_HZ       5000
#define PWM_RESOLUTION_BITS 8
#define PWM_MAX 255

// ==========================================================
// ENCODER PINS (same as main node)
// ==========================================================
#define LEFT_ENC_A   18
#define LEFT_ENC_B   19

#define RIGHT_ENC_A  23
#define RIGHT_ENC_B  5

volatile long left_ticks = 0;
volatile long right_ticks = 0;

long last_left_ticks = 0;
long last_right_ticks = 0;
unsigned long last_print_time = 0;
const unsigned long PRINT_INTERVAL_MS = 200;

int current_speed = 255;   // default PWM, as requested
char current_cmd = 'x';    // start stopped

// ==========================================================
// Encoder ISRs (same logic as main node)
// ==========================================================
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(LEFT_ENC_B) == HIGH) {
    left_ticks++;
  } else {
    left_ticks--;
  }
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) {
    right_ticks--;
  } else {
    right_ticks++;
  }
}

// ==========================================================
// Motor helpers
// ==========================================================
void setLeftMotor(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  ledcWrite(PWM_CH_LEFT, abs(pwm));
}

void setRightMotor(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  ledcWrite(PWM_CH_RIGHT, abs(pwm));
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
}

// ==========================================================
// Apply whatever the current command + speed is
// ==========================================================
void applyCommand() {
  switch (current_cmd) {
    case 'w':
      setLeftMotor(current_speed);
      setRightMotor(current_speed);
      break;
    case 's':
      setLeftMotor(-current_speed);
      setRightMotor(-current_speed);
      break;
    case 'a':
      setLeftMotor(-current_speed);
      setRightMotor(current_speed);
      break;
    case 'd':
      setLeftMotor(current_speed);
      setRightMotor(-current_speed);
      break;
    case 'x':
    default:
      stopMotors();
      break;
  }
}

void printHelp() {
  Serial.println();
  Serial.println("=== Encoder/Motor Test ===");
  Serial.println("w = forward   s = backward");
  Serial.println("a = left      d = right");
  Serial.println("x = stop      r = reset ticks");
  Serial.println("+ = speed up  - = speed down");
  Serial.print("Current speed: ");
  Serial.println(current_speed);
  Serial.println("==========================");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(PWM_CH_LEFT, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(ENA, PWM_CH_LEFT);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(ENB, PWM_CH_RIGHT);

  stopMotors();

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  last_print_time = millis();

  printHelp();
}

void loop() {
  // --- Read serial commands (non-blocking, one char at a time) ---
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 'w' || c == 's' || c == 'a' || c == 'd' || c == 'x') {
      current_cmd = c;
      applyCommand();
      Serial.print("Command: ");
      Serial.println(c);
    } else if (c == '+') {
      current_speed = constrain(current_speed + 10, 0, PWM_MAX);
      applyCommand();
      Serial.print("Speed: ");
      Serial.println(current_speed);
    } else if (c == '-') {
      current_speed = constrain(current_speed - 10, 0, PWM_MAX);
      applyCommand();
      Serial.print("Speed: ");
      Serial.println(current_speed);
    } else if (c == 'r') {
      noInterrupts();
      left_ticks = 0;
      right_ticks = 0;
      interrupts();
      last_left_ticks = 0;
      last_right_ticks = 0;
      Serial.println("Tick counters reset.");
    } else if (c == 'h') {
      printHelp();
    }
    // ignore newline/carriage-return/anything else
  }

  // --- Periodically print encoder counts + rate ---
  unsigned long now = millis();
  if (now - last_print_time >= PRINT_INTERVAL_MS) {
    noInterrupts();
    long cur_left = left_ticks;
    long cur_right = right_ticks;
    interrupts();

    long delta_left = cur_left - last_left_ticks;
    long delta_right = cur_right - last_right_ticks;
    last_left_ticks = cur_left;
    last_right_ticks = cur_right;

    float dt_s = (now - last_print_time) / 1000.0f;
    float left_tps = delta_left / dt_s;
    float right_tps = delta_right / dt_s;

    Serial.print("L_ticks: ");
    Serial.print(cur_left);
    Serial.print("  R_ticks: ");
    Serial.print(cur_right);
    Serial.print("  | L_ticks/s: ");
    Serial.print(left_tps, 1);
    Serial.print("  R_ticks/s: ");
    Serial.print(right_tps, 1);
    Serial.print("  | cmd: ");
    Serial.print(current_cmd);
    Serial.print("  speed: ");
    Serial.println(current_speed);

    last_print_time = now;
  }
}
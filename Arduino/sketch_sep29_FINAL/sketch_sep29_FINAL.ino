/*
 * ESP32 thin base firmware  -  FLASH ONCE, NEVER EDIT AGAIN
 * ---------------------------------------------------------
 * The ESP32 only does what must be fast/real-time:
 *   - 4x quadrature decoding of both encoders
 *   - per-wheel speed PID (gains, PWM frequency, filters, inversions are ALL set from the Pi)
 *   - PWM output to an L298N-style driver
 *   - BNO085 IMU readout
 * It knows nothing about wheel radius / track width / odometry. The Pi does all kinematics.
 *
 * Needs only the "Adafruit BNO08x" library (no micro-ROS). Classic ESP32 (uses GPIO.in in the ISR).
 * Serial link: USB, 115200 baud, newline-terminated text lines. Do not Serial.print() anything else.
 *
 * Pi -> ESP32                                   ESP32 -> Pi
 *   CFGBEGIN                                      D,<us>,<left>,<right>,qx,qy,qz,qw,gx,gy,gz,ax,ay,az,flags,pwmL,pwmR
 *   SET,<key>,<value>   (repeated)                I,<info text>
 *   CFGEND,<number of SET lines>
 *   W,<left rad/s>,<right rad/s>   (wheel targets, send at ~50 Hz)
 *
 * flags: bit0 = configured, bit1 = IMU alive, bit2 = W commands arriving (watchdog ok)
 * Motors stay OFF until a complete, count-checked config has been received, and stop
 * if no W line arrives within cmd_to ms.
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ---------- Wiring (only change if you re-wire) ----------
#define ENA 25
#define IN1 26
#define IN2 27
#define ENB 33
#define IN3 13
#define IN4 32
#define LEFT_ENC_A  18
#define LEFT_ENC_B  19
#define RIGHT_ENC_A 23
#define RIGHT_ENC_B 5
#define IMU_SDA 21
#define IMU_SCL 22

#define BAUD        115200
#define PWM_BITS    10
#define PWM_MAXDUTY ((1 << PWM_BITS) - 1)
#define CTRL_HZ     100
#define MAX_W       40.0f      // rad/s clamp on commanded wheel speed
#define PWM_CH_L 0
#define PWM_CH_R 1

// ---------- Runtime configuration (defaults only used until the Pi sends its config) ----------
struct Cfg {
  float tpr = 3504.0f;                      // encoder counts per wheel rev (4x decoding)
  float kp_l = 40.0f, ki_l = 0.0f, kff_l = 0.0f;   // PWM(0..255) per rad/s, per rad, feed-forward per rad/s
  float kp_r = 40.0f, ki_r = 0.0f, kff_r = 0.0f;
  float pwm_min = 0.0f;                     // breakaway offset (0..255) applied when output is non-zero
  float lpf = 0.3f;                         // speed filter (0..1, smaller = smoother)
  float idle_w = 0.05f;                     // |target| below this rad/s -> motor fully OFF (no buzzing)
  uint32_t pwm_freq = 20000;
  uint32_t cmd_to_ms = 300;
  uint32_t telem_hz = 50;
  int imu_src = 0;                          // 0 = game rotation vector (no magnetometer), 1 = rotation vector
  bool inv_ml = false, inv_mr = false;      // invert motor direction
  bool inv_el = false, inv_er = true;       // invert encoder direction (right mirrored, as in your old firmware)
} cfg;

// ---------- State ----------
volatile int32_t left_ticks = 0, right_ticks = 0;
volatile uint8_t l_prev = 0, r_prev = 0;
int32_t prev_l = 0, prev_r = 0;
float w_l = 0, w_r = 0;                     // filtered wheel speeds, rad/s
float tgt_l = 0, tgt_r = 0;
float int_l = 0, int_r = 0;
float pwm_l = 0, pwm_r = 0;
uint32_t last_w_ms = 0, last_ctrl_us = 0, last_tel_us = 0;
bool configured = false, cmd_active = false;
int cfg_cnt = 0;

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sv;
bool imu_started = false;
uint32_t last_imu_ms = 0, last_imu_try = 0;
float qx = 0, qy = 0, qz = 0, qw = 1, gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;

void applyImuSource();

// ---------- Encoders: 4x quadrature, both channels, state table ----------
static const int8_t DRAM_ATTR QDEC[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};

void IRAM_ATTR isrLeft() {
  uint32_t in = GPIO.in;
  uint8_t cur = (((in >> LEFT_ENC_A) & 1) << 1) | ((in >> LEFT_ENC_B) & 1);
  left_ticks += QDEC[(l_prev << 2) | cur];
  l_prev = cur;
}
void IRAM_ATTR isrRight() {
  uint32_t in = GPIO.in;
  uint8_t cur = (((in >> RIGHT_ENC_A) & 1) << 1) | ((in >> RIGHT_ENC_B) & 1);
  right_ticks += QDEC[(r_prev << 2) | cur];
  r_prev = cur;
}

// ---------- PWM (works on Arduino-ESP32 core 2.x and 3.x) ----------
void pwmBegin(uint32_t f) {
  static bool attached = false;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!attached) { ledcAttach(ENA, f, PWM_BITS); ledcAttach(ENB, f, PWM_BITS); attached = true; }
  else { ledcChangeFrequency(ENA, f, PWM_BITS); ledcChangeFrequency(ENB, f, PWM_BITS); }
#else
  ledcSetup(PWM_CH_L, f, PWM_BITS);
  ledcSetup(PWM_CH_R, f, PWM_BITS);
  if (!attached) { ledcAttachPin(ENA, PWM_CH_L); ledcAttachPin(ENB, PWM_CH_R); attached = true; }
#endif
}
void pwmWrite(int side, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(side == 0 ? ENA : ENB, duty);
#else
  ledcWrite(side == 0 ? PWM_CH_L : PWM_CH_R, duty);
#endif
}

// pwm in [-255,255], + = forward. Pin pattern matches your old firmware; flip with inv_ml / inv_mr.
void setMotor(int side, float pwm) {
  if (side == 0 ? cfg.inv_ml : cfg.inv_mr) pwm = -pwm;
  int a = side == 0 ? IN1 : IN3, b = side == 0 ? IN2 : IN4;
  if (pwm >= 0) { digitalWrite(a, LOW);  digitalWrite(b, HIGH); }
  else          { digitalWrite(a, HIGH); digitalWrite(b, LOW); }
  pwmWrite(side, (uint32_t)(fabsf(pwm) * PWM_MAXDUTY / 255.0f + 0.5f));
}

// ---------- Config from the Pi ----------
bool applySet(const char* k, float v) {
  if      (!strcmp(k, "tpr"))      { if (v <= 0) return false; cfg.tpr = v; }
  else if (!strcmp(k, "kp_l"))     cfg.kp_l = v;
  else if (!strcmp(k, "ki_l"))     cfg.ki_l = v;
  else if (!strcmp(k, "kff_l"))    cfg.kff_l = v;
  else if (!strcmp(k, "kp_r"))     cfg.kp_r = v;
  else if (!strcmp(k, "ki_r"))     cfg.ki_r = v;
  else if (!strcmp(k, "kff_r"))    cfg.kff_r = v;
  else if (!strcmp(k, "pwm_min"))  cfg.pwm_min = constrain(v, 0.0f, 200.0f);
  else if (!strcmp(k, "lpf"))      cfg.lpf = constrain(v, 0.01f, 1.0f);
  else if (!strcmp(k, "idle_w"))   cfg.idle_w = max(v, 0.0f);
  else if (!strcmp(k, "cmd_to"))   cfg.cmd_to_ms = constrain((int)v, 50, 5000);
  else if (!strcmp(k, "telem_hz")) cfg.telem_hz = constrain((int)v, 5, 100);
  else if (!strcmp(k, "pwm_freq")) { cfg.pwm_freq = constrain((int)v, 500, 40000); pwmBegin(cfg.pwm_freq); }
  else if (!strcmp(k, "imu_src"))  { cfg.imu_src = ((int)v) ? 1 : 0; if (imu_started) applyImuSource(); }
  else if (!strcmp(k, "inv_ml"))   cfg.inv_ml = (v != 0);
  else if (!strcmp(k, "inv_mr"))   cfg.inv_mr = (v != 0);
  else if (!strcmp(k, "inv_el"))   cfg.inv_el = (v != 0);
  else if (!strcmp(k, "inv_er"))   cfg.inv_er = (v != 0);
  else return false;
  return true;
}

static bool parseF(const char* s, float& out) {
  if (!s) return false;
  char* e;
  float v = strtof(s, &e);
  if (e == s) return false;
  out = v;
  return true;
}

void handleLine(char* line) {
  char* cmd = strtok(line, ",");
  if (!cmd) return;
  if (!strcmp(cmd, "W")) {
    float a, b;
    if (parseF(strtok(NULL, ","), a) && parseF(strtok(NULL, ","), b)) {
      tgt_l = constrain(a, -MAX_W, MAX_W);
      tgt_r = constrain(b, -MAX_W, MAX_W);
      last_w_ms = millis();
    }
  } else if (!strcmp(cmd, "SET")) {
    char* k = strtok(NULL, ",");
    float v;
    if (k && parseF(strtok(NULL, ","), v) && applySet(k, v)) cfg_cnt++;
    else Serial.printf("I,bad SET %s\n", k ? k : "?");
  } else if (!strcmp(cmd, "CFGBEGIN")) {
    cfg_cnt = 0;
  } else if (!strcmp(cmd, "CFGEND")) {
    char* n = strtok(NULL, ",");
    configured = (n && atoi(n) == cfg_cnt);
    Serial.printf("I,config %s (%d keys)\n", configured ? "OK" : "INCOMPLETE", cfg_cnt);
  }
}

void readSerial() {
  static char rx[160];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) { rx[n] = 0; handleLine(rx); n = 0; }
    } else if (n < sizeof(rx) - 1) rx[n++] = c;
    else n = 0;  // overflow: drop the line
  }
}

// ---------- IMU ----------
void applyImuSource() {
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
  bno08x.enableReport(SH2_ACCELEROMETER, 20000);
  if (cfg.imu_src == 0) {
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
    bno08x.enableReport(SH2_ROTATION_VECTOR, 0);
  } else {
    bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 0);
  }
}

void imuBegin() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  if (bno08x.begin_I2C()) {
    imu_started = true;
    applyImuSource();
    Serial.println("I,IMU ok");
  } else {
    imu_started = false;   // not fatal: robot still drives, retry later
  }
}

void pollImu() {
  if (!imu_started) {
    if (millis() - last_imu_try > 3000) { last_imu_try = millis(); imuBegin(); }
    return;
  }
  if (bno08x.wasReset()) applyImuSource();
  while (bno08x.getSensorEvent(&sv)) {
    last_imu_ms = millis();
    switch (sv.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        qx = sv.un.gameRotationVector.i; qy = sv.un.gameRotationVector.j;
        qz = sv.un.gameRotationVector.k; qw = sv.un.gameRotationVector.real; break;
      case SH2_ROTATION_VECTOR:
        qx = sv.un.rotationVector.i; qy = sv.un.rotationVector.j;
        qz = sv.un.rotationVector.k; qw = sv.un.rotationVector.real; break;
      case SH2_GYROSCOPE_CALIBRATED:
        gx = sv.un.gyroscope.x; gy = sv.un.gyroscope.y; gz = sv.un.gyroscope.z; break;
      case SH2_ACCELEROMETER:
        ax = sv.un.accelerometer.x; ay = sv.un.accelerometer.y; az = sv.un.accelerometer.z; break;
      default: break;
    }
  }
}

// ---------- Wheel speed control ----------
float wheelPid(float tgt, float meas, float kp, float ki, float kff, float& integ, float dt) {
  if (fabsf(tgt) < cfg.idle_w) { integ = 0; return 0; }     // idle: output exactly 0 -> silent motors
  float err = tgt - meas;
  if (ki > 0) { integ += err * dt; float lim = 255.0f / ki; integ = constrain(integ, -lim, lim); }
  else integ = 0;
  float u = constrain(kff * tgt + kp * err + ki * integ, -255.0f, 255.0f);
  if (cfg.pwm_min > 0 && fabsf(u) > 1.0f) {                 // dead-zone compensation
    float s = u > 0 ? 1.0f : -1.0f;
    u = s * (cfg.pwm_min + fabsf(u) * (255.0f - cfg.pwm_min) / 255.0f);
  }
  return u;
}

void controlStep() {
  uint32_t nowu = micros();
  float dt = (nowu - last_ctrl_us) * 1e-6f;
  last_ctrl_us = nowu;
  if (dt <= 0 || dt > 0.1f) dt = 1.0f / CTRL_HZ;

  int32_t l, r;
  noInterrupts(); l = left_ticks; r = right_ticks; interrupts();
  if (cfg.inv_el) l = -l;
  if (cfg.inv_er) r = -r;
  float k = 2.0f * PI / cfg.tpr / dt;
  w_l += cfg.lpf * ((l - prev_l) * k - w_l);
  w_r += cfg.lpf * ((r - prev_r) * k - w_r);
  prev_l = l; prev_r = r;

  cmd_active = configured && (millis() - last_w_ms) < cfg.cmd_to_ms;
  float tl = cmd_active ? tgt_l : 0, tr = cmd_active ? tgt_r : 0;
  pwm_l = wheelPid(tl, w_l, cfg.kp_l, cfg.ki_l, cfg.kff_l, int_l, dt);
  pwm_r = wheelPid(tr, w_r, cfg.kp_r, cfg.ki_r, cfg.kff_r, int_r, dt);
  if (!configured) { pwm_l = 0; pwm_r = 0; }
  setMotor(0, pwm_l);
  setMotor(1, pwm_r);
}

void sendTelemetry() {
  int32_t l, r;
  uint32_t us;
  noInterrupts(); l = left_ticks; r = right_ticks; us = micros(); interrupts();   // ticks + timestamp together
  if (cfg.inv_el) l = -l;
  if (cfg.inv_er) r = -r;
  uint8_t flags = (configured ? 1 : 0) | ((millis() - last_imu_ms < 500 && imu_started) ? 2 : 0) | (cmd_active ? 4 : 0);
  char buf[200];
  int n = snprintf(buf, sizeof(buf), "D,%lu,%ld,%ld,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%u,%d,%d\n",
                   (unsigned long)us, (long)l, (long)r, qx, qy, qz, qw, gx, gy, gz, ax, ay, az,
                   flags, (int)pwm_l, (int)pwm_r);
  Serial.write((const uint8_t*)buf, n);
}

// ---------- Arduino ----------
void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(BAUD);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pwmBegin(cfg.pwm_freq);
  setMotor(0, 0); setMotor(1, 0);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP); pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  l_prev = (digitalRead(LEFT_ENC_A) << 1) | digitalRead(LEFT_ENC_B);
  r_prev = (digitalRead(RIGHT_ENC_A) << 1) | digitalRead(RIGHT_ENC_B);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isrRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), isrRight, CHANGE);

  imuBegin();
  Serial.println("I,boot");
  last_ctrl_us = micros();
  last_tel_us = micros();
}

void loop() {
  readSerial();
  pollImu();
  uint32_t now = micros();
  if ((uint32_t)(now - last_ctrl_us) >= 1000000UL / CTRL_HZ) controlStep();
  if ((uint32_t)(now - last_tel_us) >= 1000000UL / cfg.telem_hz) { last_tel_us = now; sendTelemetry(); }
}
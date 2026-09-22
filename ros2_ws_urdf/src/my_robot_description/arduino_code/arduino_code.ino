/*
 * ESP32 micro-ROS Differential Drive Node
 * -----------------------------------------
 * Subscribes : /cmd_vel   (geometry_msgs/msg/Twist)
 * Publishes  : /odom      (nav_msgs/msg/Odometry)
 *
 * Motor driver : L298N-style (ENA/IN1/IN2 = left, ENB/IN3/IN4 = right)
 * Encoders     : Quadrature, 2 channels per wheel, read via interrupts
 *
 * IMPORTANT - values you MUST tune for your robot before this works well:
 *   WHEEL_RADIUS_M, WHEEL_SEPARATION_M, ENCODER_TICKS_PER_REV
 *   KP_LEFT / KI_LEFT / KP_RIGHT / KI_RIGHT (velocity PI controller gains)
 *
 * Requires: micro_ros_arduino library (match the version to your ROS 2 distro,
 * e.g. the "jazzy" branch/release of micro_ros_arduino).
 *
 * Run the agent on your PC before/while the ESP32 boots:
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
 */

#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

// ==========================================================
// USER-TUNABLE ROBOT PARAMETERS
// ==========================================================
#define WHEEL_RADIUS_M        0.1f   // TODO: measure your wheel radius (m)
#define WHEEL_SEPARATION_M    0.3f    // TODO: measure track width, wheel-to-wheel (m)
#define ENCODER_TICKS_PER_REV 3528.0f  // TODO: (CPR * gear ratio * quadrature factor)

#define PWM_MAX          255
#define PWM_FREQ_HZ       5000
#define PWM_RESOLUTION_BITS 8

#define CONTROL_LOOP_HZ   50           // motor PID + encoder integration rate
#define ODOM_PUBLISH_HZ   20           // /odom publish rate
#define CMD_VEL_TIMEOUT_MS 500         // stop motors if no cmd_vel received in this time

// Velocity PI controller gains (rad/s wheel angular velocity error -> PWM)
// Start conservative and tune upward.
#define KP_LEFT   40.0f
#define KI_LEFT   15.0f
#define KP_RIGHT  40.0f
#define KI_RIGHT  15.0f

// ==========================================================
// MOTOR DRIVER PINS (from user)
// ==========================================================
#define ENA 25
#define IN1 26
#define IN2 27

#define ENB 33
#define IN3 12
#define IN4 14

#define PWM_CH_LEFT  0
#define PWM_CH_RIGHT 1

// ==========================================================
// ENCODER PINS (from user)
// ==========================================================
#define LEFT_ENC_A   18
#define LEFT_ENC_B   19

#define RIGHT_ENC_A  16
#define RIGHT_ENC_B  17

// ==========================================================
// micro-ROS globals
// ==========================================================
rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t odom_publisher;
geometry_msgs__msg__Twist cmd_vel_msg;
nav_msgs__msg__Odometry odom_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// ==========================================================
// Shared state (volatile: touched inside ISRs)
// ==========================================================
volatile long left_ticks = 0;
volatile long right_ticks = 0;

long last_left_ticks = 0;
long last_right_ticks = 0;

float target_lin_vel = 0.0f;   // from /cmd_vel, m/s
float target_ang_vel = 0.0f;   // from /cmd_vel, rad/s

unsigned long last_cmd_vel_time = 0;
unsigned long last_control_time = 0;

// Odometry state
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta = 0.0f;

// PI controller integral terms
float left_integral = 0.0f;
float right_integral = 0.0f;

// ==========================================================
// Encoder ISRs (simple quadrature: use A channel edge + B level for direction)
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
    right_ticks--;   // reversed vs left because wheels are mirrored
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
// micro-ROS error loop (halts execution forever on fatal error)
// ==========================================================
void error_loop() {
  while (1) {
    delay(100);
  }
}

// ==========================================================
// /cmd_vel subscriber callback
// ==========================================================
void cmdVelCallback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  target_lin_vel = (float)msg->linear.x;
  target_ang_vel = (float)msg->angular.z;
  last_cmd_vel_time = millis();
}

// ==========================================================
// Quaternion helper (2D yaw-only rotation)
// ==========================================================
void yawToQuaternion(float yaw, double &qx, double &qy, double &qz, double &qw) {
  qx = 0.0;
  qy = 0.0;
  qz = sin(yaw * 0.5);
  qw = cos(yaw * 0.5);
}

// ==========================================================
// Control timer callback: runs motor PI control + odometry integration
// ==========================================================
void controlTimerCallback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == NULL) return;

  unsigned long now = millis();
  float dt = (now - last_control_time) / 1000.0f;
  if (dt <= 0.0f) dt = 1.0f / CONTROL_LOOP_HZ;
  last_control_time = now;

  // Safety: stop if no cmd_vel received recently
  if (now - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
    target_lin_vel = 0.0f;
    target_ang_vel = 0.0f;
  }

  // --- Read encoder deltas (atomic-ish: disable interrupts briefly) ---
  noInterrupts();
  long cur_left = left_ticks;
  long cur_right = right_ticks;
  interrupts();

  long delta_left = cur_left - last_left_ticks;
  long delta_right = cur_right - last_right_ticks;
  last_left_ticks = cur_left;
  last_right_ticks = cur_right;

  float dist_per_tick = (2.0f * PI * WHEEL_RADIUS_M) / ENCODER_TICKS_PER_REV;

  float left_wheel_dist  = delta_left  * dist_per_tick;
  float right_wheel_dist = delta_right * dist_per_tick;

  float left_wheel_vel  = left_wheel_dist  / dt;  // m/s at wheel rim
  float right_wheel_vel = right_wheel_dist / dt;  // m/s at wheel rim

  float left_wheel_ang_vel  = left_wheel_vel  / WHEEL_RADIUS_M;  // rad/s
  float right_wheel_ang_vel = right_wheel_vel / WHEEL_RADIUS_M;  // rad/s

  // --- Differential drive inverse kinematics: target wheel velocities ---
  float target_left_vel  = target_lin_vel - (target_ang_vel * WHEEL_SEPARATION_M / 2.0f);
  float target_right_vel = target_lin_vel + (target_ang_vel * WHEEL_SEPARATION_M / 2.0f);

  float target_left_ang_vel  = target_left_vel  / WHEEL_RADIUS_M;
  float target_right_ang_vel = target_right_vel / WHEEL_RADIUS_M;

  // --- PI control on wheel angular velocity ---
  float left_error  = target_left_ang_vel  - left_wheel_ang_vel;
  float right_error = target_right_ang_vel - right_wheel_ang_vel;

  left_integral  += left_error  * dt;
  right_integral += right_error * dt;

  // simple anti-windup clamp
  left_integral  = constrain(left_integral,  -50.0f, 50.0f);
  right_integral = constrain(right_integral, -50.0f, 50.0f);

  int left_pwm  = (int)(KP_LEFT  * left_error  + KI_LEFT  * left_integral);
  int right_pwm = (int)(KP_RIGHT * right_error + KI_RIGHT * right_integral);

  // If target is exactly zero and we're basically stopped, kill PWM to avoid jitter
  if (target_lin_vel == 0.0f && target_ang_vel == 0.0f) {
    left_pwm = 0;
    right_pwm = 0;
    left_integral = 0;
    right_integral = 0;
  }

  setLeftMotor(left_pwm);
  setRightMotor(right_pwm);

  // --- Odometry integration (differential drive forward kinematics) ---
  float v = (left_wheel_vel + right_wheel_vel) / 2.0f;
  float w = (right_wheel_vel - left_wheel_vel) / WHEEL_SEPARATION_M;

  odom_theta += w * dt;
  odom_x += v * cos(odom_theta) * dt;
  odom_y += v * sin(odom_theta) * dt;

  // --- Fill and publish /odom at ODOM_PUBLISH_HZ (every Nth control tick) ---
  static uint8_t publish_divider = 0;
  publish_divider++;
  if (publish_divider >= (CONTROL_LOOP_HZ / ODOM_PUBLISH_HZ)) {
    publish_divider = 0;

    int64_t stamp_ms = rmw_uros_epoch_millis();
    odom_msg.header.stamp.sec = (int32_t)(stamp_ms / 1000);
    odom_msg.header.stamp.nanosec = (uint32_t)((stamp_ms % 1000) * 1000000);

    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0;

    double qx, qy, qz, qw;
    yawToQuaternion(odom_theta, qx, qy, qz, qw);
    odom_msg.pose.pose.orientation.x = qx;
    odom_msg.pose.pose.orientation.y = qy;
    odom_msg.pose.pose.orientation.z = qz;
    odom_msg.pose.pose.orientation.w = qw;

    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = w;

    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
  }
}

// ==========================================================
// Setup
// ==========================================================
void setup() {
  // --- micro-ROS transport (serial over USB) ---
  set_microros_transports();

  // --- Motor pins ---
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(PWM_CH_LEFT, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(ENA, PWM_CH_LEFT);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(ENB, PWM_CH_RIGHT);

  stopMotors();

  // --- Encoder pins ---
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  delay(2000); // give the agent time to be ready on the PC side

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_diffdrive_node", "", &support));

  // --- Subscriber: /cmd_vel ---
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  // --- Publisher: /odom ---
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom"));

  odom_msg.header.frame_id.data = (char *)"odom";
  odom_msg.header.frame_id.size = strlen("odom");
  odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;

  odom_msg.child_frame_id.data = (char *)"base_footprint";
  odom_msg.child_frame_id.size = strlen("base_footprint");
  odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;

  // --- Control timer (drives PID + odom integration + publish) ---
  const unsigned int control_period_ms = 1000 / CONTROL_LOOP_HZ;
  RCCHECK(rclc_timer_init_default(
    &control_timer,
    &support,
    RCL_MS_TO_NS(control_period_ms),
    controlTimerCallback));

  // --- Executor: 1 subscription + 1 timer ---
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  last_control_time = millis();
  last_cmd_vel_time = millis(); // don't immediately trigger the timeout on boot
}

// ==========================================================
// Main loop
// ==========================================================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
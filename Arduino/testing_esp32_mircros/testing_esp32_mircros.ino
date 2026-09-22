/*
 * ESP32 micro-ROS Differential Drive Node (Open-Loop / No PID)
 * -----------------------------------------
 * Subscribes : /cmd_vel   (geometry_msgs/msg/Twist)
 * Publishes  : /odom      (nav_msgs/msg/Odometry)
 *              /imu/data  (sensor_msgs/msg/Imu)   <- for robot_localization
 */

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ==========================================================
// WIFI / MICRO-ROS AGENT CONFIG 
// ==========================================================
#define WIFI_SSID       "CONNECT AND DIE"
#define WIFI_PASSWORD   "nfdq5490"
#define AGENT_IP        "10.19.72.9"   
#define AGENT_PORT      8888

// ==========================================================
// USER-TUNABLE ROBOT PARAMETERS
// ==========================================================
#define WHEEL_RADIUS_M        0.0425f   
#define WHEEL_SEPARATION_M    0.28f    
#define ENCODER_TICKS_PER_REV 876.0f  

#define PWM_MAX          255
#define PWM_FREQ_HZ       5000
#define PWM_RESOLUTION_BITS 8

#define CONTROL_LOOP_HZ   50           
#define ODOM_PUBLISH_HZ   20           
#define IMU_PUBLISH_HZ    50           
#define IMU_REPORT_INTERVAL_US 10000   
#define CMD_VEL_TIMEOUT_MS 500         

// Estimated max angular velocity of the wheel (rad/s) at full PWM.
// Increase this if the robot drives too fast in RViz compared to reality.
// Decrease this if the robot drives too slow in RViz compared to reality.
#define MAX_WHEEL_ANG_VEL 10.0f 

// ==========================================================
// MOTOR DRIVER PINS 
// ==========================================================
#define ENA 25
#define IN1 26
#define IN2 27

#define ENB 33
#define IN3 13
#define IN4 32

#define PWM_CH_LEFT  0
#define PWM_CH_RIGHT 1

// ==========================================================
// ENCODER PINS 
// ==========================================================
#define LEFT_ENC_A   18
#define LEFT_ENC_B   19

#define RIGHT_ENC_A  23
#define RIGHT_ENC_B  5

// ==========================================================
// IMU (BNO085) PINS
// ==========================================================
#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 22

// ==========================================================
// micro-ROS globals
// ==========================================================
rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
geometry_msgs__msg__Twist cmd_vel_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_timer_t imu_timer;

// --- BNO085 IMU ---
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

double imu_qx = 0.0, imu_qy = 0.0, imu_qz = 0.0, imu_qw = 1.0;
double imu_gx = 0.0, imu_gy = 0.0, imu_gz = 0.0;   
double imu_ax = 0.0, imu_ay = 0.0, imu_az = 0.0;   

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// ==========================================================
// Shared state (volatile: touched inside ISRs)
// ==========================================================
volatile long left_ticks = 0;
volatile long right_ticks = 0;

long last_left_ticks = 0;
long last_right_ticks = 0;

float target_lin_vel = 0.0f;   
float target_ang_vel = 0.0f;   

unsigned long last_cmd_vel_time = 0;
unsigned long last_control_time = 0;

// Odometry state
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta = 0.0f;

// ==========================================================
// Encoder ISRs
// ==========================================================
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(LEFT_ENC_B) == HIGH) {
    left_ticks--;
  } else {  
    left_ticks++;
  }
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) {
    right_ticks++;   
  } else {
    right_ticks--;
  }
}

// ==========================================================
// Motor helpers
// ==========================================================
void setLeftMotor(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  ledcWrite(PWM_CH_LEFT, abs(pwm));
}

void setRightMotor(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm >= 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  ledcWrite(PWM_CH_RIGHT, abs(pwm));
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
}

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
// BNO085 helpers
// ==========================================================
void setImuReports() {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US)) { error_loop(); }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_REPORT_INTERVAL_US)) { error_loop(); }
  if (!bno08x.enableReport(SH2_ACCELEROMETER, IMU_REPORT_INTERVAL_US)) { error_loop(); }
}

void pollImu() {
  while (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
        imu_qx = sensorValue.un.rotationVector.i;
        imu_qy = sensorValue.un.rotationVector.j;
        imu_qz = sensorValue.un.rotationVector.k;
        imu_qw = sensorValue.un.rotationVector.real;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        imu_gx = sensorValue.un.gyroscope.x;
        imu_gy = sensorValue.un.gyroscope.y;
        imu_gz = sensorValue.un.gyroscope.z;
        break;
      case SH2_ACCELEROMETER:
        imu_ax = sensorValue.un.accelerometer.x;
        imu_ay = sensorValue.un.accelerometer.y;
        imu_az = sensorValue.un.accelerometer.z;
        break;
      default:
        break;
    }
  }
}

void imuTimerCallback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == NULL) return;

  int64_t stamp_ms = rmw_uros_epoch_millis();
  imu_msg.header.stamp.sec = (int32_t)(stamp_ms / 1000);
  imu_msg.header.stamp.nanosec = (uint32_t)((stamp_ms % 1000) * 1000000);

  imu_msg.orientation.x = imu_qx;
  imu_msg.orientation.y = imu_qy;
  imu_msg.orientation.z = imu_qz;
  imu_msg.orientation.w = imu_qw;

  imu_msg.angular_velocity.x = imu_gx;
  imu_msg.angular_velocity.y = imu_gy;
  imu_msg.angular_velocity.z = imu_gz;

  imu_msg.linear_acceleration.x = imu_ax;
  imu_msg.linear_acceleration.y = imu_ay;
  imu_msg.linear_acceleration.z = imu_az;

  RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
}

void yawToQuaternion(float yaw, double &qx, double &qy, double &qz, double &qw) {
  qx = 0.0;
  qy = 0.0;
  qz = sin(yaw * 0.5);
  qw = cos(yaw * 0.5);
}

// ==========================================================
// Control timer callback
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

  // --- Read encoder deltas ---
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

  float left_wheel_vel  = left_wheel_dist  / dt;  
  float right_wheel_vel = right_wheel_dist / dt;  

  // --- Differential drive inverse kinematics ---
  float target_left_vel  = target_lin_vel - (target_ang_vel * WHEEL_SEPARATION_M / 2.0f);
  float target_right_vel = target_lin_vel + (target_ang_vel * WHEEL_SEPARATION_M / 2.0f);

  float target_left_ang_vel  = target_left_vel  / WHEEL_RADIUS_M;
  float target_right_ang_vel = target_right_vel / WHEEL_RADIUS_M;

  // --- OPEN-LOOP MOTOR CONTROL (NO PID) ---
  int left_pwm  = (int)((target_left_ang_vel / MAX_WHEEL_ANG_VEL) * PWM_MAX);
  int right_pwm = (int)((target_right_ang_vel / MAX_WHEEL_ANG_VEL) * PWM_MAX);

  left_pwm  = constrain(left_pwm,  -PWM_MAX, PWM_MAX);
  right_pwm = constrain(right_pwm, -PWM_MAX, PWM_MAX);

  if (target_lin_vel == 0.0f && target_ang_vel == 0.0f) {
    left_pwm = 0;
    right_pwm = 0;
  }

  setLeftMotor(left_pwm);
  setRightMotor(right_pwm);

  // --- Odometry integration (forward kinematics for RViz) ---
  float v = (left_wheel_vel + right_wheel_vel) / 2.0f;
  float w = (right_wheel_vel - left_wheel_vel) / WHEEL_SEPARATION_M;

  odom_theta += w * dt;
  odom_x += v * cos(odom_theta) * dt;
  odom_y += v * sin(odom_theta) * dt;

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
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected, IP address: ");
  Serial.println(WiFi.localIP());

  set_microros_wifi_transports(
    (char *)WIFI_SSID,
    (char *)WIFI_PASSWORD,
    (char *)AGENT_IP,
    AGENT_PORT);

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

  Wire.begin();
  if (!bno08x.begin_I2C()) {
    error_loop();
  }
  setImuReports();

  delay(2000); 

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_diffdrive_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

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

  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data"));

  imu_msg.header.frame_id.data = (char *)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }
  imu_msg.orientation_covariance[0] = 0.0025;        
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[8] = 0.0025;
  imu_msg.angular_velocity_covariance[0] = 0.0004;
  imu_msg.angular_velocity_covariance[4] = 0.0004;
  imu_msg.angular_velocity_covariance[8] = 0.0004;
  imu_msg.linear_acceleration_covariance[0] = 0.0064;
  imu_msg.linear_acceleration_covariance[4] = 0.0064;
  imu_msg.linear_acceleration_covariance[8] = 0.0064;

  const unsigned int control_period_ms = 1000 / CONTROL_LOOP_HZ;
  RCCHECK(rclc_timer_init_default(
    &control_timer,
    &support,
    RCL_MS_TO_NS(control_period_ms),
    controlTimerCallback));

  const unsigned int imu_period_ms = 1000 / IMU_PUBLISH_HZ;
  RCCHECK(rclc_timer_init_default(
    &imu_timer,
    &support,
    RCL_MS_TO_NS(imu_period_ms),
    imuTimerCallback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));

  last_control_time = millis();
  last_cmd_vel_time = millis(); 
}

// ==========================================================
// Main loop
// ==========================================================
void loop() {
  pollImu();
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 500) {
    Serial.print("Left Ticks: ");
    Serial.print(left_ticks);
    Serial.print("  |  Right Ticks: ");
    Serial.println(right_ticks);
    last_print_time = millis();
  }
}
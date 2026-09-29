#!/usr/bin/env python3
"""
ESP32 differential-drive driver (ROS 2) - ALL parameters and ALL math live here on the Pi.

Run:   python3 esp32_driver.py --ros-args --params-file robot_params.yaml
Live:  ros2 param set /esp32_driver kp_left 30.0          (pushed to the ESP32 immediately)
Save:  ros2 param dump /esp32_driver > robot_params.yaml

The ESP32 firmware never needs editing: it receives every setting below over serial at start-up
(and again automatically if the ESP32 resets), and a changed parameter is sent the moment you set it.
Pi-side maths: cmd_vel ramping, inverse kinematics (incl. per-wheel scale), odometry, IMU gyro bias.
"""
import math
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Imu

# name -> default. NOTE: use 1.0 (float) not 1 in YAML for float params.
DEFAULTS = {
    'port': '/tmp/esp32_base', 'baudrate': 115200,
    # --- geometry / calibration (only ticks_per_rev is also sent to the ESP32) ---
    'wheel_radius': 0.0425, 'wheel_separation': 0.28,
    'ticks_per_rev': 3272.0,          # 4x decoding: your old 1x value 876 x 4
    'left_scale': 1.0, 'right_scale': 1.0,   # per-wheel distance correction (fixes drift when wheels differ)
    # --- wheel speed PID on the ESP32: PWM(0..255) per rad/s (kp), per rad (ki), per rad/s feed-forward (kff) ---
    'kp_left': 40.0, 'ki_left': 60.0, 'kff_left': 0.0,
    'kp_right': 40.0, 'ki_right': 60.0, 'kff_right': 0.0,
    'pwm_min': 0.0, 'pwm_freq': 20000, 'speed_lpf': 0.3, 'idle_w': 0.05,
    'fw_cmd_timeout_ms': 300, 'telem_hz': 50,
    'invert_left_motor': False, 'invert_right_motor': False,
    'invert_left_encoder': False, 'invert_right_encoder': True,
    'imu_source': 0,                   # 0 = game rotation vector (no magnetometer), 1 = with magnetometer
    # --- Pi-side command shaping ---
    'max_linear_accel': 1.5, 'max_angular_accel': 2.0,
    'max_linear_decel': 1.5, 'max_angular_decel': 5.0,
    'vel_snap_linear': 0.02, 'vel_snap_angular': 0.05,
    'cmd_timeout': 0.5, 'cmd_rate_hz': 50.0,
    # --- telemetry sanity ---
    'max_dt': 0.5, 'max_ticks_per_read': 8000.0, 'encoder_noise_ticks': 2,
    # --- IMU corrections and frames ---
    'gyro_bias_x': 0.0, 'gyro_bias_y': 0.0, 'gyro_bias_z': 0.0,
    'odom_frame': 'odom', 'base_frame': 'base_footprint', 'imu_frame': 'imu_link',
    # --- covariances (variances) ---
    'cov_pos': 0.02, 'cov_yaw': 0.05, 'cov_vx': 0.02, 'cov_wz': 0.05,
    'cov_imu_orient': 0.0025, 'cov_imu_gyro': 0.0004, 'cov_imu_accel': 0.0064,
}

# ROS param -> (firmware key, converter)
FW = {
    'ticks_per_rev': ('tpr', float),
    'kp_left': ('kp_l', float), 'ki_left': ('ki_l', float), 'kff_left': ('kff_l', float),
    'kp_right': ('kp_r', float), 'ki_right': ('ki_r', float), 'kff_right': ('kff_r', float),
    'pwm_min': ('pwm_min', float), 'pwm_freq': ('pwm_freq', int), 'speed_lpf': ('lpf', float),
    'idle_w': ('idle_w', float), 'fw_cmd_timeout_ms': ('cmd_to', int), 'telem_hz': ('telem_hz', int),
    'invert_left_motor': ('inv_ml', int), 'invert_right_motor': ('inv_mr', int),
    'invert_left_encoder': ('inv_el', int), 'invert_right_encoder': ('inv_er', int),
    'imu_source': ('imu_src', int),
}


class ESP32Driver(Node):
    def __init__(self):
        super().__init__('esp32_driver')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.P = {k: self.get_parameter(k).value for k in DEFAULTS}
        self.add_on_set_parameters_callback(self._on_params)

        try:
            self.ser = serial.Serial(self.P['port'], self.P['baudrate'], timeout=0)
            self.get_logger().info(f"Connected to ESP32 on {self.P['port']}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to ESP32: {e}")
            raise SystemExit

        self.rx = bytearray()
        self.fw_configured = False
        self.last_cfg_sent = -1e9
        self._last_warn = 0.0

        self.x = self.y = self.theta = 0.0
        self.last_us = self.last_l = self.last_r = None
        self.target_v = self.target_w = self.cur_v = self.cur_w = 0.0
        self.last_cmd_stamp = time.monotonic()
        self.last_ramp = time.monotonic()

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        self.create_timer(0.005, self.read_serial)
        self.create_timer(1.0 / float(self.P['cmd_rate_hz']), self.ramp_and_send)

    # ------------------------------------------------------------ helpers
    def _warn(self, msg):
        if time.monotonic() - self._last_warn > 2.0:
            self.get_logger().warn(msg)
            self._last_warn = time.monotonic()

    def _write(self, s):
        try:
            self.ser.write(s.encode())
        except Exception as e:
            self._warn(f'Serial write failed: {e}')

    def send_config(self):
        """Push every firmware-side setting. Count-checked so a lost line is detected and retried."""
        self.last_cfg_sent = time.monotonic()
        self._write('CFGBEGIN\n')
        for p, (fw, conv) in FW.items():
            time.sleep(0.003)   # don't overrun the ESP32 RX buffer
            self._write(f'SET,{fw},{conv(self.P[p])}\n')
        self._write(f'CFGEND,{len(FW)}\n')

    def _on_params(self, params):
        for p in params:
            if p.name in self.P:
                self.P[p.name] = p.value
                if p.name in FW and self.fw_configured:
                    fw, conv = FW[p.name]
                    self._write(f'SET,{fw},{conv(p.value)}\n')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ cmd_vel -> wheel targets
    def cmd_vel_callback(self, msg):
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z
        self.last_cmd_stamp = time.monotonic()

    @staticmethod
    def _step(cur, tgt, accel, decel, dt):
        slowing = abs(tgt) < abs(cur) or tgt * cur < 0.0
        rate = (decel if slowing else accel) * dt
        return cur + max(-rate, min(rate, tgt - cur))

    def ramp_and_send(self):
        P = self.P
        now = time.monotonic()
        dt = min(max(now - self.last_ramp, 1e-3), 0.1)
        self.last_ramp = now

        if not self.fw_configured and now - self.last_cfg_sent > 1.0:
            self.send_config()

        stale = now - self.last_cmd_stamp > P['cmd_timeout']
        tv = 0.0 if stale else self.target_v
        tw = 0.0 if stale else self.target_w
        self.cur_v = self._step(self.cur_v, tv, P['max_linear_accel'], P['max_linear_decel'], dt)
        self.cur_w = self._step(self.cur_w, tw, P['max_angular_accel'], P['max_angular_decel'], dt)
        if tv == 0.0 and abs(self.cur_v) < P['vel_snap_linear']:
            self.cur_v = 0.0
        if tw == 0.0 and abs(self.cur_w) < P['vel_snap_angular']:
            self.cur_w = 0.0

        # inverse kinematics; per-wheel scale makes both wheels cover equal REAL distance
        half = self.cur_w * P['wheel_separation'] / 2.0
        wl = (self.cur_v - half) / (P['wheel_radius'] * P['left_scale'])
        wr = (self.cur_v + half) / (P['wheel_radius'] * P['right_scale'])
        self._write(f'W,{wl:.4f},{wr:.4f}\n')

    # ------------------------------------------------------------ serial in
    def read_serial(self):
        try:
            n = self.ser.in_waiting
            if n:
                self.rx += self.ser.read(n)
        except Exception as e:
            return self._warn(f'Serial read failed: {e}')
        while True:
            i = self.rx.find(b'\n')
            if i < 0:
                break
            line = bytes(self.rx[:i]).decode('utf-8', 'ignore').strip()
            del self.rx[:i + 1]
            if line.startswith('D,'):
                self.process_telemetry(line)
            elif line.startswith('I,'):
                self.get_logger().info(f'[esp32] {line[2:]}')
        if len(self.rx) > 4096:
            self.rx.clear()

    def process_telemetry(self, line):
        p = line.split(',')
        if len(p) != 17:
            return self._warn(f'Malformed telemetry ({len(p)} fields)')
        try:
            t_us, lt, rt = int(p[1]), int(p[2]), int(p[3])
            q = [float(v) for v in p[4:8]]
            g = [float(v) for v in p[8:11]]
            a = [float(v) for v in p[11:14]]
            flags = int(p[14])
        except ValueError:
            return self._warn('Non-numeric telemetry field')

        self.fw_configured = bool(flags & 1)
        stamp = self.get_clock().now().to_msg()
        if flags & 2:
            self.publish_imu(stamp, q, g, a)

        if self.last_us is None:
            self.last_us, self.last_l, self.last_r = t_us, lt, rt
            return
        dt = ((t_us - self.last_us) & 0xFFFFFFFF) / 1e6      # ESP32 clock: no serial-jitter noise
        dl, dr = lt - self.last_l, rt - self.last_r
        self.last_us, self.last_l, self.last_r = t_us, lt, rt
        P = self.P
        if dt <= 0.0 or dt > P['max_dt']:
            return self._warn(f'Rejected telemetry dt={dt:.3f}s (ESP32 reset?)')
        if abs(dl) > P['max_ticks_per_read'] or abs(dr) > P['max_ticks_per_read']:
            return self._warn(f'Rejected implausible tick delta L={dl} R={dr}')

        # noise deadband ONLY while commanded to stand still (never discards real motion)
        if (self.cur_v == 0.0 and self.cur_w == 0.0
                and abs(dl) <= P['encoder_noise_ticks'] and abs(dr) <= P['encoder_noise_ticks']):
            dl = dr = 0

        per_tick = 2.0 * math.pi * P['wheel_radius'] / P['ticks_per_rev']
        d_left = dl * per_tick * P['left_scale']
        d_right = dr * per_tick * P['right_scale']
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / P['wheel_separation']

        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta = math.atan2(math.sin(self.theta + d_theta), math.cos(self.theta + d_theta))

        m = Odometry()
        m.header.stamp = stamp
        m.header.frame_id = P['odom_frame']
        m.child_frame_id = P['base_frame']
        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        m.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        m.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        m.twist.twist.linear.x = d_center / dt
        m.twist.twist.angular.z = d_theta / dt
        m.pose.covariance[0] = P['cov_pos']
        m.pose.covariance[7] = P['cov_pos']
        m.pose.covariance[14] = m.pose.covariance[21] = m.pose.covariance[28] = 1e6
        m.pose.covariance[35] = P['cov_yaw']
        m.twist.covariance[0] = P['cov_vx']
        m.twist.covariance[35] = P['cov_wz']
        self.odom_pub.publish(m)

    def publish_imu(self, stamp, q, g, a):
        P = self.P
        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = P['imu_frame']
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
        m.angular_velocity.x = g[0] - P['gyro_bias_x']
        m.angular_velocity.y = g[1] - P['gyro_bias_y']
        m.angular_velocity.z = g[2] - P['gyro_bias_z']
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = a
        for name, key in (('orientation_covariance', 'cov_imu_orient'),
                          ('angular_velocity_covariance', 'cov_imu_gyro'),
                          ('linear_acceleration_covariance', 'cov_imu_accel')):
            c = float(P[key])
            setattr(m, name, [c, 0.0, 0.0, 0.0, c, 0.0, 0.0, 0.0, c])
        self.imu_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ESP32Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._write('W,0.0,0.0\n')
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
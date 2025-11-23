#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RB-35GM 11TYPE (Encoder 26P/R) + 감속비 고려
- 라즈베리파이4 + Ubuntu 22.04 + ROS 2 Humble
- RPi.GPIO 사용
- 쿼드러처 X4 디코딩 엔코더
- /cmd_vel 기반 속도 명령
- 엔코더 + PID로 바퀴 속도 제어
- /imu/data 구독해서 pitch 기반 경사 보정(feed-forward) 옵션
"""

import RPi.GPIO as GPIO
import time
import math
from threading import Lock

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

# ==================== 하드웨어 핀 정의 (BOARD 모드) ====================
MOTOR1_DIR = 40
MOTOR1_PWM = 32
MOTOR1_EN  = 37
MOTOR1_HALL_A = 35
MOTOR1_HALL_B = 36

MOTOR2_DIR = 18
MOTOR2_PWM = 12
MOTOR2_EN  = 15
MOTOR2_HALL_A = 13
MOTOR2_HALL_B = 16

# ==================== 엔코더/기구 파라미터 ====================
ENCODER_PPR    = 26.0      # 모터축 P/R
GEAR_RATIO     = 60.0      # ★ 실제 감속비로 수정
WHEEL_DIAMETER = 0.115     # [m] 바퀴 지름 (11.5cm)

MOTOR_TICKS_PER_REV = ENCODER_PPR * 4.0
TICKS_PER_REV       = MOTOR_TICKS_PER_REV * GEAR_RATIO
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER

# 차체 파라미터 (좌우 바퀴 간 거리)
WHEEL_BASE = 0.30   # [m] 실제 차체에 맞게 수정


class PID:
    def __init__(self, kp, ki, kd, out_min=0.0, out_max=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True

    def update(self, target, measurement, dt):
        error = target - measurement

        p = self.kp * error
        self.integral += error * dt
        i = self.ki * self.integral

        if self.first or dt <= 0.0:
            d = 0.0
            self.first = False
        else:
            d = self.kd * (error - self.prev_error) / dt

        self.prev_error = error

        out = p + i + d
        if out > self.out_max:
            out = self.out_max
        elif out < self.out_min:
            out = self.out_min
        return out


def decode_x4(prev_a, prev_b, curr_a, curr_b):
    delta = 0
    direction = 0

    # A 엣지
    if prev_a == 0 and curr_a == 1:      # rising
        delta = 1
        direction = 1 if curr_b == 0 else -1
    elif prev_a == 1 and curr_a == 0:    # falling
        delta = 1
        direction = 1 if curr_b == 1 else -1

    # B 엣지
    elif prev_b == 0 and curr_b == 1:
        delta = 1
        direction = 1 if curr_a == 1 else -1
    elif prev_b == 1 and curr_b == 0:
        delta = 1
        direction = 1 if curr_a == 0 else -1

    return delta, direction


class RB35DriverNode(Node):
    def __init__(self):
        super().__init__('motor')

        # ===== 파라미터 =====
        self.declare_parameter('control_hz', 50.0)       # PID 주기 [Hz]
        self.declare_parameter('base_pwm', 20.0)         # 최소 구동 PWM
        self.declare_parameter('max_pwm', 100.0)         # 최대 PWM
        self.declare_parameter('kp', 100.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('debug', False)
        self.declare_parameter('use_imu_pitch_ff', False)
        self.declare_parameter('pitch_k', 0.0)           # 경사 feed-forward gain

        control_hz = float(self.get_parameter('control_hz').value)
        self.base_pwm = float(self.get_parameter('base_pwm').value)
        self.max_pwm = float(self.get_parameter('max_pwm').value)
        kp = float(self.get_parameter('kp').value)
        ki = float(self.get_parameter('ki').value)
        kd = float(self.get_parameter('kd').value)
        self.debug = bool(self.get_parameter('debug').value)
        self.use_imu_pitch_ff = bool(self.get_parameter('use_imu_pitch_ff').value)
        self.pitch_k = float(self.get_parameter('pitch_k').value)

        self.dt = 1.0 / control_hz

        self.get_logger().info(f"RB35DriverNode started (control_hz={control_hz})")
        self.get_logger().info(
            f"TICKS_PER_REV={TICKS_PER_REV}, wheel_circ={WHEEL_CIRCUMFERENCE:.4f} m"
        )

        # ===== 내부 상태 =====
        self.encoder_lock = Lock()
        self.m1_count = 0
        self.m2_count = 0
        self.m1_count_prev = 0
        self.m2_count_prev = 0
        self.m1_dir = 0
        self.m2_dir = 0

        self.m1_prev_a = 0
        self.m1_prev_b = 0
        self.m2_prev_a = 0
        self.m2_prev_b = 0

        self.target_v = 0.0     # [m/s]
        self.target_w = 0.0     # [rad/s]
        self.target_v_left = 0.0
        self.target_v_right = 0.0

        self.meas_v_left = 0.0
        self.meas_v_right = 0.0

        self.pwm_left = 0.0
        self.pwm_right = 0.0

        self.last_pitch = None  # [rad]

        self.pid_left = PID(kp, ki, kd, out_min=0.0, out_max=self.max_pwm - self.base_pwm)
        self.pid_right = PID(kp, ki, kd, out_min=0.0, out_max=self.max_pwm - self.base_pwm)

        # GPIO 세팅
        self.setup_gpio()

        # ROS 통신
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # 제어 루프 타이머
        self.control_timer = self.create_timer(self.dt, self.control_loop)

    # ===== GPIO 초기화 =====
    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        GPIO.setup(MOTOR1_DIR, GPIO.OUT)
        GPIO.setup(MOTOR1_PWM, GPIO.OUT)
        GPIO.setup(MOTOR1_EN,  GPIO.OUT)
        GPIO.setup(MOTOR1_HALL_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(MOTOR1_HALL_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.setup(MOTOR2_DIR, GPIO.OUT)
        GPIO.setup(MOTOR2_PWM, GPIO.OUT)
        GPIO.setup(MOTOR2_EN,  GPIO.OUT)
        GPIO.setup(MOTOR2_HALL_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(MOTOR2_HALL_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.pwm1 = GPIO.PWM(MOTOR1_PWM, 1000)
        self.pwm2 = GPIO.PWM(MOTOR2_PWM, 1000)
        self.pwm1.start(0.0)
        self.pwm2.start(0.0)

        self.m1_prev_a = GPIO.input(MOTOR1_HALL_A)
        self.m1_prev_b = GPIO.input(MOTOR1_HALL_B)
        self.m2_prev_a = GPIO.input(MOTOR2_HALL_A)
        self.m2_prev_b = GPIO.input(MOTOR2_HALL_B)

        GPIO.add_event_detect(MOTOR1_HALL_A, GPIO.BOTH, callback=self.motor1_callback, bouncetime=1)
        GPIO.add_event_detect(MOTOR1_HALL_B, GPIO.BOTH, callback=self.motor1_callback, bouncetime=1)
        GPIO.add_event_detect(MOTOR2_HALL_A, GPIO.BOTH, callback=self.motor2_callback, bouncetime=1)
        GPIO.add_event_detect(MOTOR2_HALL_B, GPIO.BOTH, callback=self.motor2_callback, bouncetime=1)

        GPIO.output(MOTOR1_EN, GPIO.LOW)
        GPIO.output(MOTOR2_EN, GPIO.LOW)
        GPIO.output(MOTOR1_DIR, GPIO.LOW)
        GPIO.output(MOTOR2_DIR, GPIO.LOW)

        self.get_logger().info("GPIO initialized (BOARD mode)")

    # ===== 엔코더 콜백 =====
    def motor1_callback(self, channel):
        curr_a = GPIO.input(MOTOR1_HALL_A)
        curr_b = GPIO.input(MOTOR1_HALL_B)
        delta, direction = decode_x4(self.m1_prev_a, self.m1_prev_b, curr_a, curr_b)

        if delta:
            with self.encoder_lock:
                self.m1_count += 1
                self.m1_dir = direction

        self.m1_prev_a = curr_a
        self.m1_prev_b = curr_b

    def motor2_callback(self, channel):
        curr_a = GPIO.input(MOTOR2_HALL_A)
        curr_b = GPIO.input(MOTOR2_HALL_B)
        delta, direction = decode_x4(self.m2_prev_a, self.m2_prev_b, curr_a, curr_b)

        if delta:
            with self.encoder_lock:
                self.m2_count += 1
                self.m2_dir = direction

        self.m2_prev_a = curr_a
        self.m2_prev_b = curr_b

    # ===== ROS 콜백 =====
    def cmd_vel_callback(self, msg: Twist):
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z

        self.target_v_left  = self.target_v - self.target_w * (WHEEL_BASE / 2.0)
        self.target_v_right = self.target_v + self.target_w * (WHEEL_BASE / 2.0)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        w, x, y, z = q.w, q.x, q.y, q.z

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        _roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        self.last_pitch = pitch

    # ===== 제어 루프 =====
    def control_loop(self):
        with self.encoder_lock:
            m1 = self.m1_count
            m2 = self.m2_count
            dir1 = self.m1_dir
            dir2 = self.m2_dir

        delta1 = m1 - self.m1_count_prev
        delta2 = m2 - self.m2_count_prev
        self.m1_count_prev = m1
        self.m2_count_prev = m2

        rev1 = delta1 / TICKS_PER_REV / self.dt
        rev2 = delta2 / TICKS_PER_REV / self.dt

        self.meas_v_left  = rev1 * WHEEL_CIRCUMFERENCE * (1 if dir1 >= 0 else -1)
        self.meas_v_right = rev2 * WHEEL_CIRCUMFERENCE * (1 if dir2 >= 0 else -1)

        cmd_left_mag  = self.pid_left.update(self.target_v_left,  self.meas_v_left,  self.dt)
        cmd_right_mag = self.pid_right.update(self.target_v_right, self.meas_v_right, self.dt)

        pwm_left  = self.base_pwm + cmd_left_mag
        pwm_right = self.base_pwm + cmd_right_mag

        if pwm_left > self.max_pwm:
            pwm_left = self.max_pwm
        if pwm_right > self.max_pwm:
            pwm_right = self.max_pwm

        # IMU pitch 기반 feed-forward (옵션)
        if self.use_imu_pitch_ff and self.last_pitch is not None:
            slope_term = self.pitch_k * math.sin(self.last_pitch)
            sign_left  = 1.0 if self.target_v_left  >= 0.0 else -1.0
            sign_right = 1.0 if self.target_v_right >= 0.0 else -1.0
            pwm_left  += slope_term * sign_left
            pwm_right += slope_term * sign_right
            pwm_left  = max(0.0, min(self.max_pwm, pwm_left))
            pwm_right = max(0.0, min(self.max_pwm, pwm_right))

        self.pwm_left = pwm_left
        self.pwm_right = pwm_right

        self.apply_motor_output(self.target_v_left, self.pwm_left,
                                self.target_v_right, self.pwm_right)

        if self.debug:
            self.get_logger().info(
                f"vL={self.meas_v_left:.3f}/{self.target_v_left:.3f}  "
                f"vR={self.meas_v_right:.3f}/{self.target_v_right:.3f}  "
                f"PWM L/R={self.pwm_left:.1f}/{self.pwm_right:.1f}"
            )

    def apply_motor_output(self, v_left, pwm_left, v_right, pwm_right):
        # 왼쪽 모터
        if abs(v_left) < 1e-3:
            GPIO.output(MOTOR1_EN, GPIO.LOW)
            self.pwm1.ChangeDutyCycle(0.0)
        else:
            GPIO.output(MOTOR1_EN, GPIO.LOW)  # 보드에 따라 HIGH가 Enable일 수도 있음
            if v_left >= 0.0:
                GPIO.output(MOTOR1_DIR, GPIO.LOW)
            else:
                GPIO.output(MOTOR1_DIR, GPIO.HIGH)
            self.pwm1.ChangeDutyCycle(pwm_left)

        # 오른쪽 모터
        if abs(v_right) < 1e-3:
            GPIO.output(MOTOR2_EN, GPIO.LOW)
            self.pwm2.ChangeDutyCycle(0.0)
        else:
            GPIO.output(MOTOR2_EN, GPIO.LOW)
            if v_right >= 0.0:
                GPIO.output(MOTOR2_DIR, GPIO.LOW)
            else:
                GPIO.output(MOTOR2_DIR, GPIO.HIGH)
            self.pwm2.ChangeDutyCycle(pwm_right)

    # ===== 종료 처리 =====
    def cleanup(self):
        try:
            self.pwm1.stop()
            self.pwm2.stop()
        except Exception:
            pass
        GPIO.cleanup()
        self.get_logger().info("GPIO cleaned up")

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RB35DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


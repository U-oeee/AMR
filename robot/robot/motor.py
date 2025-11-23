#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
motor.py
- RB-35GM 11TYPE (Encoder 26P/R) + 감속비 고려
- 라즈베리파이4 + Ubuntu 22.04 + ROS 2 Humble
- RPi.GPIO + ROS2 Node
- /cmd_vel 기반으로 좌우 바퀴 속도 제어
- 평지에서는 base_pwm로 주행, 경사에서는 PID로 평지 기준 속도 유지
"""

import math
from threading import Lock

import RPi.GPIO as GPIO

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
        self.declare_parameter('control_hz', 50.0)        # 제어 주기 [Hz]
        self.declare_parameter('base_pwm', 30.0)          # 평지 기본 PWM[%]
        self.declare_parameter('max_pwm_left', 100.0)     # 왼쪽 최대 PWM
        self.declare_parameter('max_pwm_right', 60.0)     # 오른쪽 최대 PWM (조금 작은 값)
        self.declare_parameter('kp', 300.0)
        self.declare_parameter('ki', 5.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('pitch_slope_deg', 3.0)    # 이 각도 이상이면 경사
        self.declare_parameter('flat_speed_alpha', 0.1)   # 평지 속도 EMA 계수
        self.declare_parameter('debug', True)

        control_hz       = float(self.get_parameter('control_hz').value)
        self.base_pwm    = float(self.get_parameter('base_pwm').value)
        self.max_pwm_L   = float(self.get_parameter('max_pwm_left').value)
        self.max_pwm_R   = float(self.get_parameter('max_pwm_right').value)
        kp               = float(self.get_parameter('kp').value)
        ki               = float(self.get_parameter('ki').value)
        kd               = float(self.get_parameter('kd').value)
        pitch_slope_deg  = float(self.get_parameter('pitch_slope_deg').value)
        self.flat_alpha  = float(self.get_parameter('flat_speed_alpha').value)
        self.debug       = bool(self.get_parameter('debug').value)

        self.dt = 1.0 / control_hz
        self.pitch_thresh_rad = math.radians(pitch_slope_deg)

        self.get_logger().info(
            f"RB35DriverNode started (control_hz={control_hz})"
        )
        self.get_logger().info(
            f"TICKS_PER_REV={TICKS_PER_REV}, wheel_circ={WHEEL_CIRCUMFERENCE:.4f} m, "
            f"base_pwm={self.base_pwm}%, max_pwm_L/R={self.max_pwm_L}/{self.max_pwm_R}"
        )

        # ===== 내부 상태 =====
        self.encoder_lock = Lock()
        self.m1_count = 0
        self.m2_count = 0
        self.m1_prev_count = 0
        self.m2_prev_count = 0
        self.m1_dir = 0
        self.m2_dir = 0
        self.m1_prev_a = 0
        self.m1_prev_b = 0
        self.m2_prev_a = 0
        self.m2_prev_b = 0

        # /cmd_vel에서 오는 목표 속도
        self.target_v = 0.0
        self.target_w = 0.0
        self.target_v_left = 0.0
        self.target_v_right = 0.0

        # 실제 속도
        self.meas_v_left = 0.0
        self.meas_v_right = 0.0

        # PWM 출력
        self.pwm_left = 0.0
        self.pwm_right = 0.0

        # IMU pitch
        self.last_pitch = None

        # 평지 기준 속도 (base_pwm에서의 속도)
        self.flat_v_left_ref = 0.0
        self.flat_v_right_ref = 0.0
        self.flat_ref_initialized = False

        # PID
        self.pid_left = PID(kp, ki, kd,
                            out_min=0.0,
                            out_max=self.max_pwm_L - self.base_pwm)
        self.pid_right = PID(kp, ki, kd,
                             out_min=0.0,
                             out_max=self.max_pwm_R - self.base_pwm)

        # GPIO 세팅
        self.setup_gpio()

        # ROS 통신
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel',
                                                self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data',
                                                self.imu_callback, 10)

        # 제어 루프 타이머
        self.control_timer = self.create_timer(self.dt, self.control_loop)

    # ---------- GPIO / 엔코더 ----------
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

        GPIO.add_event_detect(MOTOR1_HALL_A, GPIO.BOTH,
                              callback=self.motor1_callback, bouncetime=1)
        GPIO.add_event_detect(MOTOR1_HALL_B, GPIO.BOTH,
                              callback=self.motor1_callback, bouncetime=1)
        GPIO.add_event_detect(MOTOR2_HALL_A, GPIO.BOTH,
                              callback=self.motor2_callback, bouncetime=1)
        GPIO.add_event_detect(MOTOR2_HALL_B, GPIO.BOTH,
                              callback=self.motor2_callback, bouncetime=1)

        GPIO.output(MOTOR1_EN, GPIO.LOW)
        GPIO.output(MOTOR2_EN, GPIO.LOW)
        GPIO.output(MOTOR1_DIR, GPIO.LOW)
        GPIO.output(MOTOR2_DIR, GPIO.LOW)

        self.get_logger().info("GPIO initialized (BOARD mode)")

    def motor1_callback(self, channel):
        curr_a = GPIO.input(MOTOR1_HALL_A)
        curr_b = GPIO.input(MOTOR1_HALL_B)
        delta, direction = decode_x4(self.m1_prev_a, self.m1_prev_b,
                                     curr_a, curr_b)
        if delta:
            with self.encoder_lock:
                self.m1_count += 1
                self.m1_dir = direction
        self.m1_prev_a = curr_a
        self.m1_prev_b = curr_b

    def motor2_callback(self, channel):
        curr_a = GPIO.input(MOTOR2_HALL_A)
        curr_b = GPIO.input(MOTOR2_HALL_B)
        delta, direction = decode_x4(self.m2_prev_a, self.m2_prev_b,
                                     curr_a, curr_b)
        if delta:
            with self.encoder_lock:
                self.m2_count += 1
                self.m2_dir = direction
        self.m2_prev_a = curr_a
        self.m2_prev_b = curr_b

    # ---------- ROS 콜백 ----------
    def cmd_vel_callback(self, msg: Twist):
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z
        self.target_v_left  = self.target_v - self.target_w * (WHEEL_BASE / 2.0)
        self.target_v_right = self.target_v + self.target_w * (WHEEL_BASE / 2.0)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        w, x, y, z = q.w, q.x, q.y, q.z

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
        self.last_pitch = pitch

    # ---------- 제어 루프 ----------
    def control_loop(self):
        # 1) 엔코더로 현재 속도 계산
        with self.encoder_lock:
            m1 = self.m1_count
            m2 = self.m2_count
            d1 = self.m1_dir
            d2 = self.m2_dir

        delta1 = m1 - self.m1_prev_count
        delta2 = m2 - self.m2_prev_count
        self.m1_prev_count = m1
        self.m2_prev_count = m2

        rev1 = delta1 / TICKS_PER_REV / self.dt
        rev2 = delta2 / TICKS_PER_REV / self.dt

        self.meas_v_left  = rev1 * WHEEL_CIRCUMFERENCE * (1 if d1 >= 0 else -1)
        self.meas_v_right = rev2 * WHEEL_CIRCUMFERENCE * (1 if d2 >= 0 else -1)

        # 2) 경사 여부
        slope_mode = False
        if self.last_pitch is not None and \
           abs(self.last_pitch) >= self.pitch_thresh_rad:
            slope_mode = True

        if self.debug and self.last_pitch is not None:
            self.get_logger().info(
                f"[IMU] pitch={math.degrees(self.last_pitch):.2f} deg"
            )

        # === cmd_vel이 거의 0 이면 완전 정지 ===
        if abs(self.target_v) < 1e-3 and abs(self.target_w) < 1e-3:
            self.pwm_left = 0.0
            self.pwm_right = 0.0
            self.apply_motor_output(0.0, 0.0)
            if self.debug:
                self.get_logger().info("[STOP] cmd_vel ~ 0 -> PWM=0")
            return

        # === 평지 모드: base_pwm로만 주행 + 기준 속도 학습 ===
        if not slope_mode:
            self.pid_left.reset()
            self.pid_right.reset()

            # 방향은 target_v_left/right 부호만 사용
            dirL = 1.0 if self.target_v_left  >= 0.0 else -1.0
            dirR = 1.0 if self.target_v_right >= 0.0 else -1.0

            self.pwm_left = self.base_pwm
            self.pwm_right = self.base_pwm
            self.apply_motor_output(dirL, dirR)

            vL_abs = abs(self.meas_v_left)
            vR_abs = abs(self.meas_v_right)

            if vL_abs > 1e-3:
                if not self.flat_ref_initialized:
                    self.flat_v_left_ref = vL_abs
                else:
                    self.flat_v_left_ref = (
                        (1.0 - self.flat_alpha) * self.flat_v_left_ref +
                        self.flat_alpha * vL_abs
                    )
            if vR_abs > 1e-3:
                if not self.flat_ref_initialized:
                    self.flat_v_right_ref = vR_abs
                else:
                    self.flat_v_right_ref = (
                        (1.0 - self.flat_alpha) * self.flat_v_right_ref +
                        self.flat_alpha * vR_abs
                    )

            if vL_abs > 1e-3 or vR_abs > 1e-3:
                self.flat_ref_initialized = True

            if self.debug:
                self.get_logger().info(
                    f"[FLAT] vL={self.meas_v_left:.3f}, vR={self.meas_v_right:.3f}, "
                    f"flat_ref L/R={self.flat_v_left_ref:.3f}/{self.flat_v_right_ref:.3f}, "
                    f"PWM L/R={self.pwm_left:.1f}/{self.pwm_right:.1f}"
                )
            return

        # === 경사 모드: 평지 기준 속도 유지 위해 PID 사용 ===
        if not self.flat_ref_initialized:
            self.flat_v_left_ref = abs(self.meas_v_left)
            self.flat_v_right_ref = abs(self.meas_v_right)
            self.flat_ref_initialized = True

        dirL = 1.0 if self.target_v_left  >= 0.0 else -1.0
        dirR = 1.0 if self.target_v_right >= 0.0 else -1.0

        target_vL = dirL * self.flat_v_left_ref
        target_vR = dirR * self.flat_v_right_ref

        cmdL = self.pid_left.update(target_vL, self.meas_v_left,  self.dt)
        cmdR = self.pid_right.update(target_vR, self.meas_v_right, self.dt)

        pwm_left  = self.base_pwm + cmdL
        pwm_right = self.base_pwm + cmdR

        pwm_left  = max(0.0, min(self.max_pwm_L, pwm_left))
        pwm_right = max(0.0, min(self.max_pwm_R, pwm_right))

        self.pwm_left = pwm_left
        self.pwm_right = pwm_right
        self.apply_motor_output(dirL, dirR)

        if self.debug:
            self.get_logger().info(
                f"[SLOPE] pitch={math.degrees(self.last_pitch):.2f} deg | "
                f"vL={self.meas_v_left:.3f}/{target_vL:.3f}, "
                f"vR={self.meas_v_right:.3f}/{target_vR:.3f}, "
                f"PWM L/R={self.pwm_left:.1f}/{self.pwm_right:.1f}"
            )

    def apply_motor_output(self, dir_left: float, dir_right: float):
        # 왼쪽
        if abs(dir_left) < 1e-3 or self.pwm_left <= 0.0:
            GPIO.output(MOTOR1_EN, GPIO.LOW)
            self.pwm1.ChangeDutyCycle(0.0)
        else:
            GPIO.output(MOTOR1_EN, GPIO.LOW)  # 보드에 따라 HIGH일 수도 있음
            GPIO.output(MOTOR1_DIR, GPIO.LOW if dir_left >= 0 else GPIO.HIGH)
            self.pwm1.ChangeDutyCycle(self.pwm_left)

        # 오른쪽
        if abs(dir_right) < 1e-3 or self.pwm_right <= 0.0:
            GPIO.output(MOTOR2_EN, GPIO.LOW)
            self.pwm2.ChangeDutyCycle(0.0)
        else:
            GPIO.output(MOTOR2_EN, GPIO.LOW)
            GPIO.output(MOTOR2_DIR, GPIO.LOW if dir_right >= 0 else GPIO.HIGH)
            self.pwm2.ChangeDutyCycle(self.pwm_right)

    # ---------- 종료 처리 ----------
    def cleanup(self):
        try:
            GPIO.remove_event_detect(MOTOR1_HALL_A)
            GPIO.remove_event_detect(MOTOR1_HALL_B)
            GPIO.remove_event_detect(MOTOR2_HALL_A)
            GPIO.remove_event_detect(MOTOR2_HALL_B)
        except Exception:
            pass

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

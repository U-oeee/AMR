#!/usr/bin/env python3
import rclpy, math, serial
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

def to_floats(parts):
    out=[]
    for p in parts:
        try:
            out.append(float(p))
        except:
            return None
    return out

class Ximu3Publisher(Node):
    def __init__(self):
        super().__init__('ximu3_publisher')
        # Params
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'imu_link')
        self.declare_parameter('publish_tf', False)   # imu_link는 고정 프레임
        self.declare_parameter('debug', False)
        self.declare_parameter('gyro_is_deg', False)  # deg/s → rad/s
        self.declare_parameter('acc_is_g', False)     # g → m/s^2
        self.declare_parameter('normalize_orientation', True)

        self.serial_port  = self.get_parameter('serial_port').value
        self.serial_baud  = int(self.get_parameter('serial_baud').value)
        self.frame_id     = self.get_parameter('frame_id').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame  = self.get_parameter('child_frame').value
        self.publish_tf   = bool(self.get_parameter('publish_tf').value)
        self.debug        = bool(self.get_parameter('debug').value)
        self.gyro_is_deg  = bool(self.get_parameter('gyro_is_deg').value)
        self.acc_is_g     = bool(self.get_parameter('acc_is_g').value)
        self.norm_quat    = bool(self.get_parameter('normalize_orientation').value)

        # QoS: 더 안정적인 구독을 위해 RELIABLE + depth 50
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.pub = self.create_publisher(Imu, '/imu/data', qos)

        # Serial
        self.ser = serial.Serial(self.serial_port, self.serial_baud, timeout=0.05)
        self.get_logger().info(f"[ximu3] Listening on {self.serial_port} @ {self.serial_baud}")

        # Buffers
        self.last_quat = None  # (w,x,y,z)
        self.last_gyr  = None  # (gx,gy,gz) rad/s
        self.last_acc  = None  # (ax,ay,az) m/s^2

        self.create_timer(0.005, self.read_serial)  # ~200Hz

    def read_serial(self):
        line = self.ser.readline().decode(errors='ignore').strip()
        if not line:
            return
        if self.debug:
            self.get_logger().info(f"RAW: {line}")

        parts = [p.strip() for p in line.split(',')]
        if len(parts) < 2:
            return

        tag = parts[0]

        # Q, ts, qw, qx, qy, qz
        if tag == 'Q':
            nums = to_floats(parts[1:])
            if not nums or len(nums) < 5:
                return
            qw, qx, qy, qz = nums[1], nums[2], nums[3], nums[4]
            self.last_quat = (qw, qx, qy, qz)

        # I, ts, gx, gy, gz, ax, ay, az
        elif tag == 'I':
            nums = to_floats(parts[1:])
            if not nums or len(nums) < 7:
                return
            gx, gy, gz = nums[1], nums[2], nums[3]
            ax, ay, az = nums[4], nums[5], nums[6]
            if self.gyro_is_deg:
                gx, gy, gz = map(math.radians, (gx, gy, gz))
            if self.acc_is_g:
                g = 9.80665
                ax, ay, az = ax*g, ay*g, az*g
            self.last_gyr = (gx, gy, gz)
            self.last_acc = (ax, ay, az)

        # 하나라도 갱신되면 퍼블리시
        if self.last_quat or self.last_gyr or self.last_acc:
            self.publish_imu()

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        if self.last_quat:
            w,x,y,z = self.last_quat
            if self.norm_quat:
                n = math.sqrt(w*w + x*x + y*y + z*z)
                if n > 0:
                    w,x,y,z = w/n, x/n, y/n, z/n
            msg.orientation = Quaternion(w=float(w), x=float(x), y=float(y), z=float(z))
            msg.orientation_covariance = [
                0.02, 0.0, 0.0,
                0.0,  0.02, 0.0,
                0.0,  0.0,  0.02
            ]
        else:
            msg.orientation_covariance[0] = -1.0

        if self.last_gyr:
            gx,gy,gz = self.last_gyr
            msg.angular_velocity.x = float(gx)
            msg.angular_velocity.y = float(gy)
            msg.angular_velocity.z = float(gz)
            msg.angular_velocity_covariance = [
                0.0025, 0.0,    0.0,
                0.0,    0.0025, 0.0,
                0.0,    0.0,    0.0025
            ]
        else:
            msg.angular_velocity_covariance[0] = -1.0

        if self.last_acc:
            ax,ay,az = self.last_acc
            msg.linear_acceleration.x = float(ax)
            msg.linear_acceleration.y = float(ay)
            msg.linear_acceleration.z = float(az)
            msg.linear_acceleration_covariance = [
                0.04, 0.0, 0.0,
                0.0, 0.04, 0.0,
                0.0, 0.0, 0.04
            ]
        else:
            msg.linear_acceleration_covariance[0] = -1.0

        self.pub.publish(msg)

def main():
    rclpy.init()
    n = Ximu3Publisher()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


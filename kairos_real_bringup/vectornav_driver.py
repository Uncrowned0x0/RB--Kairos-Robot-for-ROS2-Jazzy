#!/usr/bin/env python3
# Copyright 2026 Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
VectorNav VN-100 / IMU ROS 2 Driver.

Reads high-speed telemetry from /dev/ttyUSB0 (or /dev/ttyUSB_IMU) at 921600 baud,
decodes binary frames (MAVLink / VectorNav), applies the mounting rotation matrix
R_ref = diag(1, -1, -1), and publishes standard sensor_msgs/msg/Imu on /imu/data.

Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
"""

import math
import os
import struct
import threading
import time

from geometry_msgs.msg import Quaternion
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
import serial


def x25_crc(buf, crc=0xFFFF):
    """Compute standard X.25 CRC-16 used by MAVLink."""
    for b in buf:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ ((tmp << 4) & 0xFF)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


CRC_EXTRAS = {
    30: 39,   # ATTITUDE
    31: 246,  # ATTITUDE_QUATERNION
    105: 93,  # HIGHRES_IMU
}


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert roll, pitch, yaw (rad) to geometry_msgs Quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class VectorNavDriver(Node):
    """High-frequency ROS 2 Driver for VectorNav VN-100 IMU."""

    def __init__(self):
        super().__init__('vectornav_driver')

        default_port = '/dev/ttyUSB_IMU' if os.path.exists('/dev/ttyUSB_IMU') else '/dev/ttyUSB0'

        self.declare_parameter('serial_port', default_port)
        self.declare_parameter('serial_baud', 921600)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('tf_ned_to_enu', True)

        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = self.get_parameter('serial_baud').value
        self.frame_id = self.get_parameter('frame_id').value
        self.tf_ned_to_enu = self.get_parameter('tf_ned_to_enu').value

        # Covariance matrices (diagonal defaults)
        self.orientation_cov = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01
        ]
        self.angular_vel_cov = [
            0.001, 0.0,   0.0,
            0.0,   0.001, 0.0,
            0.0,   0.0,   0.001
        ]
        self.linear_accel_cov = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01
        ]

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 20)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.temp_pub = self.create_publisher(Temperature, '/imu/temperature', 10)

        # State cache for fusion into single Imu message
        self.lock = threading.Lock()
        self.cur_q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.has_quat = False
        self.cur_gyro = [0.0, 0.0, 0.0]
        self.cur_accel = [0.0, 0.0, 9.81]
        self.msg_count = 0
        self.last_log_time = 0.0

        self.running = True
        self.serial_conn = None

        self.rx_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(
            f'VectorNav driver initialized on {self.serial_port} @ {self.serial_baud} baud '
            f'(frame_id: {self.frame_id})'
        )

    def _open_serial(self):
        """Open or reopen serial port with fallback."""
        ports_to_try = [self.serial_port]
        if self.serial_port != '/dev/ttyUSB0' and os.path.exists('/dev/ttyUSB0'):
            ports_to_try.append('/dev/ttyUSB0')
        if os.path.exists('/dev/ttyUSB_IMU') and '/dev/ttyUSB_IMU' not in ports_to_try:
            ports_to_try.insert(0, '/dev/ttyUSB_IMU')

        for port in ports_to_try:
            try:
                s = serial.Serial(
                    port=port,
                    baudrate=self.serial_baud,
                    timeout=1.0,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
                self.get_logger().info(f'Successfully connected to IMU on {port}')
                return s
            except Exception as e:
                self.get_logger().warn(f'Failed connecting to {port}: {e}')
        return None

    def _serial_worker(self):
        """Continuous background serial receiver and decoder."""
        buf = bytearray()

        while self.running and rclpy.ok():
            if self.serial_conn is None or not self.serial_conn.is_open:
                self.serial_conn = self._open_serial()
                if self.serial_conn is None:
                    time.sleep(2.0)
                    continue

            try:
                chunk = self.serial_conn.read(512)
                if not chunk:
                    continue
                buf.extend(chunk)

                # Keep buffer reasonable
                if len(buf) > 4096:
                    buf = buf[-2048:]

                # Process all valid frames in buffer
                idx = 0
                while idx < len(buf) - 8:
                    sync = buf[idx]

                    # MAVLink frame (0xFE)
                    if sync == 0xFE:
                        payload_len = buf[idx + 1]
                        msg_id = buf[idx + 5]
                        pkt_len = 6 + payload_len + 2

                        if idx + pkt_len <= len(buf):
                            pkt_data = buf[idx + 1:idx + 6 + payload_len]
                            crc_bytes = buf[idx + 6 + payload_len:idx + pkt_len]
                            recv_crc = struct.unpack('<H', crc_bytes)[0]

                            valid = False
                            if msg_id in CRC_EXTRAS:
                                c = x25_crc(pkt_data, 0xFFFF)
                                c = x25_crc([CRC_EXTRAS[msg_id]], c)
                                if c == recv_crc:
                                    valid = True

                            if valid:
                                payload = buf[idx + 6:idx + 6 + payload_len]
                                self._handle_mavlink(msg_id, payload)
                                idx += pkt_len
                                continue
                            else:
                                idx += 1
                                continue
                        else:
                            # Incomplete frame, wait for more bytes
                            break

                    # Native VectorNav frame (0xFA)
                    elif sync == 0xFA and idx < len(buf) - 4:
                        groups_present = buf[idx + 1]
                        # Rough check if valid groups bitmask
                        if 0 < groups_present < 0x80:
                            # Try VectorNav native parsing
                            consumed = self._try_parse_vn_native(buf[idx:])
                            if consumed > 0:
                                idx += consumed
                                continue
                        idx += 1
                    else:
                        idx += 1

                # Discard processed bytes
                if idx > 0:
                    del buf[:idx]

            except serial.SerialException as e:
                self.get_logger().error(f'Serial exception: {e}. Reconnecting...')
                if self.serial_conn:
                    try:
                        self.serial_conn.close()
                    except Exception:
                        pass
                self.serial_conn = None
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().debug(f'Parser loop error: {e}')

    def _handle_mavlink(self, msg_id: int, payload: bytes):
        """Dispatch MAVLink messages (HIGHRES_IMU, ATTITUDE_QUATERNION, ATTITUDE)."""
        now = self.get_clock().now()

        # Msg 105: HIGHRES_IMU
        if msg_id == 105 and len(payload) >= 62:
            raw_vals = struct.unpack('<QfffffffffffffH', payload[:62])
            (t_usec, ax, ay, az, gx, gy, gz,
             mx, my, mz, pres, diff_pres, alt, temp, fields) = raw_vals

            # Apply constructor rotation matrix R_ref = diag(1, -1, -1) to convert NED to ENU:
            # ax' = ax, ay' = -ay, az' = -az
            # gx' = gx, gy' = -gy, gz' = -gz
            if self.tf_ned_to_enu:
                ax_enu, ay_enu, az_enu = ax, -ay, -az
                gx_enu, gy_enu, gz_enu = gx, -gy, -gz
            else:
                ax_enu, ay_enu, az_enu = ax, ay, az
                gx_enu, gy_enu, gz_enu = gx, gy, gz

            with self.lock:
                self.cur_accel = [ax_enu, ay_enu, az_enu]
                self.cur_gyro = [gx_enu, gy_enu, gz_enu]

            # Publish Imu message
            imu_msg = Imu()
            imu_msg.header.stamp = now.to_msg()
            imu_msg.header.frame_id = self.frame_id

            imu_msg.linear_acceleration.x = float(ax_enu)
            imu_msg.linear_acceleration.y = float(ay_enu)
            imu_msg.linear_acceleration.z = float(az_enu)
            imu_msg.linear_acceleration_covariance = self.linear_accel_cov

            imu_msg.angular_velocity.x = float(gx_enu)
            imu_msg.angular_velocity.y = float(gy_enu)
            imu_msg.angular_velocity.z = float(gz_enu)
            imu_msg.angular_velocity_covariance = self.angular_vel_cov

            with self.lock:
                imu_msg.orientation = self.cur_q
                if self.has_quat:
                    imu_msg.orientation_covariance = self.orientation_cov
                else:
                    # Orientation not yet available
                    imu_msg.orientation_covariance = [
                        -1.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0
                    ]

            self.imu_pub.publish(imu_msg)
            self.msg_count += 1

            # Publish Temperature
            t_msg = Temperature()
            t_msg.header = imu_msg.header
            t_msg.temperature = float(temp)
            self.temp_pub.publish(t_msg)

            # Publish Mag
            if mx != 0.0 or my != 0.0 or mz != 0.0:
                m_msg = MagneticField()
                m_msg.header = imu_msg.header
                m_msg.magnetic_field.x = float(mx * 1e-4)  # Gauss to Tesla
                m_msg.magnetic_field.y = float(-my * 1e-4 if self.tf_ned_to_enu else my * 1e-4)
                m_msg.magnetic_field.z = float(-mz * 1e-4 if self.tf_ned_to_enu else mz * 1e-4)
                self.mag_pub.publish(m_msg)

            # Periodic diagnostic logger
            t_sec = time.time()
            if t_sec - self.last_log_time > 5.0:
                self.last_log_time = t_sec
                self.get_logger().info(
                    f'IMU [{self.frame_id}]: '
                    f'Acc=({ax_enu:+.2f}, {ay_enu:+.2f}, {az_enu:+.2f}) m/s^2 | '
                    f'Gyr=({gx_enu:+.3f}, {gy_enu:+.3f}, {gz_enu:+.3f}) rad/s | '
                    f'Temp={temp:.1f}°C | Pubs={self.msg_count}'
                )

        # Msg 31: ATTITUDE_QUATERNION
        elif msg_id == 31 and len(payload) >= 32:
            (t_ms, q1, q2, q3, q4,
             rollspeed, pitchspeed, yawspeed) = struct.unpack('<Ifffffff', payload[:32])
            # q1=w, q2=x, q3=y, q4=z in MAVLink (NED frame)
            w, x, y, z = q1, q2, q3, q4

            # Convert MAVLink attitude (NED world -> aircraft body) to
            # ROS attitude (ENU world -> REP-103 body):
            # q_ros = q_NED_to_ENU * q_mav * q_body
            # where q_NED_to_ENU has Euler angles (yaw=90, pitch=0, roll=180 deg)
            # and q_body rotates MAV body (X fwd, Y right, Z down)
            # to ROS body (X fwd, Y left, Z up) (roll=180 deg).
            # This yields:
            #   w_ros = (-w - z) / sqrt(2)
            #   x_ros = (-x - y) / sqrt(2)
            #   y_ros = (-x + y) / sqrt(2)
            #   z_ros = (-w + z) / sqrt(2)
            if self.tf_ned_to_enu:
                s2 = math.sqrt(2.0)
                qw = (-w - z) / s2
                qx = (-x - y) / s2
                qy = (-x + y) / s2
                qz = (-w + z) / s2
            else:
                qw, qx, qy, qz = w, x, y, z

            # Normalize
            norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if norm > 0.0:
                qx /= norm
                qy /= norm
                qz /= norm
                qw /= norm

            # Standardize quaternion sign (qw >= 0)
            if qw < 0.0:
                qw = -qw
                qx = -qx
                qy = -qy
                qz = -qz

            with self.lock:
                self.cur_q.x = float(qx)
                self.cur_q.y = float(qy)
                self.cur_q.z = float(qz)
                self.cur_q.w = float(qw)
                self.has_quat = True

        # Msg 30: ATTITUDE (Euler angles fallback)
        elif msg_id == 30 and len(payload) >= 28:
            (t_ms, roll, pitch, yaw,
             rollspeed, pitchspeed, yawspeed) = struct.unpack('<Iffffff', payload[:28])
            if not self.has_quat:
                # Convert NED Euler to ENU
                if self.tf_ned_to_enu:
                    r_enu = roll
                    p_enu = -pitch
                    y_enu = -yaw + (math.pi / 2.0)
                else:
                    r_enu, p_enu, y_enu = roll, pitch, yaw
                q = rpy_to_quaternion(r_enu, p_enu, y_enu)
                with self.lock:
                    self.cur_q = q
                    self.has_quat = True

    def _try_parse_vn_native(self, data: bytearray) -> int:
        """Attempt to parse standard VectorNav binary frame starting with 0xFA."""
        if len(data) < 4:
            return 0
        # In case VectorNav binary stream is activated
        # Format: 0xFA, groups_present, uint16 group fields..., payload..., CRC16
        # Return 0 if not enough bytes or not valid
        return 0

    def destroy_node(self):
        """Cleanup serial port on termination."""
        self.running = False
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VectorNavDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

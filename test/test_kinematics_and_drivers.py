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

"""Unit tests for Mecanum kinematics, VectorNav IMU conversion, and Dual Laser Merger."""

import math


def test_mecanum_kinematics_roundtrip():
    """Verify forward and inverse Mecanum kinematics round-trip."""
    wheel_diameter = 0.25
    track_width = 0.538
    wheel_base = 0.430

    R = wheel_diameter / 2.0
    L = (track_width + wheel_base) / 2.0

    test_velocities = [
        (0.10, 0.0, 0.0),
        (0.0, 0.10, 0.0),
        (0.0, 0.0, 0.20),
        (0.12, -0.08, 0.15),
        (-0.10, 0.10, -0.25)
    ]

    for vx, vy, wz in test_velocities:
        # Inverse kinematics
        w1 = (vx - vy - wz * L) / R
        w2 = (vx + vy - wz * L) / R
        w3 = (vx + vy + wz * L) / R
        w4 = (vx - vy + wz * L) / R

        # Forward kinematics
        vx_fk = (R / 4.0) * (w1 + w2 + w3 + w4)
        vy_fk = (R / 4.0) * (-w1 + w2 + w3 - w4)
        wz_fk = (R / (4.0 * L)) * (-w1 - w2 + w3 + w4)

        assert abs(vx - vx_fk) < 1e-6, f'vx mismatch: expected {vx}, got {vx_fk}'
        assert abs(vy - vy_fk) < 1e-6, f'vy mismatch: expected {vy}, got {vy_fk}'
        assert abs(wz - wz_fk) < 1e-6, f'wz mismatch: expected {wz}, got {wz_fk}'


def test_vectornav_ned_to_enu_quaternion():
    """Verify NED to ENU quaternion coordinate conversion produces upright ROS orientation."""
    # Test case 1: Robot at rest, level, facing North (MAVLink roll=0, pitch=0, yaw=0)
    # In MAVLink NED: q1(w)=1, q2(x)=0, q3(y)=0, q4(z)=0
    w, x, y, z = 1.0, 0.0, 0.0, 0.0
    s2 = math.sqrt(2.0)
    qw = (-w - z) / s2
    qx = (-x - y) / s2
    qy = (-x + y) / s2
    qz = (-w + z) / s2

    # Standardize sign
    if qw < 0.0:
        qw, qx, qy, qz = -qw, -qx, -qy, -qz

    # Convert to roll, pitch, yaw
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # Robot facing North in NED corresponds to yaw = +90 deg (pi/2) in ENU
    assert abs(roll) < 1e-6, f'Expected roll ~0, got {roll}'
    assert abs(pitch) < 1e-6, f'Expected pitch ~0, got {pitch}'
    assert abs(yaw - (math.pi / 2.0)) < 1e-6, f'Expected yaw pi/2, got {yaw}'

    # Test case 2: Live IMU packet values (level on lab floor)
    # MAVLink attitude: roll=-0.026 rad, pitch=0.013 rad, yaw=1.816 rad
    w_live, x_live, y_live, z_live = -0.615, 0.013, 0.006, -0.788
    qw_l = (-w_live - z_live) / s2
    qx_l = (-x_live - y_live) / s2
    qy_l = (-x_live + y_live) / s2
    qz_l = (-w_live + z_live) / s2
    norm = math.sqrt(qw_l**2 + qx_l**2 + qy_l**2 + qz_l**2)
    qw_l, qx_l, qy_l, qz_l = qw_l / norm, qx_l / norm, qy_l / norm, qz_l / norm
    if qw_l < 0.0:
        qw_l, qx_l, qy_l, qz_l = -qw_l, -qx_l, -qy_l, -qz_l

    # In ROS ENU, roll and pitch MUST be near zero (< 5 deg), NOT 178 deg
    sinr_cosp_l = 2.0 * (qw_l * qx_l + qy_l * qz_l)
    cosr_cosp_l = 1.0 - 2.0 * (qx_l * qx_l + qy_l * qy_l)
    roll_deg = math.degrees(math.atan2(sinr_cosp_l, cosr_cosp_l))
    assert abs(roll_deg) < 5.0, f'Robot inverted! roll_deg={roll_deg}'


def test_dual_laser_merger_inverted_projection():
    """Verify 3D projection matrix correctly handles upside-down mounted LiDARs."""
    # Front laser in URDF: rpy = [0, -pi, 3*pi/4 (135 deg)]
    # In quaternion:
    roll = 0.0
    pitch = -math.pi
    yaw = 2.356194490192345

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    # Projection matrix entries
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qw * qz)
    r10 = 2.0 * (qx * qy + qw * qz)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)

    # Beam straight ahead in laser frame (lx=1, ly=0)
    gx_fwd = r00 * 1.0 + r01 * 0.0
    gy_fwd = r10 * 1.0 + r11 * 0.0

    # Beam to the left in laser frame (lx=0, ly=1)
    gx_left = r00 * 0.0 + r01 * 1.0
    gy_left = r10 * 0.0 + r11 * 1.0

    # Forward direction in robot frame should be near (cos(-45), sin(-45)) = (0.707, -0.707)
    assert abs(gx_fwd - 0.7071) < 0.01
    assert abs(gy_fwd - (-0.7071)) < 0.01

    # Left beam in laser frame (which is inverted) should point opposite to right-hand rule
    # Dot product between forward and left must be zero (orthogonal)
    dot = gx_fwd * gx_left + gy_fwd * gy_left
    assert abs(dot) < 1e-6, f'Non-orthogonal projection: dot={dot}'


def test_dual_laser_merger_bin_wrapping():
    """Verify azimuths at +/- pi wrap correctly modulo num_bins and do not drop."""
    angle_min = -math.pi
    angle_inc = math.radians(0.5)
    num_bins = int(round((math.pi - angle_min) / angle_inc))  # 720 bins

    # Test near +pi
    phi_pos_pi = math.pi - 1e-9
    bin_idx_pos = int(round((phi_pos_pi - angle_min) / angle_inc)) % num_bins
    assert 0 <= bin_idx_pos < num_bins

    # Test at exactly +pi
    phi_pi = math.pi
    bin_idx_pi = int(round((phi_pi - angle_min) / angle_inc)) % num_bins
    assert 0 <= bin_idx_pi < num_bins
    assert bin_idx_pi == 0  # +pi wraps to bin 0 (-pi)

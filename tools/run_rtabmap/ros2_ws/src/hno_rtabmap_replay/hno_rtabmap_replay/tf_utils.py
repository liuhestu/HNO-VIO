import math
import numpy as np


def normalize_quat(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n <= 0:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def quat_to_matrix(q):
    x, y, z, w = normalize_quat(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ])


def matrix_to_quat(R):
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return normalize_quat([x, y, z, w])


def pose_to_matrix(p, q):
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(q)
    T[:3, 3] = np.asarray(p, dtype=float)
    return T


def matrix_to_pose(T):
    T = np.asarray(T, dtype=float)
    return T[:3, 3].copy(), matrix_to_quat(T[:3, :3])


def invert_transform(T):
    T = np.asarray(T, dtype=float)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


def quat_to_rpy_deg(q):
    x, y, z, w = normalize_quat(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.degrees([roll, pitch, yaw])


def unwrap_degrees(values):
    values = np.asarray(values, dtype=float).copy()
    if values.size == 0:
        return values
    offset = 0.0
    prev = values[0]
    for i in range(1, len(values)):
        current = values[i] + offset
        diff = current - prev
        if diff > 180.0:
            offset -= 360.0
            current -= 360.0
        elif diff < -180.0:
            offset += 360.0
            current += 360.0
        values[i] = current
        prev = current
    return values


def make_transform_msg(parent, child, stamp, T):
    from geometry_msgs.msg import TransformStamped

    p, q = matrix_to_pose(T)
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(p[0])
    msg.transform.translation.y = float(p[1])
    msg.transform.translation.z = float(p[2])
    msg.transform.rotation.x = float(q[0])
    msg.transform.rotation.y = float(q[1])
    msg.transform.rotation.z = float(q[2])
    msg.transform.rotation.w = float(q[3])
    return msg

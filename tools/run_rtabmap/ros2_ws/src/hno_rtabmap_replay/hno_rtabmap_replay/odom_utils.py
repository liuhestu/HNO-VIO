import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .tf_utils import normalize_quat, quat_to_matrix


@dataclass
class OdomPose:
    stamp_ns: int
    p: np.ndarray
    q: np.ndarray

    @property
    def stamp_sec(self):
        return self.stamp_ns * 1e-9


def read_hno_odom_csv(path):
    poses = []
    with Path(path).open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        required = ["timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]
        if reader.fieldnames is None or any(k not in reader.fieldnames for k in required):
            raise RuntimeError(f"missing required columns in {path}: {required}")
        for row in reader:
            t = float(row["timestamp"])
            stamp_ns = int(round(t * 1e9))
            p = np.array([float(row["tx"]), float(row["ty"]), float(row["tz"])], dtype=float)
            q = normalize_quat([float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])])
            poses.append(OdomPose(stamp_ns, p, q))
    return poses


def read_tum(path):
    poses = []
    with Path(path).open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            t = float(parts[0])
            stamp_ns = int(round(t * 1e9))
            p = np.array([float(parts[1]), float(parts[2]), float(parts[3])], dtype=float)
            q = normalize_quat([float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])])
            poses.append(OdomPose(stamp_ns, p, q))
    return poses


def write_standard_csv(path, poses):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_ns", "px", "py", "pz", "qx", "qy", "qz", "qw"])
        for pose in poses:
            writer.writerow([pose.stamp_ns, *pose.p.tolist(), *pose.q.tolist()])


def read_standard_csv(path):
    poses = []
    with Path(path).open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            p = np.array([float(row["px"]), float(row["py"]), float(row["pz"])], dtype=float)
            q = normalize_quat([float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])])
            poses.append(OdomPose(int(row["timestamp_ns"]), p, q))
    return poses


def write_tum(path, poses):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for pose in poses:
            f.write(
                f"{pose.stamp_sec:.9f} {pose.p[0]:.9f} {pose.p[1]:.9f} {pose.p[2]:.9f} "
                f"{pose.q[0]:.9f} {pose.q[1]:.9f} {pose.q[2]:.9f} {pose.q[3]:.9f}\n"
            )


def validate_poses(poses, cam_start_ns=None, cam_end_ns=None, max_step_translation=0.5, max_step_rotation_deg=30.0):
    if len(poses) < 100:
        raise RuntimeError(f"row count < 100: {len(poses)}")
    stamps = [p.stamp_ns for p in poses]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise RuntimeError("timestamps are not strictly increasing")

    nan_count = 0
    q_bad = 0
    max_step = 0.0
    max_step_rot = 0.0
    max_step_time = 0.0
    large_step_count = 0
    for pose in poses:
        vals = list(pose.p) + list(pose.q)
        if any(not math.isfinite(v) for v in vals):
            nan_count += 1
        if abs(np.linalg.norm(pose.q) - 1.0) > 0.02:
            q_bad += 1
    for a, b in zip(poses, poses[1:]):
        dt = (b.stamp_ns - a.stamp_ns) * 1e-9
        step = float(np.linalg.norm(b.p - a.p))
        dq = quat_to_matrix(a.q).T @ quat_to_matrix(b.q)
        rot = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dq) - 1.0) / 2.0))))
        if step > max_step:
            max_step = step
            max_step_time = dt
        max_step_rot = max(max_step_rot, rot)
        if step > max_step_translation or rot > max_step_rotation_deg:
            large_step_count += 1
    if nan_count:
        raise RuntimeError(f"NaN/Inf detected: {nan_count}")
    if q_bad > max(1, int(0.01 * len(poses))):
        raise RuntimeError(f"too many quaternion norm failures: {q_bad}")
    if cam_start_ns is not None and cam_end_ns is not None:
        if poses[-1].stamp_ns < cam_start_ns or poses[0].stamp_ns > cam_end_ns:
            raise RuntimeError("odom and cam0 timestamp ranges do not overlap")
    if large_step_count:
        raise RuntimeError(f"large odom step count > 0: {large_step_count}")
    duration = (poses[-1].stamp_ns - poses[0].stamp_ns) * 1e-9
    return {
        "row_count": len(poses),
        "start_timestamp_ns": poses[0].stamp_ns,
        "end_timestamp_ns": poses[-1].stamp_ns,
        "duration_sec": duration,
        "mean_hz": (len(poses) - 1) / duration if duration > 0 else 0.0,
        "quaternion_norm_bad_count": q_bad,
        "nan_count": nan_count,
        "max_step_translation_m": max_step,
        "max_step_rotation_deg": max_step_rot,
        "max_step_time_sec": max_step_time,
        "large_step_count": large_step_count,
    }


def nearest_pose(poses, stamp_ns, max_diff_ns):
    if not poses:
        return None, None
    lo, hi = 0, len(poses)
    while lo < hi:
        mid = (lo + hi) // 2
        if poses[mid].stamp_ns < stamp_ns:
            lo = mid + 1
        else:
            hi = mid
    candidates = []
    if lo < len(poses):
        candidates.append(poses[lo])
    if lo > 0:
        candidates.append(poses[lo - 1])
    best = min(candidates, key=lambda p: abs(p.stamp_ns - stamp_ns))
    diff = abs(best.stamp_ns - stamp_ns)
    if diff > max_diff_ns:
        return None, diff
    return best, diff

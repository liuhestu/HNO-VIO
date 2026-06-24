import csv
from pathlib import Path

import numpy as np
import yaml


def read_euroc_image_csv(path):
    rows = []
    with Path(path).open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            ts, filename = line.split(",", 1)
            rows.append((int(ts), filename))
    return rows


def pair_stereo(cam0_csv, cam1_csv):
    left = read_euroc_image_csv(cam0_csv)
    right = dict(read_euroc_image_csv(cam1_csv))
    return [(ts, fn, right[ts]) for ts, fn in left if ts in right]


def read_sensor_yaml(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    T = np.array(data["T_BS"]["data"], dtype=float).reshape(4, 4)
    fx, fy, cx, cy = [float(v) for v in data["intrinsics"]]
    dist = [float(v) for v in data["distortion_coefficients"]]
    width, height = [int(v) for v in data["resolution"]]
    return {
        "T_BS": T,
        "K": np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=float),
        "D": np.array(dist, dtype=float),
        "width": width,
        "height": height,
    }


def read_gt_csv(path):
    poses = []
    with Path(path).open(encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            stamp_ns = int(row[0])
            p = np.array([float(row[1]), float(row[2]), float(row[3])], dtype=float)
            qw, qx, qy, qz = [float(row[i]) for i in range(4, 8)]
            poses.append((stamp_ns, p, np.array([qx, qy, qz, qw], dtype=float)))
    return poses

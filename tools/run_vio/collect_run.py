#!/usr/bin/env python3
import argparse
import csv
import json
import math
import signal
import sys
import threading
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud


class Collector:
    def __init__(self, output_dir):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / "odom.csv"
        self.tum_path = self.output_dir / "odom.tum"
        self.status_path = self.output_dir / "collector_status.json"
        self.lock = threading.Lock()
        self.first_stamp = None
        self.last_stamp = None
        self.last_position = None
        self.path_length = 0.0
        self.pose_count = 0
        self.feature_counts = []
        self.tail_feature_counts = []

        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.tum_file = self.tum_path.open("w", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"])

    def pose_cb(self, msg):
        stamp = msg.header.stamp.to_sec()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        row = [stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        with self.lock:
            if self.first_stamp is None:
                self.first_stamp = stamp
            if self.last_position is not None:
                dx = p.x - self.last_position[0]
                dy = p.y - self.last_position[1]
                dz = p.z - self.last_position[2]
                self.path_length += math.sqrt(dx * dx + dy * dy + dz * dz)
            self.last_position = (p.x, p.y, p.z)
            self.last_stamp = stamp
            self.pose_count += 1
            self.csv_writer.writerow(row)
            self.tum_file.write(f"{stamp:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")
            if self.pose_count % 20 == 0:
                self.csv_file.flush()
                self.tum_file.flush()

    def features_cb(self, msg):
        count = len(msg.points)
        with self.lock:
            self.feature_counts.append(count)
            self.tail_feature_counts.append(count)
            if len(self.tail_feature_counts) > 20:
                self.tail_feature_counts = self.tail_feature_counts[-20:]

    def close(self):
        with self.lock:
            self.csv_file.flush()
            self.tum_file.flush()
            self.csv_file.close()
            self.tum_file.close()
            duration = 0.0
            if self.first_stamp is not None and self.last_stamp is not None:
                duration = max(0.0, self.last_stamp - self.first_stamp)
            mean_features = 0.0
            if self.feature_counts:
                mean_features = sum(self.feature_counts) / float(len(self.feature_counts))
            active_tail = 0
            if self.tail_feature_counts:
                active_tail = int(round(sum(self.tail_feature_counts) / float(len(self.tail_feature_counts))))
            status = {
                "first_timestamp": self.first_stamp,
                "last_timestamp": self.last_stamp,
                "duration_sec": duration,
                "pose_count": self.pose_count,
                "path_length_m": self.path_length,
                "mean_num_features": mean_features,
                "active_landmarks_tail": active_tail,
            }
            self.status_path.write_text(json.dumps(status, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--pose-topic", default="/run_hno_vio/pose")
    parser.add_argument("--features-topic", default="/run_hno_vio/features_3d")
    args = parser.parse_args()

    collector = Collector(args.output_dir)

    def shutdown(signum=None, frame=None):
        rospy.signal_shutdown("collector stopping")

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    rospy.init_node("hno_vio_collect_run", anonymous=True, disable_signals=True)
    rospy.Subscriber(args.pose_topic, PoseWithCovarianceStamped, collector.pose_cb, queue_size=1000)
    rospy.Subscriber(args.features_topic, PointCloud, collector.features_cb, queue_size=100)

    try:
        rospy.spin()
    finally:
        collector.close()


if __name__ == "__main__":
    sys.exit(main())

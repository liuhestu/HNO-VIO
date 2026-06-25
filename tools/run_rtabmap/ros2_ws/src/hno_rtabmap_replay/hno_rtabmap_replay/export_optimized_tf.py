"""Export optimized RTAB-Map graph odometry from a recorded ROS 2 bag.

Usage:
    hno_export_optimized_tf --bag RTABMAP_OUTPUT_BAG --out-dir OUT_DIR

Inputs:
    --bag: ROS 2 bag recorded from the RTAB-Map run.
    --out-dir: Offline results directory.
    Optional: --tf-match-time, --max-tf-diff-sec, --min-graph-poses,
    --warn-graph-poses.

Outputs:
    OUT_DIR/odom_optimized.txt
    OUT_DIR/logs/export_report.txt
"""

import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rtabmap_msgs.msg import MapData
from tf2_msgs.msg import TFMessage

from .odom_utils import OdomPose, write_tum
from .tf_utils import invert_transform, matrix_to_pose, pose_to_matrix


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def transform_to_matrix(transform):
    p = np.array([
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ], dtype=float)
    q = np.array([
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    ], dtype=float)
    return pose_to_matrix(p, q)


def nearest_by_stamp(items, stamp_ns, max_diff_ns=200_000_000):
    if not items:
        return None
    lo, hi = 0, len(items)
    while lo < hi:
        mid = (lo + hi) // 2
        if items[mid][0] < stamp_ns:
            lo = mid + 1
        else:
            hi = mid
    candidates = []
    if lo < len(items):
        candidates.append(items[lo])
    if lo > 0:
        candidates.append(items[lo - 1])
    best = min(candidates, key=lambda item: abs(item[0] - stamp_ns))
    if abs(best[0] - stamp_ns) > max_diff_ns:
        return None
    return best[1]


def open_reader(bag_dir):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)
    return reader


def read_bag(bag_dir):
    reader = open_reader(bag_dir)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    raw = []
    map_odom_header = []
    map_odom_bag = []
    map_base_header = []
    map_base_bag = []
    node_stamps = {}
    final_map_data = None
    topic_counts = {}

    while reader.has_next():
        topic, data, bag_stamp = reader.read_next()
        topic_counts[topic] = topic_counts.get(topic, 0) + 1
        msg_type_name = topic_types.get(topic)
        if msg_type_name is None:
            continue
        msg_type = get_message(msg_type_name)
        msg = deserialize_message(data, msg_type)

        if topic == "/hno_vio/odom" and isinstance(msg, Odometry):
            stamp_ns = stamp_to_ns(msg.header.stamp)
            p = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ], dtype=float)
            q = np.array([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ], dtype=float)
            raw.append((stamp_ns, int(bag_stamp), pose_to_matrix(p, q)))
        elif topic in ("/tf", "/tf_static") and isinstance(msg, TFMessage):
            for transform in msg.transforms:
                stamp_ns = stamp_to_ns(transform.header.stamp)
                parent = transform.header.frame_id.strip("/")
                child = transform.child_frame_id.strip("/")
                T = transform_to_matrix(transform.transform)
                if parent == "map" and child == "odom":
                    map_odom_header.append((stamp_ns, T))
                    map_odom_bag.append((int(bag_stamp), T))
                elif parent == "odom" and child == "map":
                    T_inv = invert_transform(T)
                    map_odom_header.append((stamp_ns, T_inv))
                    map_odom_bag.append((int(bag_stamp), T_inv))
                elif parent == "map" and child == "base_link":
                    map_base_header.append((stamp_ns, T))
                    map_base_bag.append((int(bag_stamp), T))
                elif parent == "base_link" and child == "map":
                    T_inv = invert_transform(T)
                    map_base_header.append((stamp_ns, T_inv))
                    map_base_bag.append((int(bag_stamp), T_inv))
        elif topic == "/rtabmap/mapData" and isinstance(msg, MapData):
            final_map_data = msg
            for node in msg.nodes:
                if float(node.stamp) > 0:
                    node_stamps[int(node.id)] = int(round(float(node.stamp) * 1e9))

    raw.sort(key=lambda item: item[0])
    map_odom_header.sort(key=lambda item: item[0])
    map_odom_bag.sort(key=lambda item: item[0])
    map_base_header.sort(key=lambda item: item[0])
    map_base_bag.sort(key=lambda item: item[0])
    graph_final = []
    missing_graph_stamps = 0
    if final_map_data is not None:
        for node_id, pose in zip(final_map_data.graph.poses_id, final_map_data.graph.poses):
            stamp_ns = node_stamps.get(int(node_id))
            if stamp_ns is None:
                missing_graph_stamps += 1
                continue
            p = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
            q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], dtype=float)
            graph_final.append((stamp_ns, pose_to_matrix(p, q)))
    graph_final.sort(key=lambda item: item[0])
    return raw, map_odom_header, map_odom_bag, map_base_header, map_base_bag, graph_final, missing_graph_stamps, topic_counts


def matrices_to_poses(items):
    poses = []
    for stamp_ns, T in items:
        p, q = matrix_to_pose(T)
        poses.append(OdomPose(stamp_ns, p, q))
    return poses


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--tf-match-time", choices=["bag", "header"], default="bag")
    parser.add_argument("--max-tf-diff-sec", type=float, default=0.2)
    parser.add_argument("--min-graph-poses", type=int, default=20)
    parser.add_argument("--warn-graph-poses", type=int, default=50)
    args = parser.parse_args()

    bag_dir = Path(args.bag)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    log_dir = out_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    if not bag_dir.exists():
        raise RuntimeError(f"output bag not found: {bag_dir}")

    raw, map_odom_header, map_odom_bag, map_base_header, map_base_bag, graph_final, missing_graph_stamps, topic_counts = read_bag(bag_dir)
    if not raw:
        raise RuntimeError("no /hno_vio/odom messages found in output bag")

    max_tf_diff_ns = int(args.max_tf_diff_sec * 1e9)
    map_odom_optimized_count = 0
    map_odom = map_odom_bag if args.tf_match_time == "bag" else map_odom_header
    map_base = map_base_bag if args.tf_match_time == "bag" else map_base_header
    if map_odom:
        for stamp_ns, bag_stamp_ns, T_odom_base in raw:
            match_stamp = bag_stamp_ns if args.tf_match_time == "bag" else stamp_ns
            T_map_odom = nearest_by_stamp(map_odom, match_stamp, max_tf_diff_ns)
            if T_map_odom is None:
                continue
            _T_map_base = T_map_odom @ T_odom_base
            map_odom_optimized_count += 1
        diagnostic_source = f"map->odom composed with odom->base_link using {args.tf_match_time} stamps"
    elif map_base:
        for stamp_ns, bag_stamp_ns, _T_odom_base in raw:
            match_stamp = bag_stamp_ns if args.tf_match_time == "bag" else stamp_ns
            T_map_base = nearest_by_stamp(map_base, match_stamp, max_tf_diff_ns)
            if T_map_base is None:
                continue
            _T_map_base = T_map_base
            map_odom_optimized_count += 1
        diagnostic_source = f"direct map->base_link using {args.tf_match_time} stamps"
    else:
        diagnostic_source = "no RTAB-Map map->odom or map->base_link TF found in output bag"

    if len(graph_final) < args.min_graph_poses:
        raise RuntimeError(f"too few graph poses exported from mapData.graph.poses: {len(graph_final)} < {args.min_graph_poses}")

    optimized_tum = out_dir / "odom_optimized.txt"
    write_tum(optimized_tum, matrices_to_poses(graph_final))

    warning = ""
    if len(graph_final) < args.warn_graph_poses:
        warning = f"WARNING: graph pose count {len(graph_final)} < {args.warn_graph_poses}"

    report = log_dir / "export_report.txt"
    report.write_text(
        "\n".join([
            f"raw_pose_count: {len(raw)}",
            f"map_odom_diagnostic_pose_count: {map_odom_optimized_count}",
            f"tf_match_time: {args.tf_match_time}",
            f"map_odom_header_tf_count: {len(map_odom_header)}",
            f"map_odom_bag_tf_count: {len(map_odom_bag)}",
            f"map_base_header_tf_count: {len(map_base_header)}",
            f"map_base_bag_tf_count: {len(map_base_bag)}",
            f"graph_final_pose_count: {len(graph_final)}",
            f"graph_final_missing_stamp_count: {missing_graph_stamps}",
            f"optimized_source: mapData.graph.poses",
            f"diagnostic_tf_source: {diagnostic_source}",
            f"warning: {warning or 'none'}",
            "topic_counts:",
            *[f"  {name}: {count}" for name, count in sorted(topic_counts.items())],
        ]) + "\n",
        encoding="utf-8",
    )
    if warning:
        print(warning)
    print(f"optimized_tum: {optimized_tum}")
    print(f"report: {report}")


if __name__ == "__main__":
    main()

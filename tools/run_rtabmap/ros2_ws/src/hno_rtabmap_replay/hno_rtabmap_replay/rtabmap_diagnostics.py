"""Write diagnostics for the RTAB-Map offline input and output bags.

Usage:
    hno_analyze_rtabmap_bag --input-bag INPUT_BAG \
        --output-bag RTABMAP_OUTPUT_BAG --out-dir LOG_DIR

Inputs:
    --input-bag: ROS 2 bag generated for RTAB-Map input replay.
    --output-bag: ROS 2 bag recorded from RTAB-Map output topics.
    --out-dir: Directory for diagnostic text reports.

Outputs:
    OUT_DIR/map_odom_stats.txt
    OUT_DIR/rtabmap_graph_stats.txt
    OUT_DIR/topic_hz_stats.txt
"""

import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rtabmap_msgs.msg import Info, MapData
from tf2_msgs.msg import TFMessage

from .tf_utils import pose_to_matrix


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def transform_to_matrix(transform):
    p = np.array([transform.translation.x, transform.translation.y, transform.translation.z], dtype=float)
    q = np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w], dtype=float)
    return pose_to_matrix(p, q)


def rotation_angle_deg(T):
    R = T[:3, :3]
    return float(np.degrees(np.arccos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))))


def open_reader(bag_dir):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)
    return reader


def topic_hz(stamps):
    if len(stamps) < 2:
        return None
    stamps = sorted(stamps)
    duration = (stamps[-1] - stamps[0]) * 1e-9
    if duration <= 0:
        return None
    return (len(stamps) - 1) / duration


def read_bag_topics(bag_dir, wanted_topics=None):
    reader = open_reader(bag_dir)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    counts = {}
    stamps = {}
    messages = []
    while reader.has_next():
        topic, data, bag_stamp = reader.read_next()
        if wanted_topics is not None and topic not in wanted_topics:
            continue
        counts[topic] = counts.get(topic, 0) + 1
        msg_type_name = topic_types.get(topic)
        if msg_type_name is None:
            stamps.setdefault(topic, []).append(int(bag_stamp))
            continue
        msg_type = get_message(msg_type_name)
        msg = deserialize_message(data, msg_type)
        stamp_ns = int(bag_stamp)
        if hasattr(msg, "header"):
            stamp_ns = stamp_to_ns(msg.header.stamp)
        elif topic == "/tf" and isinstance(msg, TFMessage) and msg.transforms:
            stamp_ns = stamp_to_ns(msg.transforms[0].header.stamp)
        stamps.setdefault(topic, []).append(stamp_ns)
        messages.append((topic, msg, stamp_ns))
    return counts, stamps, messages


def transform_stats_lines(prefix, transforms):
    lines = [f"{prefix}_count: {len(transforms)}"]
    if not transforms:
        return lines + [f"{prefix}_effectively_identity: unknown_no_transforms"]
    trans_norm = np.array([np.linalg.norm(T[:3, 3]) for _stamp, T in transforms])
    rot_deg = np.array([rotation_angle_deg(T) for _stamp, T in transforms])
    first_T = transforms[0][1]
    delta_trans = np.array([np.linalg.norm(T[:3, 3] - first_T[:3, 3]) for _stamp, T in transforms])
    delta_rot = np.array([rotation_angle_deg(np.linalg.inv(first_T) @ T) for _stamp, T in transforms])
    lines.extend([
        f"{prefix}_first_stamp_ns: {transforms[0][0]}",
        f"{prefix}_last_stamp_ns: {transforms[-1][0]}",
        f"{prefix}_duration_sec: {(transforms[-1][0] - transforms[0][0]) * 1e-9:.9f}",
        f"{prefix}_translation_norm_mean_m: {float(np.mean(trans_norm)):.12f}",
        f"{prefix}_translation_norm_max_m: {float(np.max(trans_norm)):.12f}",
        f"{prefix}_rotation_from_identity_mean_deg: {float(np.mean(rot_deg)):.12f}",
        f"{prefix}_rotation_from_identity_max_deg: {float(np.max(rot_deg)):.12f}",
        f"{prefix}_delta_from_first_translation_mean_m: {float(np.mean(delta_trans)):.12f}",
        f"{prefix}_delta_from_first_translation_max_m: {float(np.max(delta_trans)):.12f}",
        f"{prefix}_delta_from_first_rotation_mean_deg: {float(np.mean(delta_rot)):.12f}",
        f"{prefix}_delta_from_first_rotation_max_deg: {float(np.max(delta_rot)):.12f}",
        f"{prefix}_effectively_identity: {bool(np.max(trans_norm) < 1e-5 and np.max(rot_deg) < 1e-4)}",
    ])
    return lines


def write_map_odom_stats(output_bag, out_dir):
    counts, _stamps, messages = read_bag_topics(output_bag, {"/tf", "/tf_static", "/rtabmap/mapData"})
    transforms = []
    mapdata_transforms = []
    for topic, msg, _stamp_ns in messages:
        if isinstance(msg, TFMessage):
            for tf in msg.transforms:
                parent = tf.header.frame_id.strip("/")
                child = tf.child_frame_id.strip("/")
                if parent == "map" and child == "odom":
                    transforms.append((stamp_to_ns(tf.header.stamp), transform_to_matrix(tf.transform)))
        elif isinstance(msg, MapData):
            mapdata_transforms.append((stamp_to_ns(msg.header.stamp), transform_to_matrix(msg.graph.map_to_odom)))
    transforms.sort(key=lambda item: item[0])
    mapdata_transforms.sort(key=lambda item: item[0])
    euroc_transforms = [(stamp, T) for stamp, T in transforms if stamp > 1_000_000_000_000_000]

    lines = [
        f"source_bag: {output_bag}",
        f"tf_topic_count: {counts.get('/tf', 0)}",
        f"mapData_topic_count: {counts.get('/rtabmap/mapData', 0)}",
        *transform_stats_lines("tf_map_odom_all", transforms),
        *transform_stats_lines("tf_map_odom_euroc_time", euroc_transforms),
        *transform_stats_lines("mapData_graph_map_to_odom", mapdata_transforms),
    ]
    (out_dir / "map_odom_stats.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def link_stats(links):
    adjacent = 0
    non_adjacent = 0
    type_counts = {}
    for link in links:
        if abs(int(link.from_id) - int(link.to_id)) <= 1:
            adjacent += 1
        else:
            non_adjacent += 1
        type_counts[int(link.type)] = type_counts.get(int(link.type), 0) + 1
    return adjacent, non_adjacent, type_counts


def write_graph_stats(output_bag, out_dir):
    wanted = {"/rtabmap/info", "/rtabmap/mapData", "/rtabmap/global_path", "/rtabmap/local_path"}
    counts, _stamps, messages = read_bag_topics(output_bag, wanted)
    info_count = 0
    loop_nonzero = 0
    proximity_nonzero = 0
    loop_ids = []
    proximity_ids = []
    mapdata_count = 0
    final_pose_count = 0
    final_link_count = 0
    final_adjacent = 0
    final_non_adjacent = 0
    max_non_adjacent = 0
    final_type_counts = {}
    max_link_count = 0
    global_path_pose_counts = []
    local_path_pose_counts = []

    for topic, msg, _stamp_ns in messages:
        if topic == "/rtabmap/info" and isinstance(msg, Info):
            info_count += 1
            if int(msg.loop_closure_id) != 0:
                loop_nonzero += 1
                loop_ids.append(int(msg.loop_closure_id))
            if int(msg.proximity_detection_id) != 0:
                proximity_nonzero += 1
                proximity_ids.append(int(msg.proximity_detection_id))
        elif topic == "/rtabmap/mapData" and isinstance(msg, MapData):
            mapdata_count += 1
            links = list(msg.graph.links)
            adjacent, non_adjacent, type_counts = link_stats(links)
            max_link_count = max(max_link_count, len(links))
            max_non_adjacent = max(max_non_adjacent, non_adjacent)
            final_pose_count = len(msg.graph.poses_id)
            final_link_count = len(links)
            final_adjacent = adjacent
            final_non_adjacent = non_adjacent
            final_type_counts = type_counts
        elif topic == "/rtabmap/global_path" and isinstance(msg, PathMsg):
            global_path_pose_counts.append(len(msg.poses))
        elif topic == "/rtabmap/local_path" and isinstance(msg, PathMsg):
            local_path_pose_counts.append(len(msg.poses))

    lines = [
        f"source_bag: {output_bag}",
        f"info_count: {info_count}",
        f"mapData_count: {mapdata_count}",
        f"global_path_msg_count: {counts.get('/rtabmap/global_path', 0)}",
        f"global_path_last_pose_count: {global_path_pose_counts[-1] if global_path_pose_counts else 0}",
        f"local_path_msg_count: {counts.get('/rtabmap/local_path', 0)}",
        f"local_path_last_pose_count: {local_path_pose_counts[-1] if local_path_pose_counts else 0}",
        f"loop_closure_id_nonzero_count: {loop_nonzero}",
        f"proximity_detection_id_nonzero_count: {proximity_nonzero}",
        f"loop_closure_ids_sample: {loop_ids[:20]}",
        f"proximity_detection_ids_sample: {proximity_ids[:20]}",
        f"final_graph_pose_count: {final_pose_count}",
        f"final_graph_link_count: {final_link_count}",
        f"final_adjacent_link_count: {final_adjacent}",
        f"final_non_adjacent_link_count: {final_non_adjacent}",
        f"max_graph_link_count: {max_link_count}",
        f"max_non_adjacent_link_count: {max_non_adjacent}",
        "final_link_type_counts:",
        *[f"  type_{key}: {value}" for key, value in sorted(final_type_counts.items())],
    ]
    (out_dir / "rtabmap_graph_stats.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_topic_hz_stats(input_bag, output_bag, out_dir):
    input_topics = {"/cam0/image_rect", "/cam1/image_rect", "/cam0/camera_info", "/cam1/camera_info", "/hno_vio/odom", "/tf"}
    output_topics = {"/rtabmap/rgbd_image", "/rtabmap/info", "/rtabmap/mapData", "/rtabmap/global_path", "/rtabmap/local_path"}
    input_counts, input_stamps, _ = read_bag_topics(input_bag, input_topics)
    output_counts, output_stamps, _ = read_bag_topics(output_bag, output_topics)
    lines = [f"input_bag: {input_bag}", f"output_bag: {output_bag}", "topics:"]
    for name in sorted(input_topics):
        hz = topic_hz(input_stamps.get(name, []))
        lines.append(f"  {name}: count={input_counts.get(name, 0)} hz={hz if hz is not None else 'n/a'}")
    for name in sorted(output_topics):
        hz = topic_hz(output_stamps.get(name, []))
        recorded = name in output_counts
        lines.append(f"  {name}: count={output_counts.get(name, 0)} hz={hz if hz is not None else 'n/a'} recorded={recorded}")
    (out_dir / "topic_hz_stats.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-bag", required=True)
    parser.add_argument("--output-bag", required=True)
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    write_map_odom_stats(Path(args.output_bag), out_dir)
    write_graph_stats(Path(args.output_bag), out_dir)
    write_topic_hz_stats(Path(args.input_bag), Path(args.output_bag), out_dir)
    print(out_dir / "map_odom_stats.txt")
    print(out_dir / "rtabmap_graph_stats.txt")
    print(out_dir / "topic_hz_stats.txt")


if __name__ == "__main__":
    main()

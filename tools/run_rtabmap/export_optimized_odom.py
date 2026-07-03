#!/usr/bin/python3
"""Export RTAB-Map optimized graph poses from /rtabmap/mapData.graph.poses."""

import argparse
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rtabmap_msgs.msg import MapData


def open_reader(bag_dir):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def read_graph_poses(output_bag):
    reader = open_reader(output_bag)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    final_map_data = None
    mapdata_count = 0

    while reader.has_next():
        topic, data, _bag_stamp = reader.read_next()
        if topic != "/rtabmap/mapData":
            continue
        msg_type_name = topic_types.get(topic)
        if msg_type_name is None:
            continue
        msg = deserialize_message(data, get_message(msg_type_name))
        if isinstance(msg, MapData):
            final_map_data = msg
            mapdata_count += 1

    if final_map_data is None:
        raise RuntimeError("no /rtabmap/mapData messages found")

    node_stamps = {}
    for node in final_map_data.nodes:
        stamp = float(getattr(node, "stamp", 0.0))
        if stamp > 0.0:
            node_stamps[int(node.id)] = stamp

    poses = []
    missing_stamps = 0
    for node_id, pose in zip(final_map_data.graph.poses_id, final_map_data.graph.poses):
        stamp = node_stamps.get(int(node_id))
        if stamp is None:
            missing_stamps += 1
            continue
        poses.append((
            stamp,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ))

    poses.sort(key=lambda row: row[0])
    return poses, mapdata_count, missing_stamps


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="RTAB-Map output rosbag2 directory")
    parser.add_argument("--out", required=True, help="Output TUM trajectory path")
    parser.add_argument("--min-poses", type=int, default=20)
    args = parser.parse_args()

    output_bag = Path(args.bag)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    log_dir = out_path.parent / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    poses, mapdata_count, missing_stamps = read_graph_poses(output_bag)
    if len(poses) < args.min_poses:
        raise RuntimeError(f"too few graph poses from /rtabmap/mapData.graph.poses: {len(poses)} < {args.min_poses}")

    with out_path.open("w", encoding="utf-8") as f:
        for row in poses:
            f.write("{:.9f} {:.12f} {:.12f} {:.12f} {:.12f} {:.12f} {:.12f} {:.12f}\n".format(*row))

    report = log_dir / "export_report.txt"
    report.write_text(
        "\n".join([
            f"source_bag: {output_bag}",
            "source_topic: /rtabmap/mapData",
            "source_field: graph.poses",
            f"mapdata_count: {mapdata_count}",
            f"optimized_pose_count: {len(poses)}",
            f"missing_graph_pose_stamps: {missing_stamps}",
            f"output: {out_path}",
        ]) + "\n",
        encoding="utf-8",
    )
    print(out_path)


if __name__ == "__main__":
    main()

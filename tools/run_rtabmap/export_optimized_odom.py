#!/usr/bin/python3
"""Export optimized RTAB-Map graph poses to a TUM trajectory."""

import argparse
import time
from pathlib import Path

import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rtabmap_msgs.msg import MapData
from rtabmap_msgs.srv import GetMap2


def open_reader(bag_dir):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def extract_graph_poses(map_data):
    node_stamps = {}
    for node in map_data.nodes:
        stamp = float(getattr(node, "stamp", 0.0))
        if stamp > 0.0:
            node_stamps[int(node.id)] = stamp

    poses = []
    missing_stamps = 0
    for node_id, pose in zip(map_data.graph.poses_id, map_data.graph.poses):
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
    return poses, missing_stamps


def read_graph_poses_from_bag(output_bag):
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

    poses, missing_stamps = extract_graph_poses(final_map_data)
    return poses, mapdata_count, missing_stamps


def publish_map_data(node, map_data, topic_name, timeout_sec):
    publisher = node.create_publisher(MapData, topic_name, 1)
    deadline = time.monotonic() + timeout_sec
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        raise RuntimeError(f"no subscriber discovered for {topic_name}")

    publisher.publish(map_data)
    publish_deadline = time.monotonic() + 1.0
    while time.monotonic() < publish_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)


def read_graph_poses_from_service(service_name, timeout_sec, publish_topic):
    rclpy.init()
    node = Node("export_optimized_odom")
    try:
        client = node.create_client(GetMap2, service_name)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"service not available: {service_name}")

        request = GetMap2.Request()
        request.global_map = True
        request.optimized = True
        request.with_images = False
        request.with_scans = False
        request.with_user_data = False
        request.with_grids = False
        request.with_words = False
        request.with_global_descriptors = False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done():
            raise TimeoutError(f"timed out waiting for {service_name}")
        if future.exception() is not None:
            raise RuntimeError(f"{service_name} failed: {future.exception()}")

        map_data = future.result().data
        if publish_topic:
            publish_map_data(node, map_data, publish_topic, timeout_sec)
        poses, missing_stamps = extract_graph_poses(map_data)
        return poses, 1, missing_stamps
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bag", help="RTAB-Map output rosbag2 directory")
    source.add_argument("--service", help="RTAB-Map GetMap2 service name")
    parser.add_argument("--publish-topic", help="Publish the service MapData once on this topic")
    parser.add_argument("--out", required=True, help="Output TUM trajectory path")
    parser.add_argument("--min-poses", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    log_dir = out_path.parent / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    if args.bag:
        source_name = str(Path(args.bag))
        source_type = "bag"
        poses, mapdata_count, missing_stamps = read_graph_poses_from_bag(Path(args.bag))
    else:
        source_name = args.service
        source_type = "GetMap2 service"
        poses, mapdata_count, missing_stamps = read_graph_poses_from_service(
            args.service, args.timeout, args.publish_topic
        )
    if len(poses) < args.min_poses:
        raise RuntimeError(f"too few graph poses from /rtabmap/mapData.graph.poses: {len(poses)} < {args.min_poses}")

    with out_path.open("w", encoding="utf-8") as f:
        for row in poses:
            f.write("{:.9f} {:.12f} {:.12f} {:.12f} {:.12f} {:.12f} {:.12f} {:.12f}\n".format(*row))

    report = log_dir / "export_report.txt"
    report.write_text(
        "\n".join([
            f"source_type: {source_type}",
            f"source: {source_name}",
            "source_field: graph.poses",
            f"published_topic: {args.publish_topic or 'none'}",
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

#!/usr/bin/env python3
"""Convert one HNO-VIO RTAB-Map run into a self-contained Rerun recording."""

import argparse
import copy
import io
import json
import logging
import os
import shlex
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from evo.core import lie_algebra, sync
from evo.tools import file_interface
from PIL import Image
from plyfile import PlyData
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


LOGGER = logging.getLogger("hno_visual_rerun")
TRAJECTORY_COLORS = {
    "ground_truth": [80, 220, 100],
    "raw": [255, 120, 50],
    "optimized": [40, 180, 255],
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Export an HNO-VIO RTAB-Map run to a Rerun .rrd file."
    )
    parser.add_argument("run_dir", help="Path to results/run_YYYYmmddTHHMMSS")
    parser.add_argument("--output", help="Output .rrd path")
    parser.add_argument(
        "--voxel",
        type=float,
        default=0.03,
        help="RTAB-Map cloud voxel size in meters (default: 0.03)",
    )
    parser.add_argument(
        "--point-radius",
        type=float,
        default=0.006,
        help="Rerun point radius in meters (default: 0.006)",
    )
    parser.add_argument(
        "--image-rate",
        type=float,
        default=10.0,
        help="Maximum logged rate per rectified camera in Hz (default: 10)",
    )
    parser.add_argument(
        "--map-rate",
        type=float,
        default=2.0,
        help="Progressive point-cloud update rate in Hz (default: 2)",
    )
    parser.add_argument(
        "--trajectory-rate",
        type=float,
        default=5.0,
        help="Progressive trajectory update rate in Hz (default: 5)",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=75,
        help="JPEG quality for camera panels (default: 75)",
    )
    parser.add_argument(
        "--no-images",
        action="store_true",
        help="Do not include rectified camera images from the RTAB-Map input bag",
    )
    parser.add_argument(
        "--no-open",
        action="store_true",
        help="Generate the .rrd without opening the Rerun Viewer",
    )
    return parser.parse_args()


def configure_logging(log_path):
    LOGGER.setLevel(logging.INFO)
    LOGGER.handlers.clear()
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    for handler in (logging.StreamHandler(), logging.FileHandler(log_path, mode="w")):
        handler.setFormatter(formatter)
        LOGGER.addHandler(handler)


def require_file(path, label):
    if not path.is_file() or path.stat().st_size == 0:
        raise FileNotFoundError(f"missing or empty {label}: {path}")


def require_command(name):
    command = shutil.which(name)
    if command is None:
        raise RuntimeError(f"required command not found: {name}")
    return command


def resolve_run_file(run_dir, value, fallback):
    if value:
        candidate = Path(value).expanduser()
        if not candidate.is_absolute():
            candidate = run_dir / candidate
        if candidate.is_file():
            return candidate.resolve()
    return fallback.resolve()


def export_rtabmap(database, output_dir, voxel):
    exporter = require_command("rtabmap-export")
    command = [
        exporter,
        "--cloud",
        "--poses",
        "--poses_format",
        "10",
        "--opt",
        "2",
        "--voxel",
        str(voxel),
        "--output",
        "hno_rtabmap",
        "--output_dir",
        str(output_dir),
        str(database),
    ]
    LOGGER.info("running: %s", shlex.join(command))
    completed = subprocess.run(command, text=True, capture_output=True, check=False)
    if completed.stdout:
        LOGGER.info("rtabmap-export stdout:\n%s", completed.stdout.rstrip())
    if completed.stderr:
        LOGGER.info("rtabmap-export stderr:\n%s", completed.stderr.rstrip())
    if completed.returncode != 0:
        raise subprocess.CalledProcessError(completed.returncode, command)

    point_clouds = sorted(output_dir.rglob("*.ply"))
    pose_files = sorted(path for path in output_dir.rglob("*.txt") if "pose" in path.name)
    if len(point_clouds) != 1:
        raise RuntimeError(
            f"expected one exported PLY point cloud, found {len(point_clouds)} in {output_dir}"
        )
    if len(pose_files) != 1:
        raise RuntimeError(
            f"expected one exported pose file, found {len(pose_files)} in {output_dir}"
        )
    return point_clouds[0], pose_files[0]


def packed_colors(values):
    packed = np.asarray(values)
    if packed.dtype.kind == "f":
        packed = packed.astype(np.float32, copy=False).view(np.uint32)
    else:
        packed = packed.astype(np.uint32, copy=False)
    return np.column_stack(
        ((packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF)
    ).astype(np.uint8)


def load_point_cloud(path):
    vertex = PlyData.read(str(path))["vertex"].data
    names = set(vertex.dtype.names or ())
    required = {"x", "y", "z"}
    if not required.issubset(names):
        raise ValueError(f"PLY vertex is missing coordinates {sorted(required - names)}: {path}")
    positions = np.column_stack((vertex["x"], vertex["y"], vertex["z"])).astype(
        np.float32
    )
    if {"red", "green", "blue"}.issubset(names):
        colors = np.column_stack(
            (vertex["red"], vertex["green"], vertex["blue"])
        ).astype(np.uint8)
    elif "rgba" in names:
        colors = packed_colors(vertex["rgba"])
    elif "rgb" in names:
        colors = packed_colors(vertex["rgb"])
    else:
        colors = np.full((len(positions), 3), 180, dtype=np.uint8)
    finite = np.isfinite(positions).all(axis=1)
    positions = positions[finite]
    colors = colors[finite]
    if len(positions) == 0:
        raise ValueError(f"exported point cloud has no finite points: {path}")
    return positions, colors


def align_to_reference(trajectory, reference, name):
    associated, associated_reference = sync.associate_trajectories(
        copy.deepcopy(trajectory),
        copy.deepcopy(reference),
        max_diff=0.01,
        first_name=name,
        snd_name="ground truth",
    )
    if len(associated.timestamps) < 3:
        raise ValueError(f"too few associated poses for {name}: {len(associated.timestamps)}")
    rotation, translation, scale = associated.align(
        associated_reference, correct_scale=False
    )
    aligned = copy.deepcopy(trajectory)
    aligned.transform(lie_algebra.se3(rotation, translation))
    LOGGER.info(
        "%s alignment: matches=%d scale=%.6f translation=%s",
        name,
        len(associated.timestamps),
        scale,
        np.array2string(translation, precision=6),
    )
    return aligned, rotation, translation


def sampled_indices(timestamps, rate):
    interval = 1.0 / rate
    indices = []
    last_timestamp = float("-inf")
    for index, timestamp in enumerate(timestamps):
        if float(timestamp) - last_timestamp + 1e-9 >= interval:
            indices.append(index)
            last_timestamp = float(timestamp)
    if indices[-1] != len(timestamps) - 1:
        indices.append(len(timestamps) - 1)
    return np.asarray(indices, dtype=np.int64)


def log_timeline_trajectory(recording, name, trajectory, rate):
    trajectory_path = f"world/trajectories/{name}"
    current_path = f"{trajectory_path}/current"
    recording.log(
        f"{current_path}/marker",
        rr.Points3D(
            [[0.0, 0.0, 0.0]],
            colors=TRAJECTORY_COLORS[name],
            radii=0.025 if name != "optimized" else 0.035,
            show_labels=False,
        ),
        static=True,
    )
    if name == "optimized":
        recording.log(
            f"{current_path}/axes",
            rr.Arrows3D(
                origins=np.zeros((3, 3)),
                vectors=np.eye(3) * 0.25,
                colors=[[255, 60, 60], [60, 220, 80], [60, 140, 255]],
                radii=0.008,
                show_labels=False,
            ),
            static=True,
        )
        recording.log(
            "plots/orientation/roll",
            rr.SeriesLines(colors=[255, 100, 100], names="roll"),
            static=True,
        )
        recording.log(
            "plots/orientation/pitch",
            rr.SeriesLines(colors=[100, 220, 120], names="pitch"),
            static=True,
        )
        recording.log(
            "plots/orientation/yaw",
            rr.SeriesLines(colors=[80, 170, 255], names="yaw"),
            static=True,
        )

    indices = sampled_indices(trajectory.timestamps, rate)
    timestamps = trajectory.timestamps[indices]
    positions = trajectory.positions_xyz[indices]
    quaternions_xyzw = np.roll(
        trajectory.orientations_quat_wxyz[indices], -1, axis=1
    )
    angles_deg = None
    if name == "optimized":
        angles_deg = np.rad2deg(
            np.unwrap(
                Rotation.from_quat(quaternions_xyzw).as_euler("xyz"), axis=0
            )
        )
    for sample_index, (timestamp, position, quaternion_xyzw) in enumerate(
        zip(
            timestamps,
            positions,
            quaternions_xyzw,
        )
    ):
        recording.set_time("timestamp", timestamp=float(timestamp))
        recording.log(
            current_path,
            rr.Transform3D(
                translation=position,
                quaternion=rr.Quaternion(xyzw=quaternion_xyzw),
            ),
        )
        recording.log(
            trajectory_path,
            rr.LineStrips3D(
                [positions[: sample_index + 1]],
                colors=TRAJECTORY_COLORS[name],
                radii=0.006 if name == "optimized" else 0.003,
                show_labels=False,
            ),
        )
        if angles_deg is not None:
            roll, pitch, yaw = angles_deg[sample_index]
            recording.log("plots/orientation/roll", rr.Scalars(roll))
            recording.log("plots/orientation/pitch", rr.Scalars(pitch))
            recording.log("plots/orientation/yaw", rr.Scalars(yaw))
    LOGGER.info(
        "logged progressive trajectory: name=%s poses=%d rate<=%.3fHz",
        name,
        len(indices),
        rate,
    )


def log_progressive_cloud(
    recording, points, colors, trajectory, point_radius, map_rate
):
    nearest_pose = cKDTree(trajectory.positions_xyz).query(points, workers=-1)[1]
    point_timestamps = trajectory.timestamps[nearest_pose]
    start_timestamp = float(trajectory.timestamps[0])
    batch_ids = np.floor((point_timestamps - start_timestamp) * map_rate).astype(
        np.int64
    )
    unique_batches = np.unique(batch_ids)
    for batch_id in unique_batches:
        selection = batch_ids == batch_id
        timestamp = start_timestamp + float(batch_id) / map_rate
        recording.set_time("timestamp", timestamp=float(timestamp))
        recording.log(
            f"world/map/batches/{batch_id:04d}",
            rr.Points3D(
                points[selection], colors=colors[selection], radii=point_radius
            ),
        )
    LOGGER.info(
        "logged progressive cloud: points=%d time_batches=%d rate<=%.3fHz",
        len(points),
        len(unique_batches),
        map_rate,
    )


def message_time_seconds(message):
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def encode_mono_jpeg(message, quality):
    if message.encoding not in ("mono8", "8UC1"):
        raise ValueError(f"unsupported rectified image encoding: {message.encoding}")
    pixels = np.frombuffer(message.data, dtype=np.uint8)
    expected = int(message.height) * int(message.step)
    if pixels.size < expected:
        raise ValueError(
            f"short image buffer: got {pixels.size} bytes, expected at least {expected}"
        )
    pixels = pixels[:expected].reshape(int(message.height), int(message.step))
    pixels = np.ascontiguousarray(pixels[:, : int(message.width)])
    encoded = io.BytesIO()
    Image.fromarray(pixels).save(
        encoded, format="JPEG", quality=quality, optimize=True
    )
    return encoded.getvalue()


def log_camera_images(recording, bag_path, image_rate, jpeg_quality):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image as RosImage
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python modules are required to include camera images; "
            "source /opt/ros/humble/setup.bash or use --no-images"
        ) from error

    topics = {
        "/cam0/image_rect": "cameras/left/image",
        "/cam1/image_rect": "cameras/right/image",
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    minimum_interval = 1.0 / image_rate
    last_timestamp = {topic: float("-inf") for topic in topics}
    counts = {topic: 0 for topic in topics}
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        message = deserialize_message(serialized, RosImage)
        timestamp = message_time_seconds(message)
        if timestamp - last_timestamp[topic] + 1e-9 < minimum_interval:
            continue
        last_timestamp[topic] = timestamp
        counts[topic] += 1
        recording.set_time("timestamp", timestamp=timestamp)
        recording.log(
            topics[topic],
            rr.EncodedImage(
                contents=encode_mono_jpeg(message, jpeg_quality),
                media_type="image/jpeg",
            ),
        )
    LOGGER.info(
        "logged rectified images: left=%d right=%d rate<=%.3fHz quality=%d",
        counts["/cam0/image_rect"],
        counts["/cam1/image_rect"],
        image_rate,
        jpeg_quality,
    )


def send_dashboard_blueprint(recording):
    main_view = rrb.Spatial3DView(
        name="SLAM map",
        origin="world",
        contents=[
            "world/map/**",
            "world/trajectories/**",
        ],
        line_grid=True,
    )
    orientation_view = rrb.TimeSeriesView(
        name="Roll / Pitch / Yaw (deg)",
        origin="plots/orientation",
        contents="plots/orientation/**",
    )
    left_view = rrb.Spatial2DView(
        name="Left rectified", origin="cameras/left", contents="cameras/left/**"
    )
    right_view = rrb.Spatial2DView(
        name="Right rectified", origin="cameras/right", contents="cameras/right/**"
    )
    recording.send_blueprint(
        rrb.Blueprint(
            rrb.Horizontal(
                rrb.Vertical(main_view, orientation_view, row_shares=[3, 1]),
                rrb.Vertical(left_view, right_view),
                column_shares=[3, 1],
            ),
            rrb.TimePanel(timeline="timestamp", playback_speed=1.0, expanded=True),
            collapse_panels=True,
        ),
        make_active=True,
        make_default=True,
    )


def write_recording(
    output,
    run_name,
    points,
    colors,
    raw,
    optimized,
    ground_truth,
    point_radius,
    image_bag,
    image_rate,
    map_rate,
    trajectory_rate,
    jpeg_quality,
):
    if output.exists():
        output.unlink()
    recording = rr.RecordingStream("hno_vio_rtabmap", recording_id=run_name)
    recording.save(output)
    try:
        send_dashboard_blueprint(recording)
        recording.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        log_progressive_cloud(
            recording, points, colors, optimized, point_radius, map_rate
        )
        trajectories = {
            "ground_truth": ground_truth,
            "raw": raw,
            "optimized": optimized,
        }
        for name, trajectory in trajectories.items():
            log_timeline_trajectory(recording, name, trajectory, trajectory_rate)
        if image_bag is not None:
            log_camera_images(recording, image_bag, image_rate, jpeg_quality)
        recording.flush()
    finally:
        recording.disconnect()
    require_file(output, "Rerun recording")


def main():
    args = parse_args()
    if args.voxel <= 0.0:
        raise ValueError("--voxel must be greater than zero")
    if args.point_radius <= 0.0:
        raise ValueError("--point-radius must be greater than zero")
    if args.image_rate <= 0.0:
        raise ValueError("--image-rate must be greater than zero")
    if args.map_rate <= 0.0:
        raise ValueError("--map-rate must be greater than zero")
    if args.trajectory_rate <= 0.0:
        raise ValueError("--trajectory-rate must be greater than zero")
    if not 1 <= args.jpeg_quality <= 100:
        raise ValueError("--jpeg-quality must be between 1 and 100")

    run_dir = Path(args.run_dir).expanduser().resolve()
    if not run_dir.is_dir():
        raise NotADirectoryError(f"run directory not found: {run_dir}")
    visual_dir = run_dir / "visual_results"
    visual_dir.mkdir(parents=True, exist_ok=True)
    configure_logging(visual_dir / "conversion.log")

    database = run_dir / "offline_results" / "rtabmap.db"
    context_path = run_dir / "run_context.json"
    require_file(database, "RTAB-Map database")
    require_file(context_path, "run_context.json")
    context = json.loads(context_path.read_text(encoding="utf-8"))

    package_root = Path(__file__).resolve().parents[2]
    raw_path = resolve_run_file(
        run_dir,
        context.get("odom_tum"),
        run_dir / "vio_results" / "odom_raw.txt",
    )
    dataset = context.get("dataset")
    if not dataset:
        raise ValueError(f"dataset is missing in {context_path}")
    ground_truth_path = resolve_run_file(
        run_dir,
        context.get("ground_truth_tum"),
        package_root / "ground_truth" / "euroc_mav" / f"{dataset}.txt",
    )
    require_file(raw_path, "raw odometry trajectory")
    require_file(ground_truth_path, "ground truth trajectory")
    image_bag = run_dir / "vio_results" / "rtabmap_input_db3"
    if not args.no_images:
        require_file(image_bag / "metadata.yaml", "RTAB-Map input bag metadata")

    output = (
        Path(args.output).expanduser().resolve()
        if args.output
        else visual_dir / "rtabmap.rrd"
    )
    output.parent.mkdir(parents=True, exist_ok=True)

    LOGGER.info("run_dir=%s", run_dir)
    LOGGER.info("database=%s", database)
    LOGGER.info("raw_trajectory=%s", raw_path)
    LOGGER.info("ground_truth=%s", ground_truth_path)
    LOGGER.info("output=%s", output)

    with tempfile.TemporaryDirectory(prefix="hno_rerun_") as temporary:
        cloud_path, optimized_path = export_rtabmap(
            database, Path(temporary), args.voxel
        )
        points, colors = load_point_cloud(cloud_path)
        raw = file_interface.read_tum_trajectory_file(raw_path)
        optimized = file_interface.read_tum_trajectory_file(optimized_path)
        ground_truth = file_interface.read_tum_trajectory_file(ground_truth_path)

        raw_aligned, _, _ = align_to_reference(raw, ground_truth, "raw")
        optimized_aligned, rotation, translation = align_to_reference(
            optimized, ground_truth, "optimized"
        )
        points_aligned = (rotation @ points.T).T + translation

        LOGGER.info(
            "loaded points=%d raw_poses=%d optimized_poses=%d gt_poses=%d",
            len(points_aligned),
            len(raw_aligned.timestamps),
            len(optimized_aligned.timestamps),
            len(ground_truth.timestamps),
        )
        write_recording(
            output,
            run_dir.name,
            points_aligned,
            colors,
            raw_aligned,
            optimized_aligned,
            ground_truth,
            args.point_radius,
            None if args.no_images else image_bag,
            args.image_rate,
            args.map_rate,
            args.trajectory_rate,
            args.jpeg_quality,
        )

    LOGGER.info("Rerun recording created: %s (%d bytes)", output, output.stat().st_size)
    if not args.no_open:
        rerun_cli = require_command("rerun")
        LOGGER.info("opening Rerun Viewer: %s", output)
        subprocess.run([rerun_cli, str(output)], check=True)


if __name__ == "__main__":
    main()

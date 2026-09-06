#!/usr/bin/env python3
"""Convert one HNO-VIO RTAB-Map run into a self-contained Rerun recording."""

import argparse
import copy
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
from evo.core import lie_algebra, sync
from evo.tools import file_interface
from plyfile import PlyData


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
        default=0.05,
        help="RTAB-Map cloud voxel size in meters (default: 0.05)",
    )
    parser.add_argument(
        "--point-radius",
        type=float,
        default=0.025,
        help="Rerun point radius in meters (default: 0.025)",
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


def log_static_trajectory(recording, name, trajectory):
    recording.log(
        f"world/trajectories/{name}",
        rr.LineStrips3D(
            [trajectory.positions_xyz],
            colors=TRAJECTORY_COLORS[name],
            radii=0.012,
            labels=[name],
        ),
        static=True,
    )


def log_timeline_trajectory(recording, name, trajectory):
    entity_path = f"world/current/{name}"
    recording.log(
        f"{entity_path}/marker",
        rr.Points3D(
            [[0.0, 0.0, 0.0]],
            colors=TRAJECTORY_COLORS[name],
            radii=0.06,
            labels=[name],
        ),
        static=True,
    )
    for timestamp, position, quaternion_wxyz in zip(
        trajectory.timestamps,
        trajectory.positions_xyz,
        trajectory.orientations_quat_wxyz,
    ):
        recording.set_time("timestamp", timestamp=float(timestamp))
        quaternion_xyzw = np.roll(quaternion_wxyz, -1)
        recording.log(
            entity_path,
            rr.Transform3D(
                translation=position,
                quaternion=rr.Quaternion(xyzw=quaternion_xyzw),
            ),
        )


def write_recording(
    output, run_name, points, colors, raw, optimized, ground_truth, point_radius
):
    if output.exists():
        output.unlink()
    recording = rr.RecordingStream("hno_vio_rtabmap", recording_id=run_name)
    recording.save(output)
    try:
        recording.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        recording.log(
            "world/rtabmap_cloud",
            rr.Points3D(points, colors=colors, radii=point_radius),
            static=True,
        )
        trajectories = {
            "ground_truth": ground_truth,
            "raw": raw,
            "optimized": optimized,
        }
        for name, trajectory in trajectories.items():
            log_static_trajectory(recording, name, trajectory)
            log_timeline_trajectory(recording, name, trajectory)
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
        )

    LOGGER.info("Rerun recording created: %s (%d bytes)", output, output.stat().st_size)
    if not args.no_open:
        rerun_cli = require_command("rerun")
        LOGGER.info("opening Rerun Viewer: %s", output)
        subprocess.run([rerun_cli, str(output)], check=True)


if __name__ == "__main__":
    main()

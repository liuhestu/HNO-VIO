import argparse
from pathlib import Path

from .euroc_utils import read_euroc_image_csv
from .odom_utils import read_hno_odom_csv, read_tum, validate_poses, write_standard_csv, write_tum


def write_report(path, stats, cam_start_ns, cam_end_ns):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for key in [
        "row_count",
        "start_timestamp_ns",
        "end_timestamp_ns",
        "duration_sec",
        "mean_hz",
        "first_cam0_timestamp_ns",
        "last_cam0_timestamp_ns",
        "odom_vs_cam0_start_diff_sec",
        "odom_vs_cam0_end_diff_sec",
        "quaternion_norm_bad_count",
        "nan_count",
        "max_step_translation_m",
        "max_step_rotation_deg",
        "max_step_time_sec",
        "large_step_count",
    ]:
        lines.append(f"{key}: {stats[key]}")
    lines.append(f"cam0_range_ns: {cam_start_ns} {cam_end_ns}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-csv", required=True)
    parser.add_argument("--input-tum", required=True)
    parser.add_argument("--euroc-mav0", required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--output-tum", required=True)
    parser.add_argument("--report", required=True)
    parser.add_argument("--max-step-translation", type=float, default=0.5)
    parser.add_argument("--max-step-rotation-deg", type=float, default=30.0)
    args = parser.parse_args()

    input_csv = Path(args.input_csv)
    input_tum = Path(args.input_tum)
    if input_csv.exists() and input_csv.stat().st_size > 0:
        poses = read_hno_odom_csv(input_csv)
        source = input_csv
    elif input_tum.exists() and input_tum.stat().st_size > 0:
        poses = read_tum(input_tum)
        source = input_tum
    else:
        raise RuntimeError(f"no usable odom source: {input_csv} or {input_tum}")

    cam_rows = read_euroc_image_csv(Path(args.euroc_mav0) / "cam0" / "data.csv")
    if not cam_rows:
        raise RuntimeError("cam0/data.csv is empty")
    cam_start_ns = cam_rows[0][0]
    cam_end_ns = cam_rows[-1][0]
    stats = validate_poses(
        poses,
        cam_start_ns=cam_start_ns,
        cam_end_ns=cam_end_ns,
        max_step_translation=args.max_step_translation,
        max_step_rotation_deg=args.max_step_rotation_deg,
    )
    stats["first_cam0_timestamp_ns"] = cam_start_ns
    stats["last_cam0_timestamp_ns"] = cam_end_ns
    stats["odom_vs_cam0_start_diff_sec"] = (poses[0].stamp_ns - cam_start_ns) * 1e-9
    stats["odom_vs_cam0_end_diff_sec"] = (poses[-1].stamp_ns - cam_end_ns) * 1e-9

    write_standard_csv(args.output_csv, poses)
    write_tum(args.output_tum, poses)
    write_report(args.report, stats, cam_start_ns, cam_end_ns)
    print(f"source: {source}")
    print(f"standard_csv: {args.output_csv}")
    print(f"standard_tum: {args.output_tum}")
    print(f"report: {args.report}")


if __name__ == "__main__":
    main()

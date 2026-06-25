"""Evaluate raw HNO-VIO odometry against RTAB-Map optimized odometry.

Usage:
    hno_eval_raw_vs_optimized --out-dir OUT_DIR --raw-tum ODOM_RAW_TXT \
        --optimized-tum ODOM_OPTIMIZED_TXT --gt-tum GROUND_TRUTH_TUM

Inputs:
    --raw-tum: Raw HNO-VIO trajectory in TUM/TXT format.
    --optimized-tum: RTAB-Map optimized trajectory in TUM/TXT format.
    --gt-tum: Ground-truth trajectory in TUM/TXT format.
    Optional: --evo-ape, --evo-rpe, --evo-traj.

Outputs:
    OUT_DIR/summary.md
    OUT_DIR/evo_*.txt
    OUT_DIR/evo_*.pdf
    OUT_DIR/evo_traj_gt_raw_optimized.pdf
    OUT_DIR/logs/rpy_raw_vs_optimized.csv
    OUT_DIR/logs/rpy_raw_vs_optimized.pdf
    OUT_DIR/logs/traj_raw_vs_optimized_matplotlib.pdf
"""

import argparse
import csv
import math
import re
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .euroc_utils import read_gt_csv
from .odom_utils import OdomPose, read_tum, write_tum
from .tf_utils import angle_diff_degrees, quat_to_rpy_deg, unwrap_degrees


def run_cmd(cmd, output_path):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    proc = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
    output_path.write_text("$ " + " ".join(cmd) + "\n\n" + proc.stdout, encoding="utf-8")
    if proc.returncode != 0:
        raise RuntimeError(f"command failed: {' '.join(cmd)}\nsee {output_path}")
    return proc.stdout


def write_evo_ape_plots(evo_ape, gt_tum, trajectories, out_dir):
    plot_dir = out_dir / "evo_plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    config_path = plot_dir / "evo_agg.json"
    config_path.write_text('{\n  "plot_backend": "Agg"\n}\n', encoding="utf-8")

    for name, tum_path in trajectories:
        for suffix in ("pdf", "png"):
            run_cmd(
                [
                    evo_ape,
                    "tum",
                    str(gt_tum),
                    str(tum_path),
                    "--align",
                    "--correct_scale",
                    "--plot",
                    "--plot_mode",
                    "xy",
                    "--save_plot",
                    str(plot_dir / f"ape_{name}_xy.{suffix}"),
                    "--silent",
                    "--no_warnings",
                    "-c",
                    str(config_path),
                ],
                plot_dir / f"ape_{name}_xy_{suffix}.log",
            )
        run_cmd(
            [
                evo_ape,
                "tum",
                str(gt_tum),
                str(tum_path),
                "--align",
                "--correct_scale",
                "--save_results",
                str(plot_dir / f"ape_{name}.zip"),
                "--silent",
                "--no_warnings",
            ],
            plot_dir / f"ape_{name}_zip.log",
        )


def parse_metric(text, name):
    match = re.search(rf"^\s*{re.escape(name)}\s+([-+0-9.eE]+)", text, re.MULTILINE)
    return float(match.group(1)) if match else math.nan


def path_length(poses):
    total = 0.0
    for a, b in zip(poses, poses[1:]):
        total += float(np.linalg.norm(b.p - a.p))
    return total


def umeyama(source, target, with_scale=True):
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    if len(source) < 3 or len(target) != len(source):
        return 1.0, np.eye(3), np.zeros(3)
    mu_src = source.mean(axis=0)
    mu_tgt = target.mean(axis=0)
    src_centered = source - mu_src
    tgt_centered = target - mu_tgt
    cov = (tgt_centered.T @ src_centered) / len(source)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1.0
    R = U @ S @ Vt
    if with_scale:
        var_src = np.sum(src_centered * src_centered) / len(source)
        scale = np.trace(np.diag(D) @ S) / var_src if var_src > 1e-12 else 1.0
    else:
        scale = 1.0
    t = mu_tgt - scale * R @ mu_src
    return scale, R, t


def aligned_positions(est, ref):
    est_pts = []
    ref_pts = []
    for pose in est:
        ref_pose = nearest_pose(ref, pose.stamp_ns)
        if ref_pose is None:
            continue
        est_pts.append(pose.p)
        ref_pts.append(ref_pose.p)
    scale, R, t = umeyama(est_pts, ref_pts, with_scale=True)
    return np.array([scale * R @ pose.p + t for pose in est])


def nearest_pose(poses, stamp_ns, max_diff_ns=50_000_000):
    if not poses:
        return None
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
    best = min(candidates, key=lambda pose: abs(pose.stamp_ns - stamp_ns))
    if abs(best.stamp_ns - stamp_ns) > max_diff_ns:
        return None
    return best


def load_tum(path):
    poses = read_tum(path)
    poses.sort(key=lambda pose: pose.stamp_ns)
    return poses


def write_gt_tum(gt_csv, gt_tum):
    gt = [OdomPose(stamp_ns, p, q) for stamp_ns, p, q in read_gt_csv(gt_csv)]
    write_tum(gt_tum, gt)
    return gt


def plot_traj(raw, opt, gt, out_path):
    plt.figure(figsize=(8, 7))
    gt_xy = np.array([pose.p[:2] for pose in gt])
    raw_xyz = aligned_positions(raw, gt)
    opt_xyz = aligned_positions(opt, gt)
    for xy, label, color in [
        (gt_xy, "GT", "black"),
        (raw_xyz[:, :2] if raw_xyz.size else raw_xyz, "HNO-VIO raw aligned", "tab:orange"),
        (opt_xyz[:, :2] if opt_xyz.size else opt_xyz, "RTAB-Map optimized aligned", "tab:blue"),
    ]:
        if xy.size == 0:
            continue
        plt.plot(xy[:, 0], xy[:, 1], label=label, linewidth=1.2, color=color)
    plt.axis("equal")
    plt.grid(True, linewidth=0.3)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_path)
    plt.close()


def raw_optimized_delta_stats(raw, opt):
    trans = []
    rot = []
    for a, b in zip(raw, opt):
        trans.append(float(np.linalg.norm(a.p - b.p)))
        Ra = np.array([
            quat_to_rpy_deg(a.q),
        ])
        Rb = np.array([
            quat_to_rpy_deg(b.q),
        ])
        diff = np.abs(Ra - Rb)
        diff = np.minimum(diff, 360.0 - diff)
        rot.append(float(np.linalg.norm(diff)))
    return {
        "translation_mean": float(np.mean(trans)) if trans else math.nan,
        "translation_max": float(np.max(trans)) if trans else math.nan,
        "rotation_rpy_norm_mean_deg": float(np.mean(rot)) if rot else math.nan,
        "rotation_rpy_norm_max_deg": float(np.max(rot)) if rot else math.nan,
    }


def nearest_pose_index(poses, stamp_ns, max_diff_ns=50_000_000):
    if not poses:
        return None
    lo, hi = 0, len(poses)
    while lo < hi:
        mid = (lo + hi) // 2
        if poses[mid].stamp_ns < stamp_ns:
            lo = mid + 1
        else:
            hi = mid
    candidates = []
    if lo < len(poses):
        candidates.append(lo)
    if lo > 0:
        candidates.append(lo - 1)
    best = min(candidates, key=lambda idx: abs(poses[idx].stamp_ns - stamp_ns))
    if abs(poses[best].stamp_ns - stamp_ns) > max_diff_ns:
        return None
    return best


def unwrap_rpy_degrees(rpy):
    rpy = np.asarray(rpy, dtype=float).copy()
    if not rpy.size:
        return rpy
    for axis in range(3):
        rpy[:, axis] = unwrap_degrees(rpy[:, axis])
    return rpy


def align_rpy_branch_to_reference(poses, rpy, ref_poses, ref_rpy):
    rpy = np.asarray(rpy, dtype=float).copy()
    if not rpy.size or not ref_rpy.size:
        return rpy
    for i, pose in enumerate(poses):
        ref_idx = nearest_pose_index(ref_poses, pose.stamp_ns)
        if ref_idx is None:
            continue
        for axis in range(3):
            ref_value = ref_rpy[ref_idx, axis]
            rpy[i, axis] = ref_value + angle_diff_degrees(rpy[i, axis], ref_value)
    return rpy


def plot_rpy(raw, opt, gt, csv_path, pdf_path):
    raw_rpy = unwrap_rpy_degrees(np.array([quat_to_rpy_deg(pose.q) for pose in raw]))
    opt_rpy = unwrap_rpy_degrees(np.array([quat_to_rpy_deg(pose.q) for pose in opt]))
    gt_rpy = unwrap_rpy_degrees(np.array([quat_to_rpy_deg(pose.q) for pose in gt]))
    opt_rpy = align_rpy_branch_to_reference(opt, opt_rpy, raw, raw_rpy)
    gt_rpy = align_rpy_branch_to_reference(gt, gt_rpy, raw, raw_rpy)

    opt_by_time = opt
    gt_by_time = gt
    with Path(csv_path).open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp_sec",
            "raw_roll_deg", "raw_pitch_deg", "raw_yaw_deg",
            "optimized_roll_deg", "optimized_pitch_deg", "optimized_yaw_deg",
            "gt_roll_deg", "gt_pitch_deg", "gt_yaw_deg",
        ])
        for i, pose in enumerate(raw):
            opt_idx = nearest_pose_index(opt_by_time, pose.stamp_ns)
            gt_idx = nearest_pose_index(gt_by_time, pose.stamp_ns)
            opt_vals = [math.nan, math.nan, math.nan]
            gt_vals = [math.nan, math.nan, math.nan]
            if opt_idx is not None and opt_rpy.size:
                opt_vals = opt_rpy[opt_idx].tolist()
            if gt_idx is not None and gt_rpy.size:
                gt_vals = gt_rpy[gt_idx].tolist()
            writer.writerow([pose.stamp_sec, *raw_rpy[i].tolist(), *opt_vals, *gt_vals])

    t0 = raw[0].stamp_sec if raw else 0.0
    plt.figure(figsize=(10, 8))
    labels = ["roll", "pitch", "yaw"]
    for axis, label in enumerate(labels):
        ax = plt.subplot(3, 1, axis + 1)
        if raw_rpy.size:
            ax.plot([p.stamp_sec - t0 for p in raw], raw_rpy[:, axis], label="raw", color="tab:orange", linewidth=0.8)
        if opt_rpy.size:
            ax.plot([p.stamp_sec - t0 for p in opt], opt_rpy[:, axis], label="optimized", color="tab:blue", linewidth=0.8)
        if gt_rpy.size:
            ax.plot([p.stamp_sec - t0 for p in gt], gt_rpy[:, axis], label="GT", color="black", linewidth=0.8)
        ax.grid(True, linewidth=0.3)
        ax.set_ylabel(f"{label} [deg]")
        if axis == 0:
            ax.legend(loc="best")
    plt.xlabel("time [s]")
    plt.tight_layout()
    plt.savefig(pdf_path)
    plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--raw-tum", required=True)
    parser.add_argument("--optimized-tum", required=True)
    parser.add_argument("--gt-tum", required=True)
    parser.add_argument("--evo-ape", default="evo_ape")
    parser.add_argument("--evo-rpe", default="evo_rpe")
    parser.add_argument("--evo-traj", default="evo_traj")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    log_dir = out_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    raw_tum = Path(args.raw_tum)
    opt_tum = Path(args.optimized_tum)
    gt_tum = Path(args.gt_tum)

    gt = load_tum(gt_tum)
    raw = load_tum(raw_tum)
    opt = load_tum(opt_tum)

    if len(opt) < 20:
        raise RuntimeError(f"optimized graph pose count < 20: {len(opt)}")
    graph_warning = ""
    if len(opt) < 50:
        graph_warning = f"WARNING: optimized graph pose count {len(opt)} < 50"

    ape_raw = run_cmd([args.evo_ape, "tum", str(gt_tum), str(raw_tum), "--align", "--correct_scale"], out_dir / "evo_ape_raw.txt")
    ape_opt = run_cmd([args.evo_ape, "tum", str(gt_tum), str(opt_tum), "--align", "--correct_scale"], out_dir / "evo_ape_optimized.txt")

    rpe_raw_trans = run_cmd([args.evo_rpe, "tum", str(gt_tum), str(raw_tum), "-r", "trans_part", "-d", "20", "-u", "f", "--align", "--correct_scale"], log_dir / "evo_rpe_raw_trans.txt")
    rpe_raw_rot = run_cmd([args.evo_rpe, "tum", str(gt_tum), str(raw_tum), "-r", "angle_deg", "-d", "20", "-u", "f", "--align", "--correct_scale"], log_dir / "evo_rpe_raw_rot.txt")
    rpe_opt_trans = run_cmd([args.evo_rpe, "tum", str(gt_tum), str(opt_tum), "-r", "trans_part", "-d", "20", "-u", "f", "--align", "--correct_scale"], log_dir / "evo_rpe_optimized_trans.txt")
    rpe_opt_rot = run_cmd([args.evo_rpe, "tum", str(gt_tum), str(opt_tum), "-r", "angle_deg", "-d", "20", "-u", "f", "--align", "--correct_scale"], log_dir / "evo_rpe_optimized_rot.txt")

    (out_dir / "evo_rpe_raw.txt").write_text(
        "# trans_part @20 frames, about 1s at 20Hz\n" + rpe_raw_trans + "\n# angle_deg @20 frames, about 1s at 20Hz\n" + rpe_raw_rot,
        encoding="utf-8",
    )
    (out_dir / "evo_rpe_optimized.txt").write_text(
        "# trans_part @20 frames, about 1s at 20Hz\n" + rpe_opt_trans + "\n# angle_deg @20 frames, about 1s at 20Hz\n" + rpe_opt_rot,
        encoding="utf-8",
    )

    plot_rpy(raw, opt, gt, log_dir / "rpy_raw_vs_optimized.csv", log_dir / "rpy_raw_vs_optimized.pdf")
    plot_traj(raw, opt, gt, log_dir / "traj_raw_vs_optimized_matplotlib.pdf")

    evo_cfg = log_dir / "evo_agg.json"
    evo_cfg.write_text('{\n  "plot_backend": "Agg"\n}\n', encoding="utf-8")
    run_cmd(
        [
            args.evo_ape,
            "tum",
            str(gt_tum),
            str(opt_tum),
            "--align",
            "--correct_scale",
            "--plot",
            "--plot_mode",
            "xyz",
            "--save_plot",
            str(out_dir / "evo_ape_optimized.pdf"),
            "--silent",
            "--no_warnings",
            "-c",
            str(evo_cfg),
        ],
        log_dir / "evo_ape_optimized_pdf.log",
    )
    run_cmd(
        [
            args.evo_rpe,
            "tum",
            str(gt_tum),
            str(opt_tum),
            "-r",
            "trans_part",
            "-d",
            "20",
            "-u",
            "f",
            "--align",
            "--correct_scale",
            "--plot",
            "--plot_mode",
            "xyz",
            "--save_plot",
            str(out_dir / "evo_rpe_trans_optimized.pdf"),
            "--silent",
            "--no_warnings",
            "-c",
            str(evo_cfg),
        ],
        log_dir / "evo_rpe_trans_optimized_pdf.log",
    )
    run_cmd(
        [
            args.evo_rpe,
            "tum",
            str(gt_tum),
            str(opt_tum),
            "-r",
            "angle_deg",
            "-d",
            "20",
            "-u",
            "f",
            "--align",
            "--correct_scale",
            "--plot",
            "--plot_mode",
            "xyz",
            "--save_plot",
            str(out_dir / "evo_rpe_rot_optimized.pdf"),
            "--silent",
            "--no_warnings",
            "-c",
            str(evo_cfg),
        ],
        log_dir / "evo_rpe_rot_optimized_pdf.log",
    )
    run_cmd(
        [
            args.evo_traj,
            "tum",
            str(raw_tum),
            str(opt_tum),
            "--ref",
            str(gt_tum),
            "--align",
            "--correct_scale",
            "--plot",
            "--plot_mode",
            "xyz",
            "--save_plot",
            str(out_dir / "evo_traj_gt_raw_optimized.pdf"),
            "--silent",
            "--no_warnings",
            "-c",
            str(evo_cfg),
        ],
        log_dir / "evo_traj_gt_raw_optimized_pdf.log",
    )
    raw_len = path_length(raw)
    opt_len = path_length(opt)
    duration = raw[-1].stamp_sec - raw[0].stamp_sec if len(raw) >= 2 else 0.0
    final_position_error = float(np.linalg.norm(raw[-1].p - opt[-1].p)) if raw and opt else math.nan
    delta_stats = raw_optimized_delta_stats(raw, opt)

    lines = [
        "# RTAB-Map Offline Evaluation",
        "",
        f"Raw odometry: `{raw_tum}`.",
        f"Optimized odometry: `{opt_tum}`.",
        f"Ground truth: `{gt_tum}`.",
        "",
        "| metric | raw | optimized |",
        "| --- | ---: | ---: |",
        f"| ATE RMSE [m] | {parse_metric(ape_raw, 'rmse'):.6f} | {parse_metric(ape_opt, 'rmse'):.6f} |",
        f"| ATE mean [m] | {parse_metric(ape_raw, 'mean'):.6f} | {parse_metric(ape_opt, 'mean'):.6f} |",
        f"| ATE median [m] | {parse_metric(ape_raw, 'median'):.6f} | {parse_metric(ape_opt, 'median'):.6f} |",
        f"| RPE trans RMSE @20 frames (~1s) [m] | {parse_metric(rpe_raw_trans, 'rmse'):.6f} | {parse_metric(rpe_opt_trans, 'rmse'):.6f} |",
        f"| RPE rot RMSE @20 frames (~1s) [deg] | {parse_metric(rpe_raw_rot, 'rmse'):.6f} | {parse_metric(rpe_opt_rot, 'rmse'):.6f} |",
        f"| path length [m] | {raw_len:.6f} | {opt_len:.6f} |",
        f"| optimized/raw path length ratio | 1.000000 | {(opt_len / raw_len) if raw_len > 0 else math.nan:.6f} |",
        f"| pose count | {len(raw)} | {len(opt)} |",
        f"| duration [s] | {duration:.6f} | {duration:.6f} |",
        f"| final raw-vs-optimized position delta [m] | {final_position_error:.6f} | {final_position_error:.6f} |",
        f"| raw-vs-optimized mean position delta [m] | {delta_stats['translation_mean']:.9f} | {delta_stats['translation_mean']:.9f} |",
        f"| raw-vs-optimized max position delta [m] | {delta_stats['translation_max']:.9f} | {delta_stats['translation_max']:.9f} |",
        f"| optimized graph pose count | {len(opt)} | {len(opt)} |",
    ]
    if graph_warning:
        lines.extend(["", graph_warning])
    lines.extend([
        "",
        "Notes:",
        "- Evaluation uses `evo_ape/evo_rpe --align --correct_scale` for shape comparison.",
        "- `odom_optimized.txt` is exported from `/rtabmap/mapData.graph.poses`.",
        "- The primary RTAB-Map odometry input is TF `odom -> base_link`; `/hno_vio/odom` is recorded for export/debugging.",
        "- `evo_traj_gt_raw_optimized.pdf` is generated by evo_traj with xyz components; use `logs/rpy_raw_vs_optimized.pdf` for branch-safe RPY.",
        "- RPY plot and CSV unwrap each curve and place optimized/GT on the nearest 360-degree branch of raw before visualization.",
    ])
    summary = out_dir / "summary.md"
    summary.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(summary)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import argparse
import json
import math
import re
import subprocess
from bisect import bisect_left
from pathlib import Path


USABLE = {
    "se3_ate_rmse": 2.0,
    "se3_ate_median": 1.5,
    "rpe1_trans_rmse": 0.50,
    "rpe1_rot_rmse_deg": 10.0,
    "duration_sec": 130.0,
}

STRONG = {
    "se3_ate_rmse": 1.0,
    "se3_ate_median": 0.8,
    "rpe1_trans_rmse": 0.25,
    "rpe1_rot_rmse_deg": 5.0,
    "duration_sec": 130.0,
}


def read_json(path, default):
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return default


def read_tum(path):
    poses = []
    if not path.exists():
        return poses
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        t = float(parts[0])
        if t > 1e12:
            t *= 1e-9
        poses.append((t, (float(parts[1]), float(parts[2]), float(parts[3]))))
    return poses


def path_length(poses):
    total = 0.0
    for (_, a), (_, b) in zip(poses, poses[1:]):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        total += math.sqrt(dx * dx + dy * dy + dz * dz)
    return total


def interp_position(poses, t):
    if not poses:
        return None
    times = [p[0] for p in poses]
    idx = bisect_left(times, t)
    if idx <= 0 or idx >= len(poses):
        return None
    t0, p0 = poses[idx - 1]
    t1, p1 = poses[idx]
    if t1 <= t0:
        return p0
    alpha = (t - t0) / (t1 - t0)
    return tuple((1.0 - alpha) * p0[i] + alpha * p1[i] for i in range(3))


def gt_length_for_est_span(gt_poses, est_poses):
    sampled = []
    for t, _ in est_poses:
        pos = interp_position(gt_poses, t)
        if pos is not None:
            sampled.append((t, pos))
    return path_length(sampled)


def run_evo(cmd, output_path):
    try:
        proc = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
    except (OSError, subprocess.TimeoutExpired) as exc:
        output_path.write_text(str(exc) + "\n", encoding="utf-8")
        return None
    output_path.write_text(proc.stdout, encoding="utf-8")
    if proc.returncode != 0:
        return None
    metrics = {}
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0] in {"rmse", "median", "mean", "min", "max", "std"}:
            try:
                metrics[parts[0]] = float(parts[1])
            except ValueError:
                pass
    return metrics


def parse_log(log_path):
    stats = {
        "tail_E_orth_frob": None,
        "mean_num_features": None,
        "active_landmarks_tail": None,
        "update_accept_ratio": None,
        "last10_update_accept_ratio": None,
        "ros_crash": False,
    }
    if not log_path.exists():
        return stats
    text = log_path.read_text(encoding="utf-8", errors="replace")
    if "process has died" in text or "Segmentation fault" in text or "terminate called" in text:
        stats["ros_crash"] = True

    e_values = [float(x) for x in re.findall(r"EOrth:([0-9.eE+-]+)", text)]
    if e_values:
        stats["tail_E_orth_frob"] = e_values[-1]

    feature_matches = re.findall(r"\[HNOFeature\].*?pts (\d+).*?db (\d+)", text)
    if feature_matches:
        pts = [int(m[0]) for m in feature_matches]
        db = [int(m[1]) for m in feature_matches]
        stats["mean_num_features"] = sum(pts) / float(len(pts))
        stats["active_landmarks_tail"] = db[-1]

    updater_matches = re.findall(r"\[HNOUpdater\] obs (\d+) accepted (\d+)", text)
    if updater_matches:
        obs = [int(m[0]) for m in updater_matches]
        accepted = [int(m[1]) for m in updater_matches]
        total_obs = sum(obs)
        stats["update_accept_ratio"] = sum(accepted) / float(total_obs) if total_obs else 0.0
        last_obs = sum(obs[-10:])
        stats["last10_update_accept_ratio"] = sum(accepted[-10:]) / float(last_obs) if last_obs else 0.0

    return stats


def is_usable(summary):
    return (
        summary.get("se3_ate_rmse") is not None and summary["se3_ate_rmse"] <= USABLE["se3_ate_rmse"] and
        summary.get("se3_ate_median") is not None and summary["se3_ate_median"] <= USABLE["se3_ate_median"] and
        summary.get("rpe1_trans_rmse") is not None and summary["rpe1_trans_rmse"] <= USABLE["rpe1_trans_rmse"] and
        summary.get("rpe1_rot_rmse_deg") is not None and summary["rpe1_rot_rmse_deg"] <= USABLE["rpe1_rot_rmse_deg"] and
        summary.get("path_length_ratio") is not None and 0.8 <= summary["path_length_ratio"] <= 1.2 and
        summary.get("duration_sec", 0.0) >= USABLE["duration_sec"]
    )


def is_strong(summary):
    return (
        summary.get("se3_ate_rmse") is not None and summary["se3_ate_rmse"] <= STRONG["se3_ate_rmse"] and
        summary.get("se3_ate_median") is not None and summary["se3_ate_median"] <= STRONG["se3_ate_median"] and
        summary.get("rpe1_trans_rmse") is not None and summary["rpe1_trans_rmse"] <= STRONG["rpe1_trans_rmse"] and
        summary.get("rpe1_rot_rmse_deg") is not None and summary["rpe1_rot_rmse_deg"] <= STRONG["rpe1_rot_rmse_deg"] and
        summary.get("path_length_ratio") is not None and 0.9 <= summary["path_length_ratio"] <= 1.1 and
        summary.get("duration_sec", 0.0) >= STRONG["duration_sec"]
    )


def classify(summary, build_failed):
    if build_failed:
        return "BUILD_FAILED"
    if summary.get("ros_crash"):
        return "ROS_CRASH"
    if summary.get("timed_out") and summary.get("duration_sec", 0.0) < USABLE["duration_sec"]:
        return "TIMEOUT"
    if summary.get("pose_count", 0) == 0:
        return "NO_ODOM"
    if summary.get("evo_failed"):
        return "EVO_FAILED"
    if summary.get("duration_sec", 0.0) < USABLE["duration_sec"]:
        return "SHORT_DURATION"
    if summary.get("tail_E_orth_frob") is not None and summary["tail_E_orth_frob"] >= 0.05:
        return "E_ORTH_COLLAPSE"
    if (
        summary.get("mean_num_features") is not None and summary["mean_num_features"] < 20
    ) or (
        summary.get("active_landmarks_tail") is not None and summary["active_landmarks_tail"] < 20
    ):
        return "FEATURE_STARVATION"
    if (
        summary.get("update_accept_ratio") is not None and summary["update_accept_ratio"] < 0.5
    ) or (
        summary.get("last10_update_accept_ratio") is not None and summary["last10_update_accept_ratio"] < 0.3
    ):
        return "LOW_UPDATE_ACCEPT_RATIO"
    if summary.get("se3_ate_rmse") is not None and summary["se3_ate_rmse"] > USABLE["se3_ate_rmse"]:
        return "ATE_DIVERGED"
    if summary.get("path_length_ratio") is not None and not (0.8 <= summary["path_length_ratio"] <= 1.2):
        return "PATH_LENGTH_BAD"
    if summary.get("rpe1_trans_rmse") is not None and summary["rpe1_trans_rmse"] > USABLE["rpe1_trans_rmse"]:
        return "RPE_TRANS_BAD"
    if summary.get("rpe1_rot_rmse_deg") is not None and summary["rpe1_rot_rmse_deg"] > USABLE["rpe1_rot_rmse_deg"]:
        return "RPE_ROT_BAD"
    return "SUCCESS"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-dir", required=True)
    parser.add_argument("--gt", required=True)
    parser.add_argument("--launch-exit", type=int, default=0)
    parser.add_argument("--timed-out", action="store_true")
    parser.add_argument("--build-failed", action="store_true")
    args = parser.parse_args()

    case_dir = Path(args.case_dir)
    logs_dir = case_dir / "logs"
    result_dir = case_dir / "evo"
    result_dir.mkdir(parents=True, exist_ok=True)

    candidate = read_json(case_dir / "params.json", {})
    collector = read_json(case_dir / "collector_status.json", {})
    log_stats = parse_log(logs_dir / "run.log")
    est_tum = case_dir / "odom.tum"
    gt_tum = Path(args.gt)

    est_poses = read_tum(est_tum)
    gt_poses = read_tum(gt_tum)
    est_len = path_length(est_poses)
    gt_len = gt_length_for_est_span(gt_poses, est_poses)
    ratio = est_len / gt_len if gt_len > 1e-9 else None

    ape = None
    rpe_trans = None
    rpe_rot = None
    if est_poses:
        ape = run_evo(["evo_ape", "tum", str(gt_tum), str(est_tum), "-a"], result_dir / "evo_ape.txt")
        rpe_trans = run_evo(
            ["evo_rpe", "tum", str(gt_tum), str(est_tum), "-a", "--delta", "1", "--delta_unit", "f", "--pose_relation", "trans_part"],
            result_dir / "evo_rpe_trans.txt")
        rpe_rot = run_evo(
            ["evo_rpe", "tum", str(gt_tum), str(est_tum), "-a", "--delta", "1", "--delta_unit", "f", "--pose_relation", "angle_deg"],
            result_dir / "evo_rpe_rot.txt")

    summary = {
        "case_id": candidate.get("case_id", case_dir.name),
        "stage": candidate.get("stage"),
        "status": "UNKNOWN",
        "failure_reason": None,
        "se3_ate_rmse": ape.get("rmse") if ape else None,
        "se3_ate_median": ape.get("median") if ape else None,
        "rpe1_trans_rmse": rpe_trans.get("rmse") if rpe_trans else None,
        "rpe1_rot_rmse_deg": rpe_rot.get("rmse") if rpe_rot else None,
        "path_length_ratio": ratio,
        "duration_sec": collector.get("duration_sec", 0.0),
        "pose_count": collector.get("pose_count", 0),
        "path_length_m": collector.get("path_length_m", est_len),
        "update_accept_ratio": log_stats.get("update_accept_ratio"),
        "last10_update_accept_ratio": log_stats.get("last10_update_accept_ratio"),
        "tail_E_orth_frob": log_stats.get("tail_E_orth_frob"),
        "mean_num_features": log_stats.get("mean_num_features", collector.get("mean_num_features")),
        "active_landmarks_tail": log_stats.get("active_landmarks_tail", collector.get("active_landmarks_tail")),
        "launch_exit": args.launch_exit,
        "timed_out": args.timed_out,
        "ros_crash": log_stats.get("ros_crash", False),
        "evo_failed": bool(est_poses and (ape is None or rpe_trans is None or rpe_rot is None)),
        "usable": False,
        "strong_baseline": False,
    }
    summary["failure_reason"] = classify(summary, args.build_failed)
    summary["usable"] = is_usable(summary)
    summary["strong_baseline"] = is_strong(summary)
    summary["status"] = "SUCCESS" if summary["failure_reason"] == "SUCCESS" else "FAILED"

    (case_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    lines = [
        f"# {summary['case_id']}",
        "",
        f"- status: {summary['status']}",
        f"- failure_reason: {summary['failure_reason']}",
        f"- usable: {summary['usable']}",
        f"- strong_baseline: {summary['strong_baseline']}",
        f"- se3_ate_rmse: {summary['se3_ate_rmse']}",
        f"- se3_ate_median: {summary['se3_ate_median']}",
        f"- rpe1_trans_rmse: {summary['rpe1_trans_rmse']}",
        f"- rpe1_rot_rmse_deg: {summary['rpe1_rot_rmse_deg']}",
        f"- path_length_ratio: {summary['path_length_ratio']}",
        f"- duration_sec: {summary['duration_sec']}",
    ]
    (case_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()

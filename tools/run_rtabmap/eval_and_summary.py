#!/usr/bin/python3
"""Run evo evaluation and write a compact RTAB-Map offline summary."""

import argparse
import json
import subprocess
from pathlib import Path


def run_cmd(cmd, log_path):
    with log_path.open("w", encoding="utf-8") as f:
        proc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, text=True, check=False)
    return proc.returncode


def read_export_report(log_dir):
    path = log_dir / "export_report.txt"
    if not path.exists():
        return []
    return path.read_text(encoding="utf-8").strip().splitlines()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--input-bag", required=True)
    parser.add_argument("--output-bag", required=True)
    parser.add_argument("--offline-dir", required=True)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    offline_dir = Path(args.offline_dir)
    log_dir = offline_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    context_path = run_dir / "run_context.json"
    context = json.loads(context_path.read_text(encoding="utf-8")) if context_path.exists() else {}

    gt = context.get("ground_truth_tum", "")
    gt_path = Path(gt)
    if gt and not gt_path.is_absolute():
        gt_path = (run_dir / gt_path).resolve()
        if not gt_path.exists():
            gt_path = (Path("/home/sharpa/hno_vio_clean/src/hno_vio") / gt).resolve()

    raw_odom = run_dir / context.get("odom_tum", "vio_results/odom_raw.txt")
    optimized_odom = offline_dir / "odom_optimized.txt"
    evo_raw_rc = None
    evo_opt_rc = None
    if gt_path.exists() and raw_odom.exists():
        evo_raw_rc = run_cmd(
            ["evo_ape", "tum", str(gt_path), str(raw_odom), "-a", "--save_results", str(offline_dir / "evo_raw.zip")],
            log_dir / "evo_raw.log",
        )
    if gt_path.exists() and optimized_odom.exists():
        evo_opt_rc = run_cmd(
            ["evo_ape", "tum", str(gt_path), str(optimized_odom), "-a", "--save_results", str(offline_dir / "evo_optimized.zip")],
            log_dir / "evo_optimized.log",
        )

    bag_info_rc = run_cmd(["ros2", "bag", "info", str(args.output_bag)], log_dir / "rtabmap_output_bag_info.txt")
    input_info_rc = run_cmd(["ros2", "bag", "info", str(args.input_bag)], log_dir / "rtabmap_input_bag_info.txt")

    lines = [
        "# RTAB-Map Offline Summary",
        "",
        f"- run_dir: `{run_dir}`",
        f"- input_bag: `{args.input_bag}`",
        f"- output_bag: `{args.output_bag}`",
        f"- rtabmap_db: `{offline_dir / 'rtabmap.db'}`",
        f"- raw_odom: `{raw_odom}`",
        f"- optimized_odom: `{optimized_odom}`",
        f"- ground_truth: `{gt_path if gt else 'n/a'}`",
        "",
        "## Export",
        "",
        *[f"- {line}" for line in read_export_report(log_dir)],
        "",
        "## Evaluation",
        "",
        f"- evo_raw_exit_code: `{evo_raw_rc if evo_raw_rc is not None else 'not_run'}`",
        f"- evo_optimized_exit_code: `{evo_opt_rc if evo_opt_rc is not None else 'not_run'}`",
        f"- input_bag_info_exit_code: `{input_info_rc}`",
        f"- output_bag_info_exit_code: `{bag_info_rc}`",
    ]
    (offline_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(offline_dir / "summary.md")


if __name__ == "__main__":
    main()

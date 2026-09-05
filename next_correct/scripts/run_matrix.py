#!/usr/bin/env python3
"""Run the four-condition V1_02 e-transient experiment matrix."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np


CONDITIONS = {
    "baseline": {
        "experiment_fix_e_hat": False,
        "experiment_force_sigma_r_zero": False,
        "update_enforce_structure": True,
    },
    "fixed_e": {
        "experiment_fix_e_hat": True,
        "experiment_force_sigma_r_zero": False,
        "update_enforce_structure": True,
    },
    "sigma_zero": {
        "experiment_fix_e_hat": False,
        "experiment_force_sigma_r_zero": True,
        "update_enforce_structure": True,
    },
    "no_projection": {
        "experiment_fix_e_hat": False,
        "experiment_force_sigma_r_zero": False,
        "update_enforce_structure": False,
    },
}

FORMAL_ORDER = (
    ("baseline", 1),
    ("fixed_e", 1),
    ("sigma_zero", 1),
    ("sigma_zero", 2),
    ("baseline", 2),
    ("fixed_e", 2),
    ("fixed_e", 3),
    ("sigma_zero", 3),
    ("baseline", 3),
    ("no_projection", 1),
    ("no_projection", 2),
    ("no_projection", 3),
)


def parse_args() -> argparse.Namespace:
    script = Path(__file__).resolve()
    default_package = script.parents[2]
    default_workspace = default_package.parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("validation", "formal"), required=True)
    parser.add_argument("--dataset", default="V1_02_medium")
    parser.add_argument(
        "--bag-path",
        type=Path,
        default=Path("/home/he/datasets/euroc/V1_02_medium_db"),
    )
    parser.add_argument("--package-root", type=Path, default=default_package)
    parser.add_argument("--workspace-root", type=Path, default=default_workspace)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=default_package / "next_correct" / "data",
    )
    parser.add_argument("--timeout-seconds", type=int, default=360)
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Skip run directories with a successful manifest instead of failing.",
    )
    return parser.parse_args()


def iso_now() -> str:
    return dt.datetime.now(dt.timezone.utc).astimezone().isoformat()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_output(package_root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=package_root,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.stdout.strip()


def successful_manifest(path: Path) -> bool:
    if not path.is_file():
        return False
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    return payload.get("status") in ("complete", "algorithm_terminal")


def launch_command(
    args: argparse.Namespace,
    condition_name: str,
    run_dir: Path,
    max_frames: int,
) -> list[str]:
    condition = CONDITIONS[condition_name]
    value = lambda flag: "true" if flag else "false"
    return [
        "ros2",
        "launch",
        "hno_vio",
        "hno_vio.launch.py",
        f"dataset:={args.dataset}",
        f"bag_path:={args.bag_path}",
        "bag_rate:=1.0",
        "use_gt_mapping:=false",
        "try_zupt:=false",
        "run_preprocess:=false",
        "rviz:=false",
        "play_bag:=true",
        "export_odom:=true",
        "essential_print:=false",
        "frontend_print:=false",
        "updater_print:=false",
        "ZUPT_print:=false",
        "pipeline_print:=false",
        f"update_enforce_structure:={value(condition['update_enforce_structure'])}",
        f"experiment_fix_e_hat:={value(condition['experiment_fix_e_hat'])}",
        "experiment_force_sigma_r_zero:="
        f"{value(condition['experiment_force_sigma_r_zero'])}",
        f"experiment_max_frames:={max_frames}",
        f"results_root:={run_dir}",
        f"odom_output_path:={run_dir / 'vio_results' / 'odom_raw.csv'}",
    ]


def terminate_process_group(process: subprocess.Popen[str]) -> None:
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def checksums(run_dir: Path) -> dict[str, str]:
    result = {}
    for path in sorted((run_dir / "vio_results").glob("*")):
        if path.is_file():
            result[str(path.relative_to(run_dir))] = sha256(path)
    return result


def inspect_outputs(
    run_dir: Path,
    condition_name: str,
    expected_rows: int | None,
    strict: bool,
) -> tuple[dict[str, bool | int | float], bool]:
    diagnostics_path = run_dir / "vio_results" / "e_diagnostics.csv"
    odom_path = run_dir / "vio_results" / "odom_raw.csv"
    if not diagnostics_path.is_file() or not odom_path.is_file():
        return {"files_present": False}, False
    table = np.genfromtxt(
        diagnostics_path,
        delimiter=",",
        names=True,
        dtype=float,
        encoding="utf-8",
    )
    table = np.atleast_1d(table)
    diag_rows = len(table)
    with odom_path.open("r", encoding="utf-8") as stream:
        odom_rows = max(0, sum(1 for _ in stream) - 1)

    def matrix(prefix: str) -> np.ndarray:
        columns = [
            table[f"{prefix}_{row}{col}"]
            for row in range(3)
            for col in range(3)
        ]
        return np.column_stack(columns).reshape((-1, 3, 3))

    raw = matrix("e_raw")
    projected = matrix("e_projected")
    committed = matrix("e_committed")
    identity = np.eye(3)
    projected_orth_error = np.linalg.norm(
        np.transpose(projected, (0, 2, 1)) @ projected - identity,
        axis=(1, 2),
    )
    projected_det_error = np.abs(np.linalg.det(projected) - 1.0)
    finite_columns = [
        "theta_e_raw_deg",
        "epsilon_orth",
        "epsilon_det",
        "visual_dE_fro",
        "sigma_r_raw_norm",
        "sigma_r_applied_norm",
    ]
    checks: dict[str, bool | int | float] = {
        "files_present": True,
        "diagnostics_rows": diag_rows,
        "odom_rows": odom_rows,
        "rows_match": diag_rows == odom_rows,
        "expected_rows_match": expected_rows is None or diag_rows == expected_rows,
        "core_values_finite": bool(all(
            np.isfinite(table[column]).all() for column in finite_columns
        )),
        "state_finite": bool(np.all(table["state_finite"] == 1.0)),
        "visual_dR_structural_zero": bool(
            np.max(np.abs(table["visual_dR_deg"])) <= 1e-8
        ),
        "projection_flag_matches": bool(np.all(
            table["structure_projection_enabled"] ==
            (0.0 if condition_name == "no_projection" else 1.0)
        )),
    }
    if condition_name == "no_projection":
        checks.update({
            "projection_not_applied": bool(
                np.max(np.abs(table["projection_correction"])) <= 1e-12
            ),
            "raw_equals_committed": bool(
                np.max(np.abs(raw - committed)) <= 1e-10
            ),
        })
    else:
        checks.update({
            "projected_is_so3": bool(
                np.max(projected_orth_error) <= 1e-8 and
                np.max(projected_det_error) <= 1e-8
            ),
        })
    if condition_name == "fixed_e":
        visual_applied = np.any(table["visual_update_applied"] > 0.5)
        checks.update({
            "committed_e_is_identity": bool(
                np.max(np.abs(committed - identity)) <= 1e-10
            ),
            "sigma_raw_is_zero": bool(
                np.max(np.abs(table["sigma_r_raw_norm"])) <= 1e-10
            ),
            "sigma_applied_is_zero": bool(
                np.max(np.abs(table["sigma_r_applied_norm"])) <= 1e-10
            ),
            "discarded_visual_dE_visible": bool(
                (not visual_applied) or np.max(table["visual_dE_fro"]) > 1e-12
            ),
        })
    if condition_name == "sigma_zero":
        checks.update({
            "sigma_applied_is_zero": bool(
                np.max(np.abs(table["sigma_r_applied_norm"])) <= 1e-10
            ),
            "sigma_raw_is_observable": bool(
                np.max(table["sigma_r_raw_max"]) > 1e-12
            ),
        })
    non_blocking = set() if strict else {
        "expected_rows_match",
        "core_values_finite",
        "state_finite",
    }
    passed = all(
        value for key, value in checks.items()
        if isinstance(value, bool) and key not in non_blocking
    )
    return checks, passed


def run_one(
    args: argparse.Namespace,
    condition_name: str,
    repeat: int,
    max_frames: int,
    binary_path: Path,
    source_commit: str,
    source_status: str,
) -> bool:
    suite = "validation" if args.mode == "validation" else args.dataset
    run_dir = args.output_root / suite / condition_name / f"run_{repeat}"
    manifest_path = run_dir / "manifest.json"
    if run_dir.exists():
        if args.resume and successful_manifest(manifest_path):
            print(f"[skip] {run_dir}")
            return True
        raise FileExistsError(
            f"run directory already exists: {run_dir}; use --resume or move it aside"
        )
    (run_dir / "vio_results").mkdir(parents=True)
    command = launch_command(args, condition_name, run_dir, max_frames)
    log_path = run_dir / "launch.log"
    started = iso_now()
    start_monotonic = time.monotonic()
    timed_out = False
    print(f"[run] {condition_name} repeat={repeat} max_frames={max_frames}")
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(
            command,
            cwd=args.package_root,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        try:
            exit_code = process.wait(timeout=args.timeout_seconds)
        except subprocess.TimeoutExpired:
            timed_out = True
            terminate_process_group(process)
            exit_code = 124
    ended = iso_now()
    elapsed = time.monotonic() - start_monotonic
    diagnostics_path = run_dir / "vio_results" / "e_diagnostics.csv"
    odom_path = run_dir / "vio_results" / "odom_raw.csv"
    row_count = 0
    if diagnostics_path.is_file():
        with diagnostics_path.open("r", encoding="utf-8") as stream:
            row_count = max(0, sum(1 for _ in stream) - 1)
    expected_rows = max_frames if max_frames > 0 else None
    output_checks, output_valid = inspect_outputs(
        run_dir, condition_name, expected_rows, args.mode == "validation"
    )
    successful_completion = (
        exit_code == 0
        and row_count > 0
        and output_valid
        and bool(output_checks.get("expected_rows_match", True))
        and bool(output_checks.get("state_finite", False))
    )
    recorded = row_count > 0 and output_valid and not timed_out
    if successful_completion:
        status = "complete"
    elif args.mode == "formal" and recorded:
        status = "algorithm_terminal"
    else:
        status = "failed"
    manifest = {
        "schema_version": 1,
        "status": status,
        "mode": args.mode,
        "dataset": args.dataset,
        "condition": condition_name,
        "repeat": repeat,
        "parameters": {
            **CONDITIONS[condition_name],
            "experiment_max_frames": max_frames,
            "bag_rate": 1.0,
            "try_zupt": False,
            "use_gt_mapping": False,
        },
        "bag_path": str(args.bag_path.resolve()),
        "command": command,
        "source_commit": source_commit,
        "source_status": source_status,
        "binary_path": str(binary_path),
        "binary_sha256": sha256(binary_path),
        "started_at": started,
        "ended_at": ended,
        "elapsed_seconds": elapsed,
        "exit_code": exit_code,
        "timed_out": timed_out,
        "diagnostics_rows": row_count,
        "output_checks": output_checks,
        "file_sha256": checksums(run_dir),
    }
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"[{status}] {run_dir} rows={row_count}")
    return successful_completion if args.mode == "validation" else recorded


def main() -> int:
    args = parse_args()
    args.package_root = args.package_root.resolve()
    args.workspace_root = args.workspace_root.resolve()
    args.output_root = args.output_root.resolve()
    args.bag_path = args.bag_path.resolve()
    if not args.bag_path.exists():
        raise FileNotFoundError(f"bag path does not exist: {args.bag_path}")
    binary_path = (
        args.workspace_root / "install" / "hno_vio" / "lib" / "hno_vio" /
        "run_hno_vio"
    )
    if not binary_path.is_file():
        raise FileNotFoundError(
            f"built binary not found: {binary_path}; build the package first"
        )
    source_commit = git_output(args.package_root, "rev-parse", "HEAD")
    source_status = git_output(args.package_root, "status", "--short")
    matrix = (
        tuple((condition, 1) for condition in CONDITIONS)
        if args.mode == "validation"
        else FORMAL_ORDER
    )
    failures = 0
    for condition_name, repeat in matrix:
        max_frames = 100 if args.mode == "validation" else (
            900 if condition_name == "no_projection" else 0
        )
        try:
            complete = run_one(
                args,
                condition_name,
                repeat,
                max_frames,
                binary_path,
                source_commit,
                source_status,
            )
        except (FileExistsError, OSError, subprocess.SubprocessError) as error:
            print(f"[error] {error}", file=sys.stderr)
            return 2
        failures += int(not complete)
        if args.mode == "validation" and not complete:
            print("validation stopped after first failed condition", file=sys.stderr)
            break
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/python3
"""Evaluate raw and RTAB-Map trajectories and generate EVO plots."""

import argparse
import json
import os
import shlex
import shutil
import subprocess
import tempfile
from pathlib import Path


def require_file(path, label):
    if not path.is_file():
        raise FileNotFoundError(f"missing {label}: {path}")


def require_command(name):
    if shutil.which(name) is None:
        raise RuntimeError(f"required command not found: {name}")


def evo_environment(cache_dir):
    env = os.environ.copy()
    conda_prefix = env.get("CONDA_PREFIX")
    if conda_prefix:
        conda_lib = str(Path(conda_prefix) / "lib")
        current = env.get("LD_LIBRARY_PATH", "")
        entries = [entry for entry in current.split(":") if entry and entry != conda_lib]
        env["LD_LIBRARY_PATH"] = ":".join([conda_lib, *entries])
    matplotlib_cache = cache_dir / "matplotlib"
    matplotlib_cache.mkdir()
    env["MPLCONFIGDIR"] = str(matplotlib_cache)
    return env


def run_command(command, env):
    print(f"+ {shlex.join(command)}", flush=True)
    subprocess.run(command, env=env, check=True)


def run_command_with_tee(command, output_path, env):
    print(f"+ {shlex.join(command)}", flush=True)
    with output_path.open("w", encoding="utf-8") as output:
        process = subprocess.Popen(
            command,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="", flush=True)
            output.write(line)
        return_code = process.wait()
    if return_code != 0:
        raise subprocess.CalledProcessError(return_code, command)


def resolve_run_file(run_dir, value):
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (run_dir / path).resolve()


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate HNO-VIO raw and RTAB-Map optimized trajectories."
    )
    parser.add_argument("run_dir", help="Path to one results/run_YYYYmmddTHHMMSS directory")
    args = parser.parse_args()

    run_dir = Path(args.run_dir).expanduser().resolve()
    if not run_dir.is_dir():
        raise NotADirectoryError(f"run directory not found: {run_dir}")

    context_path = run_dir / "run_context.json"
    require_file(context_path, "run_context.json")
    context = json.loads(context_path.read_text(encoding="utf-8"))

    ground_truth_value = context.get("ground_truth_tum")
    if not ground_truth_value:
        raise ValueError(f"ground_truth_tum is missing in {context_path}")

    ground_truth = resolve_run_file(run_dir, ground_truth_value)
    raw_odom = resolve_run_file(run_dir, context.get("odom_tum", "vio_results/odom_raw.txt"))
    optimized_odom = run_dir / "offline_results" / "odom_optimized.txt"
    require_file(ground_truth, "ground truth trajectory")
    require_file(raw_odom, "raw odometry trajectory")
    require_file(optimized_odom, "optimized odometry trajectory")

    for command in ("evo_ape", "evo_res", "evo_traj"):
        require_command(command)

    evo_results = run_dir / "evo_results"
    if evo_results.exists():
        shutil.rmtree(evo_results)
    evo_results.mkdir(parents=True)

    ape_raw = evo_results / "ape_raw.zip"
    ape_optimized = evo_results / "ape_optimized.zip"
    with tempfile.TemporaryDirectory(prefix="hno_evo_") as temporary_dir:
        temporary_path = Path(temporary_dir)
        evo_config = temporary_path / "evo_config.json"
        evo_config.write_text(
            json.dumps({"plot_backend": "Agg", "plot_split": False}),
            encoding="utf-8",
        )
        env = evo_environment(temporary_path)

        run_command(
            [
                "evo_ape",
                "tum",
                str(ground_truth),
                str(raw_odom),
                "-a",
                "-r",
                "trans_part",
                "--save_results",
                str(ape_raw),
            ],
            env,
        )
        run_command(
            [
                "evo_ape",
                "tum",
                str(ground_truth),
                str(optimized_odom),
                "-a",
                "-r",
                "trans_part",
                "--save_results",
                str(ape_optimized),
            ],
            env,
        )
        run_command_with_tee(
            [
                "evo_res",
                str(ape_raw),
                str(ape_optimized),
                "--save_plot",
                str(evo_results / "ate_plot.pdf"),
                "--config",
                str(evo_config),
            ],
            evo_results / "ate_stats.txt",
            env,
        )
        run_command(
            [
                "evo_traj",
                "tum",
                str(raw_odom),
                str(optimized_odom),
                "--ref",
                str(ground_truth),
                "-a",
                "--save_plot",
                str(evo_results / "traj.png"),
                "--config",
                str(evo_config),
            ],
            env,
        )
        for plot_name in (
            "traj_trajectories.png",
            "traj_xyz.png",
            "traj_rpy.png",
            "traj_speeds.png",
        ):
            require_file(evo_results / plot_name, "EVO trajectory plot")

    print(f"EVO results: {evo_results}")


if __name__ == "__main__":
    main()

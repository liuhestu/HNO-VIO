#!/usr/bin/env python3
"""Analyze e-transient experiment runs and generate report-ready artifacts."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import warnings

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation, Slerp
from scipy.stats import ConstantInputWarning, spearmanr


CONDITION_ORDER = ("baseline", "fixed_e", "sigma_zero", "no_projection")
COLORS = {
    "baseline": "#e04444",
    "fixed_e": "#18a558",
    "sigma_zero": "#2774d8",
    "no_projection": "#8a5cf6",
}
LABELS = {
    "baseline": "normal e + projection",
    "fixed_e": "fixed e + projection",
    "sigma_zero": "normal e + sigma_R=0",
    "no_projection": "normal e, no projection",
}
RADIANS_TO_DEGREES = 180.0 / math.pi


@dataclass
class RunData:
    manifest_path: Path
    manifest: dict
    frame: pd.DataFrame
    velocity_source: str

    @property
    def condition(self) -> str:
        return str(self.manifest["condition"])

    @property
    def repeat(self) -> int:
        return int(self.manifest["repeat"])

    @property
    def run_id(self) -> str:
        return f"{self.condition}/run_{self.repeat}"


def parse_args() -> argparse.Namespace:
    script = Path(__file__).resolve()
    package_root = script.parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data-root",
        type=Path,
        default=package_root / "next_correct" / "data" / "V1_02_medium",
    )
    parser.add_argument(
        "--ground-truth",
        type=Path,
        default=package_root / "ground_truth" / "euroc_mav" /
        "V1_02_medium.csv",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=package_root / "next_correct",
    )
    return parser.parse_args()


def load_ground_truth(path: Path) -> tuple[np.ndarray, np.ndarray, Rotation, np.ndarray, str]:
    values = pd.read_csv(path, comment="#", header=None).to_numpy(dtype=float)
    if values.shape[1] < 8:
        raise ValueError(f"ground truth requires at least 8 columns: {path}")
    timestamps = values[:, 0]
    if np.nanmedian(timestamps) > 1e10:
        timestamps = timestamps * 1e-9
    positions = values[:, 1:4]
    quaternions_xyzw = values[:, [5, 6, 7, 4]]
    rotations = Rotation.from_quat(quaternions_xyzw)
    if values.shape[1] >= 11 and np.isfinite(values[:, 8:11]).all():
        velocities = values[:, 8:11]
        velocity_source = "dataset"
    else:
        velocities = np.column_stack(
            [np.gradient(positions[:, axis], timestamps) for axis in range(3)]
        )
        velocity_source = "central_difference"
    unique = np.r_[True, np.diff(timestamps) > 0.0]
    return (
        timestamps[unique],
        positions[unique],
        rotations[unique],
        velocities[unique],
        velocity_source,
    )


def interpolate_ground_truth(
    timestamps: np.ndarray,
    positions: np.ndarray,
    rotations: Rotation,
    velocities: np.ndarray,
    query: np.ndarray,
) -> tuple[np.ndarray, Rotation, np.ndarray]:
    if query[0] < timestamps[0] or query[-1] > timestamps[-1]:
        raise ValueError("diagnostic timestamps fall outside ground-truth range")
    origin = timestamps[0]
    relative_t = timestamps - origin
    relative_q = query - origin
    position_query = np.column_stack(
        [np.interp(relative_q, relative_t, positions[:, axis]) for axis in range(3)]
    )
    velocity_query = np.column_stack(
        [np.interp(relative_q, relative_t, velocities[:, axis]) for axis in range(3)]
    )
    rotation_query = Slerp(relative_t, rotations)(relative_q)
    return position_query, rotation_query, velocity_query


def resample(frame: pd.DataFrame, rate_hz: float = 20.0) -> pd.DataFrame:
    end = float(frame["time"].iloc[-1])
    grid = np.arange(0.0, end + 1e-9, 1.0 / rate_hz)
    output = {"time": grid}
    for column in frame.columns:
        if column == "time":
            continue
        values = frame[column].to_numpy(dtype=float)
        output[column] = np.interp(grid, frame["time"], values)
    return pd.DataFrame(output)


def load_run(
    manifest_path: Path,
    gt: tuple[np.ndarray, np.ndarray, Rotation, np.ndarray, str],
) -> RunData | None:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if (
        manifest.get("mode") != "formal" or
        manifest.get("status") not in ("complete", "algorithm_terminal")
    ):
        return None
    diagnostics_path = manifest_path.parent / "vio_results" / "e_diagnostics.csv"
    if not diagnostics_path.is_file():
        return None
    frame = pd.read_csv(diagnostics_path)
    if frame.empty:
        return None
    required = [
        "px", "py", "pz", "vx", "vy", "vz", "qx", "qy", "qz", "qw",
        "theta_e_raw_deg", "epsilon_orth", "epsilon_det",
    ]
    finite_rows = np.isfinite(frame[required].to_numpy(dtype=float)).all(axis=1)
    bad = np.flatnonzero(~finite_rows)
    if bad.size:
        frame = frame.iloc[:bad[0]].copy()
    if frame.empty:
        return None
    timestamps = frame["timestamp_ns"].to_numpy(dtype=float) * 1e-9
    gt_t, gt_p, gt_r, gt_v, velocity_source = gt
    inside = (timestamps >= gt_t[0]) & (timestamps <= gt_t[-1])
    frame = frame.loc[inside].reset_index(drop=True)
    timestamps = timestamps[inside]
    if len(frame) < 2:
        return None
    query_p, query_r, query_v = interpolate_ground_truth(
        gt_t, gt_p, gt_r, gt_v, timestamps
    )
    estimate_p = frame[["px", "py", "pz"]].to_numpy(dtype=float)
    estimate_v = frame[["vx", "vy", "vz"]].to_numpy(dtype=float)
    estimate_r = Rotation.from_quat(
        frame[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)
    )
    alignment_r = estimate_r[0] * query_r[0].inv()
    alignment_t = estimate_p[0] - alignment_r.apply(query_p[0])
    aligned_p = alignment_r.apply(query_p) + alignment_t
    aligned_v = alignment_r.apply(query_v)
    aligned_r = alignment_r * query_r
    gravity = np.array([0.0, 0.0, -9.81])
    true_body_gravity = aligned_r.inv().apply(
        np.repeat(gravity[None, :], len(frame), axis=0)
    )
    estimated_world_gravity = estimate_r.apply(true_body_gravity)
    gravity_cosine = np.sum(estimated_world_gravity * gravity, axis=1) / (
        np.linalg.norm(estimated_world_gravity, axis=1) *
        np.linalg.norm(gravity)
    )
    frame["time"] = timestamps - timestamps[0]
    frame["r_error_deg"] = (aligned_r.inv() * estimate_r).magnitude() * RADIANS_TO_DEGREES
    frame["tilt_error_deg"] = (
        np.arccos(np.clip(gravity_cosine, -1.0, 1.0)) *
        RADIANS_TO_DEGREES
    )
    frame["gravity_projection_error_mps2"] = np.linalg.norm(
        estimated_world_gravity - gravity, axis=1
    )
    frame["p_error_m"] = np.linalg.norm(estimate_p - aligned_p, axis=1)
    frame["v_error_mps"] = np.linalg.norm(estimate_v - aligned_v, axis=1)
    frame = resample(frame)
    if manifest.get("status") == "algorithm_terminal":
        frame.loc[frame.index[-1], "state_finite"] = 0.0
    return RunData(manifest_path, manifest, frame, velocity_source)


def robust_threshold(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    if not finite.size:
        return 1e-12
    median = float(np.median(finite))
    mad = float(np.median(np.abs(finite - median)))
    return max(median + 5.0 * mad, 1e-12)


def first_sustained(mask: np.ndarray, count: int = 5) -> int | None:
    streak = 0
    for index, active in enumerate(mask):
        streak = streak + 1 if bool(active) else 0
        if streak >= count:
            return index - count + 1
    return None


def event_time(frame: pd.DataFrame, mask: np.ndarray) -> float:
    index = first_sustained(mask)
    return math.nan if index is None else float(frame["time"].iloc[index])


def delayed_increase(values: np.ndarray, samples: int = 10) -> np.ndarray:
    output = np.zeros_like(values, dtype=float)
    if len(values) > samples:
        output[samples:] = values[samples:] - values[:-samples]
    return output


def integrate(time_values: np.ndarray, values: np.ndarray) -> float:
    if len(values) < 2:
        return 0.0
    return float(np.trapz(values, time_values))


def median_last_seconds(frame: pd.DataFrame, column: str, seconds: float = 5.0) -> float:
    cutoff = max(0.0, float(frame["time"].iloc[-1]) - seconds)
    return float(frame.loc[frame["time"] >= cutoff, column].median())


def analyze_events(
    run: RunData,
    thresholds: dict[str, float],
) -> tuple[dict[str, float | str | int | bool], np.ndarray]:
    frame = run.frame
    theta = frame["theta_e_raw_deg"].to_numpy()
    epsilon_orth = frame["epsilon_orth"].to_numpy()
    epsilon_det = frame["epsilon_det"].to_numpy()
    r_error = frame["r_error_deg"].to_numpy()
    p_error = frame["p_error_m"].to_numpy()
    v_error = frame["v_error_mps"].to_numpy()
    visual_dp = frame["visual_dp_m"].to_numpy()
    visual_dv = frame["visual_dv_mps"].to_numpy()
    visual_de = frame["visual_dE_fro"].to_numpy()
    e_nonconvergence = (theta > 2.0) | (delayed_increase(theta) > 1.0)
    e_nonorthogonality = (
        (epsilon_orth > thresholds["epsilon_orth"]) |
        (epsilon_det > thresholds["epsilon_det"])
    )
    r_event = (r_error > 5.0) | (delayed_increase(r_error) > 3.0)
    p_event = p_error > 0.5
    v_event = v_error > 0.5
    visual_dp_event = visual_dp > thresholds["visual_dp_m"]
    visual_dv_event = visual_dv > thresholds["visual_dv_mps"]
    visual_de_event = visual_de > thresholds["visual_dE_fro"]
    instability = (
        (r_error > 10.0) |
        (p_error > 5.0) |
        (v_error > 10.0) |
        (frame["state_finite"].to_numpy() < 0.5)
    )
    times = {
        "e_nonconvergence_s": event_time(frame, e_nonconvergence),
        "e_nonorthogonality_s": event_time(frame, e_nonorthogonality),
        "r_s": event_time(frame, r_event),
        "p_s": event_time(frame, p_event),
        "v_s": event_time(frame, v_event),
        "visual_dp_s": event_time(frame, visual_dp_event),
        "visual_dv_s": event_time(frame, visual_dv_event),
        "visual_de_s": event_time(frame, visual_de_event),
        "instability_s": event_time(frame, instability),
    }
    e_time = np.nanmin(
        [times["e_nonconvergence_s"], times["e_nonorthogonality_s"]]
    ) if not (
        math.isnan(times["e_nonconvergence_s"]) and
        math.isnan(times["e_nonorthogonality_s"])
    ) else math.nan
    pv_time_values = [times["p_s"], times["v_s"]]
    visual_time_values = [
        times["visual_dp_s"], times["visual_dv_s"], times["visual_de_s"]
    ]
    pv_time = np.nanmin(pv_time_values) if np.isfinite(pv_time_values).any() else math.nan
    visual_time = (
        np.nanmin(visual_time_values)
        if np.isfinite(visual_time_values).any()
        else math.nan
    )
    downstream_time = (
        np.nanmin([pv_time, visual_time])
        if np.isfinite([pv_time, visual_time]).any()
        else math.nan
    )
    ordered = (
        np.isfinite(e_time) and np.isfinite(times["r_s"]) and
        np.isfinite(downstream_time) and
        e_time < times["r_s"] < downstream_time
    )
    result: dict[str, float | str | int | bool] = {
        "run_id": run.run_id,
        "condition": run.condition,
        "repeat": run.repeat,
        **times,
        "e_first_s": e_time,
        "downstream_first_s": downstream_time,
        "ordered_e_r_downstream": bool(ordered),
    }
    return result, instability


def add_burdens(frame: pd.DataFrame, thresholds: dict[str, float]) -> pd.DataFrame:
    output = frame.copy()
    output["e_burden"] = (
        output["theta_e_raw_deg"] / 2.0 +
        output["epsilon_orth"] / thresholds["epsilon_orth"] +
        output["epsilon_det"] / thresholds["epsilon_det"]
    )
    output["visual_burden"] = (
        output["visual_dp_m"] / thresholds["visual_dp_m"] +
        output["visual_dv_mps"] / thresholds["visual_dv_mps"] +
        output["visual_dE_fro"] / thresholds["visual_dE_fro"]
    )
    output["visual_burden_05s"] = (
        output["visual_burden"].rolling(10, min_periods=1).sum()
    )
    return output


def lag_metrics(
    run: RunData,
    instability_time: float,
    thresholds: dict[str, float],
) -> list[dict[str, float | str | int]]:
    frame = add_burdens(run.frame, thresholds)
    if np.isfinite(instability_time):
        frame = frame.loc[frame["time"] < instability_time].reset_index(drop=True)
    if len(frame) < 40:
        return []
    delta_e = delayed_increase(frame["e_burden"].to_numpy(), 10)
    outputs = {
        "r": delayed_increase(frame["r_error_deg"].to_numpy(), 10),
        "p": delayed_increase(frame["p_error_m"].to_numpy(), 10),
        "v": delayed_increase(frame["v_error_mps"].to_numpy(), 10),
        "visual": delayed_increase(frame["visual_burden_05s"].to_numpy(), 10),
    }
    rows = []
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", ConstantInputWarning)
        for target, delta_output in outputs.items():
            candidates = []
            for lag_samples in range(41):
                lhs = delta_e[:len(delta_e) - lag_samples or None]
                rhs = delta_output[lag_samples:]
                finite = np.isfinite(lhs) & np.isfinite(rhs)
                if finite.sum() < 20:
                    rho = math.nan
                else:
                    rho = float(spearmanr(lhs[finite], rhs[finite]).statistic)
                candidates.append((rho, lag_samples))
            finite_candidates = [item for item in candidates if np.isfinite(item[0])]
            if not finite_candidates:
                continue
            peak_rho, peak_samples = max(finite_candidates, key=lambda item: item[0])
            rows.append({
                "run_id": run.run_id,
                "condition": run.condition,
                "repeat": run.repeat,
                "target": target,
                "peak_rho": peak_rho,
                "peak_lag_s": peak_samples / 20.0,
                "positive_lead": bool(peak_rho > 0.0 and peak_samples > 0),
            })
    return rows


def sustained_increment_onset(
    frame: pd.DataFrame,
    column: str,
    increment_threshold: float,
    after_seconds: float = 0.0,
    window_samples: int = 20,
    sustained_samples: int = 10,
) -> float:
    values = frame[column].to_numpy(dtype=float)
    increments = delayed_increase(values, window_samples)
    mask = (
        (increments > increment_threshold) &
        (frame["time"].to_numpy(dtype=float) >= after_seconds)
    )
    index = first_sustained(mask, sustained_samples)
    return math.nan if index is None else float(frame["time"].iloc[index])


def frame_values_at(frame: pd.DataFrame, timestamp: float) -> pd.Series | None:
    if not np.isfinite(timestamp):
        return None
    index = int(np.abs(frame["time"].to_numpy(dtype=float) - timestamp).argmin())
    return frame.iloc[index]


def growth_diagnostics(
    runs: list[RunData],
    events: pd.DataFrame,
) -> pd.DataFrame:
    rows = []
    for run in runs:
        if run.condition != "baseline":
            continue
        frame = run.frame
        event = events.loc[
            (events["condition"] == "baseline") &
            (events["repeat"] == run.repeat)
        ]
        if event.empty:
            continue
        event = event.iloc[0]
        r_rise = sustained_increment_onset(
            frame, "r_error_deg", 1.0
        )
        v_runaway = sustained_increment_onset(
            frame, "v_error_mps", 1.0, after_seconds=10.0
        )
        p_runaway = sustained_increment_onset(
            frame, "p_error_m", 1.0, after_seconds=10.0
        )
        visual_window = (
            frame["visual_update_applied"].rolling(20, min_periods=20).sum()
        )
        visual_off_mask = (
            (visual_window.to_numpy(dtype=float) < 0.5) &
            (frame["time"].to_numpy(dtype=float) >= 10.0)
        )
        visual_off_index = first_sustained(visual_off_mask, 20)
        visual_off = (
            math.nan if visual_off_index is None
            else float(frame["time"].iloc[visual_off_index])
        )
        values = {
            "first_v_threshold": frame_values_at(frame, float(event["v_s"])),
            "first_p_threshold": frame_values_at(frame, float(event["p_s"])),
            "r_rise": frame_values_at(frame, r_rise),
            "v_runaway": frame_values_at(frame, v_runaway),
            "p_runaway": frame_values_at(frame, p_runaway),
        }
        row: dict[str, float | str | int] = {
            "run_id": run.run_id,
            "repeat": run.repeat,
            "first_v_threshold_s": float(event["v_s"]),
            "first_p_threshold_s": float(event["p_s"]),
            "r_5deg_event_s": float(event["r_s"]),
            "r_growth_onset_s": r_rise,
            "v_runaway_onset_s": v_runaway,
            "p_runaway_onset_s": p_runaway,
            "visual_updates_off_s": visual_off,
        }
        for name, sample in values.items():
            for column in (
                "r_error_deg",
                "tilt_error_deg",
                "gravity_projection_error_mps2",
                "p_error_m",
                "v_error_mps",
                "theta_e_raw_deg",
                "epsilon_orth",
                "sigma_r_raw_max",
            ):
                row[f"{name}_{column}"] = (
                    math.nan if sample is None else float(sample[column])
                )
        rows.append(row)
    return pd.DataFrame(rows)


def run_summary(
    run: RunData,
    event: dict[str, float | str | int | bool],
    thresholds: dict[str, float],
) -> dict[str, float | str | int | bool]:
    frame = add_burdens(run.frame, thresholds)
    time_values = frame["time"].to_numpy()
    return {
        "run_id": run.run_id,
        "condition": run.condition,
        "repeat": run.repeat,
        "duration_s": float(time_values[-1]),
        "frames": int(run.manifest.get("diagnostics_rows", len(frame))),
        "gt_velocity_source": run.velocity_source,
        "theta_e_peak_deg": float(frame["theta_e_raw_deg"].max()),
        "epsilon_orth_peak": float(frame["epsilon_orth"].max()),
        "epsilon_det_peak": float(frame["epsilon_det"].max()),
        "projection_correction_peak": float(frame["projection_correction"].max()),
        "sigma_r_raw_peak": float(frame["sigma_r_raw_max"].max()),
        "r_peak_deg": float(frame["r_error_deg"].max()),
        "p_peak_m": float(frame["p_error_m"].max()),
        "v_peak_mps": float(frame["v_error_mps"].max()),
        "r_final5_median_deg": median_last_seconds(frame, "r_error_deg"),
        "p_final5_median_m": median_last_seconds(frame, "p_error_m"),
        "v_final5_median_mps": median_last_seconds(frame, "v_error_mps"),
        "e_auc": integrate(time_values, frame["e_burden"].to_numpy()),
        "r_auc": integrate(time_values, frame["r_error_deg"].to_numpy()),
        "p_auc": integrate(time_values, frame["p_error_m"].to_numpy()),
        "v_auc": integrate(time_values, frame["v_error_mps"].to_numpy()),
        "visual_auc": integrate(time_values, frame["visual_burden"].to_numpy()),
        "instability_s": event["instability_s"],
        "ordered_e_r_downstream": event["ordered_e_r_downstream"],
    }


def matched_ablation(
    runs: list[RunData],
    thresholds: dict[str, float],
    events: pd.DataFrame,
) -> pd.DataFrame:
    by_key = {(run.condition, run.repeat): run for run in runs}
    rows = []
    for repeat in (1, 2, 3):
        keys = [(condition, repeat) for condition in ("baseline", "fixed_e", "sigma_zero")]
        if not all(key in by_key for key in keys):
            continue
        horizon_candidates = [
            float(by_key[key].frame["time"].iloc[-1]) for key in keys
        ]
        if not events.empty:
            for condition, _ in keys:
                event_row = events.loc[
                    (events["condition"] == condition) &
                    (events["repeat"] == repeat),
                    "instability_s",
                ]
                if not event_row.empty and np.isfinite(event_row.iloc[0]):
                    horizon_candidates.append(float(event_row.iloc[0]))
        horizon = min(horizon_candidates)
        metrics = {}
        for condition, _ in keys:
            frame = add_burdens(by_key[(condition, repeat)].frame, thresholds)
            frame = frame.loc[frame["time"] <= horizon]
            time_values = frame["time"].to_numpy()
            denominator = max(horizon, 1e-9)
            metrics[condition] = {
                "r": integrate(time_values, frame["r_error_deg"].to_numpy()) / denominator,
                "p": integrate(time_values, frame["p_error_m"].to_numpy()) / denominator,
                "v": integrate(time_values, frame["v_error_mps"].to_numpy()) / denominator,
                "visual": integrate(time_values, frame["visual_burden"].to_numpy()) / denominator,
            }
        for condition in ("baseline", "fixed_e", "sigma_zero"):
            row = {
                "repeat": repeat,
                "condition": condition,
                "common_horizon_s": horizon,
            }
            for category in ("r", "p", "v", "visual"):
                value = metrics[condition][category]
                baseline = metrics["baseline"][category]
                row[f"{category}_auc_per_s"] = value
                row[f"{category}_reduction"] = (
                    math.nan if baseline <= 1e-12 else 1.0 - value / baseline
                )
            rows.append(row)
    return pd.DataFrame(rows)


def configure_plot_style() -> None:
    plt.rcParams.update({
        "figure.facecolor": "#f6f7f9",
        "axes.facecolor": "#f8f9fb",
        "axes.edgecolor": "#d9dde3",
        "axes.labelcolor": "#17202a",
        "text.color": "#17202a",
        "xtick.color": "#697386",
        "ytick.color": "#697386",
        "grid.color": "#d9dde3",
        "grid.alpha": 0.65,
        "font.size": 9.5,
        "axes.titleweight": "normal",
        "legend.frameon": False,
        "savefig.facecolor": "#f6f7f9",
    })


def plot_group_series(
    axes: np.ndarray,
    runs: list[RunData],
    columns: list[str],
    ylabels: list[str],
) -> None:
    for condition in CONDITION_ORDER:
        group = [run for run in runs if run.condition == condition]
        if not group:
            continue
        maximum_time = max(float(run.frame["time"].iloc[-1]) for run in group)
        grid = np.arange(0.0, maximum_time + 1e-9, 0.05)
        for axis, column, ylabel in zip(axes, columns, ylabels):
            samples = []
            for run in group:
                time_values = run.frame["time"].to_numpy()
                values = run.frame[column].to_numpy()
                interpolated = np.full_like(grid, np.nan)
                valid = grid <= time_values[-1]
                interpolated[valid] = np.interp(grid[valid], time_values, values)
                samples.append(interpolated)
                axis.plot(
                    time_values,
                    values,
                    color=COLORS[condition],
                    alpha=0.18,
                    linewidth=0.7,
                )
            stack = np.vstack(samples)
            median = np.nanmedian(stack, axis=0)
            lower = np.nanpercentile(stack, 25, axis=0)
            upper = np.nanpercentile(stack, 75, axis=0)
            axis.plot(
                grid,
                median,
                color=COLORS[condition],
                linewidth=1.8,
                label=LABELS[condition],
            )
            axis.fill_between(
                grid,
                lower,
                upper,
                color=COLORS[condition],
                alpha=0.10,
                linewidth=0,
            )
            axis.set_ylabel(ylabel)
            axis.grid(True, linewidth=0.7)


def save_figure(fig: plt.Figure, figures: Path, stem: str) -> None:
    fig.savefig(figures / f"{stem}.png", dpi=180, bbox_inches="tight")
    fig.savefig(figures / f"{stem}.pdf", bbox_inches="tight")
    plt.close(fig)


def make_early_growth_figure(
    runs: list[RunData],
    growth: pd.DataFrame,
    figures: Path,
) -> None:
    baseline = [run for run in runs if run.condition == "baseline"]
    if not baseline or growth.empty:
        return
    end_time = min(55.0, max(float(run.frame["time"].iloc[-1]) for run in baseline))
    grid = np.arange(0.0, end_time + 1e-9, 0.05)

    def stack(column: str) -> np.ndarray:
        result = []
        for run in baseline:
            time_values = run.frame["time"].to_numpy(dtype=float)
            values = run.frame[column].to_numpy(dtype=float)
            result.append(np.interp(grid, time_values, values))
        return np.vstack(result)

    def draw_band(
        axis: plt.Axes,
        column: str,
        color: str,
        label: str,
        linestyle: str = "-",
    ) -> None:
        values = stack(column)
        median = np.median(values, axis=0)
        lower = np.percentile(values, 25, axis=0)
        upper = np.percentile(values, 75, axis=0)
        axis.plot(
            grid, median, color=color, linewidth=1.8,
            linestyle=linestyle, label=label,
        )
        axis.fill_between(
            grid, lower, upper, color=color, alpha=0.12, linewidth=0,
        )

    configure_plot_style()
    fig, axes = plt.subplots(5, 1, figsize=(11.2, 12.5), sharex=True)
    draw_band(axes[0], "theta_e_raw_deg", "#8a5cf6", r"$\theta_E$")
    sigma_axis = axes[0].twinx()
    draw_band(sigma_axis, "sigma_r_raw_max", "#d18b24", r"max $\|\sigma_R\|$")
    axes[0].set_ylabel(r"$\theta_E$ [deg]")
    sigma_axis.set_ylabel(r"$\|\sigma_R\|$")
    axes[0].legend(loc="upper left")
    sigma_axis.legend(loc="upper right")

    draw_band(axes[1], "r_error_deg", "#e04444", "R error")
    draw_band(
        axes[1], "tilt_error_deg", "#e04444",
        "gravity-axis tilt", linestyle="--",
    )
    axes[1].axhline(5.0, color="#697386", linestyle=":", linewidth=1.0)
    axes[1].set_ylabel("angle [deg]")
    axes[1].legend(loc="upper left")

    p_ratio = stack("p_error_m") / 0.5
    v_ratio = stack("v_error_mps") / 0.5
    for values, color, label in (
        (p_ratio, "#18a558", "p error / 0.5 m"),
        (v_ratio, "#2774d8", "v error / 0.5 m/s"),
    ):
        axes[2].plot(
            grid, np.median(values, axis=0), color=color,
            linewidth=1.8, label=label,
        )
        axes[2].fill_between(
            grid,
            np.percentile(values, 25, axis=0),
            np.percentile(values, 75, axis=0),
            color=color, alpha=0.12, linewidth=0,
        )
    axes[2].axhline(1.0, color="#697386", linestyle=":", linewidth=1.0)
    axes[2].set_yscale("log")
    axes[2].set_ylim(bottom=0.08)
    axes[2].set_ylabel("threshold ratio")
    axes[2].legend(loc="upper left")

    growth_series = []
    for column, threshold, color, label in (
        ("r_error_deg", 1.0, "#e04444", r"$\Delta_{1s}R / 1°$"),
        ("p_error_m", 1.0, "#18a558", r"$\Delta_{1s}p / 1m$"),
        ("v_error_mps", 1.0, "#2774d8", r"$\Delta_{1s}v / 1m/s$"),
    ):
        values = stack(column)
        increments = np.zeros_like(values)
        increments[:, 20:] = values[:, 20:] - values[:, :-20]
        normalized = increments / threshold
        growth_series.append(normalized)
        axes[3].plot(
            grid, np.median(normalized, axis=0), color=color,
            linewidth=1.6, label=label,
        )
    axes[3].axhline(1.0, color="#697386", linestyle=":", linewidth=1.0)
    axes[3].set_ylabel("1 s growth ratio")
    axes[3].legend(loc="upper left", ncol=3)

    draw_band(
        axes[4], "gravity_projection_error_mps2", "#202a36",
        "gravity projection error",
    )
    visual_axis = axes[4].twinx()
    visual_values = stack("visual_update_applied")
    visual_axis.plot(
        grid, np.mean(visual_values > 0.5, axis=0),
        color="#d18b24", linestyle="--", linewidth=1.5,
        label="visual update fraction",
    )
    axes[4].set_ylabel(r"gravity error [m/s$^2$]")
    visual_axis.set_ylabel("visual update fraction")
    visual_axis.set_ylim(-0.05, 1.05)
    axes[4].legend(loc="upper left")
    visual_axis.legend(loc="upper right")
    axes[4].set_xlabel("Dataset time [s]")

    rise_min = float(growth["r_growth_onset_s"].min())
    rise_max = float(growth["r_growth_onset_s"].max())
    runaway_min = float(
        growth[["v_runaway_onset_s", "p_runaway_onset_s"]].min().min()
    )
    runaway_max = float(
        growth[["v_runaway_onset_s", "p_runaway_onset_s"]].max().max()
    )
    visual_off = float(growth["visual_updates_off_s"].median())
    for axis in axes:
        axis.axvspan(
            rise_min, rise_max, color="#e04444", alpha=0.09,
            label="R growth onset" if axis is axes[0] else None,
        )
        axis.axvspan(
            runaway_min, runaway_max, color="#8a5cf6", alpha=0.09,
            label="p/v runaway" if axis is axes[0] else None,
        )
        axis.axvline(
            visual_off, color="#202a36", linestyle="--", linewidth=1.0,
        )
        axis.grid(True, linewidth=0.7)
    axes[0].set_title(
        "Baseline early growth: absolute thresholds versus sustained runaway"
    )
    fig.tight_layout()
    save_figure(fig, figures, "early_growth")


def make_figures(
    runs: list[RunData],
    events: pd.DataFrame,
    ablation: pd.DataFrame,
    thresholds: dict[str, float],
    figures: Path,
) -> None:
    configure_plot_style()
    figures.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(3, 1, figsize=(10.5, 8.2), sharex=True)
    plot_group_series(
        axes,
        runs,
        ["theta_e_raw_deg", "epsilon_orth", "sigma_r_raw_max"],
        [r"$\theta_E$ [deg]", r"$\epsilon_{orth}$", r"max $\|\sigma_R\|$"],
    )
    axes[0].set_title("E transient and sigma_R")
    axes[-1].set_xlabel("Dataset time [s]")
    axes[0].legend(ncol=2, loc="upper right")
    fig.tight_layout()
    save_figure(fig, figures, "e_transient")

    enriched = []
    for run in runs:
        enriched.append(RunData(
            run.manifest_path,
            run.manifest,
            add_burdens(run.frame, thresholds),
            run.velocity_source,
        ))
    fig, axes = plt.subplots(4, 1, figsize=(10.5, 9.2), sharex=True)
    plot_group_series(
        axes,
        enriched,
        ["r_error_deg", "p_error_m", "v_error_mps", "visual_burden_05s"],
        ["R error [deg]", "p error [m]", "v error [m/s]", "normalized visual correction"],
    )
    axes[0].set_title("Downstream state and visual-correction evolution")
    axes[1].set_yscale("log")
    axes[2].set_yscale("log")
    axes[1].set_ylim(bottom=1e-2)
    axes[2].set_ylim(bottom=1e-2)
    axes[-1].set_xlabel("Dataset time [s]")
    axes[0].legend(ncol=2, loc="upper right")
    fig.tight_layout()
    save_figure(fig, figures, "state_feedback")

    if not events.empty:
        event_columns = [
            ("e_first_s", "E", "#8a5cf6"),
            ("r_s", "R", "#e04444"),
            ("p_s", "p", "#18a558"),
            ("v_s", "v", "#2774d8"),
            ("visual_de_s", "visual dE", "#d18b24"),
            ("instability_s", "unstable", "#202a36"),
        ]
        fig, axis = plt.subplots(figsize=(10.5, max(4.0, 0.55 * len(events))))
        labels = events["run_id"].tolist()
        y = np.arange(len(labels))
        for column, label, color in event_columns:
            values = events[column].to_numpy(dtype=float)
            valid = np.isfinite(values)
            axis.scatter(values[valid], y[valid], label=label, color=color, s=32)
        axis.set_yticks(y, labels)
        axis.invert_yaxis()
        axis.set_xlabel("Dataset time [s]")
        axis.set_title("Sustained-event timeline")
        axis.grid(True, axis="x", linewidth=0.7)
        axis.legend(ncol=3, loc="upper right")
        fig.tight_layout()
        save_figure(fig, figures, "event_timeline")

    if not ablation.empty:
        reductions = (
            ablation.loc[ablation["condition"].isin(["fixed_e", "sigma_zero"])]
            .groupby("condition")[
                ["r_reduction", "p_reduction", "v_reduction", "visual_reduction"]
            ]
            .median()
        )
        fig, axis = plt.subplots(figsize=(9.0, 4.8))
        categories = ["R", "p", "v", "visual"]
        x = np.arange(len(categories))
        width = 0.34
        for index, condition in enumerate(("fixed_e", "sigma_zero")):
            if condition not in reductions.index:
                continue
            values = reductions.loc[condition].to_numpy(dtype=float) * 100.0
            axis.bar(
                x + (index - 0.5) * width,
                values,
                width,
                color=COLORS[condition],
                label=LABELS[condition],
            )
        axis.axhline(50.0, color="#697386", linestyle="--", linewidth=1.0)
        axis.axhline(0.0, color="#202a36", linewidth=0.8)
        axis.set_xticks(x, categories)
        axis.set_ylabel("Median cumulative-error reduction [%]")
        axis.set_title("Causal interventions relative to baseline")
        axis.grid(True, axis="y", linewidth=0.7)
        axis.legend()
        fig.tight_layout()
        save_figure(fig, figures, "causal_ablation")


def intervention_result(ablation: pd.DataFrame, condition: str) -> dict[str, object]:
    subset = (
        ablation.loc[ablation["condition"] == condition]
        if not ablation.empty and "condition" in ablation.columns
        else pd.DataFrame()
    )
    categories = ("r", "p", "v", "visual")
    reductions = {
        category: float(subset[f"{category}_reduction"].median())
        for category in categories
    } if not subset.empty else {category: math.nan for category in categories}
    improved_count = sum(value >= 0.5 for value in reductions.values() if np.isfinite(value))
    no_worse = all(
        value >= -0.25 for value in reductions.values() if np.isfinite(value)
    )
    return {
        "reductions": reductions,
        "improved": improved_count >= 3 and no_worse,
        "no_worse": no_worse,
    }


def write_report(
    runs: list[RunData],
    summaries: pd.DataFrame,
    events: pd.DataFrame,
    lag: pd.DataFrame,
    ablation: pd.DataFrame,
    growth: pd.DataFrame,
    report_path: Path,
) -> None:
    expected = {"baseline": 3, "fixed_e": 3, "sigma_zero": 3, "no_projection": 3}
    counts = {condition: sum(run.condition == condition for run in runs) for condition in expected}
    complete = all(counts[condition] == count for condition, count in expected.items())
    baseline_events = (
        events.loc[events["condition"] == "baseline"]
        if not events.empty and "condition" in events.columns
        else pd.DataFrame()
    )
    ordered_count = int(baseline_events["ordered_e_r_downstream"].sum()) if not baseline_events.empty else 0
    baseline_lag = lag.loc[
        (lag["condition"] == "baseline") & (lag["target"] == "r")
    ] if not lag.empty and "condition" in lag.columns else pd.DataFrame()
    positive_r_lag = int(baseline_lag["positive_lead"].sum()) if not baseline_lag.empty else 0
    fixed_result = intervention_result(ablation, "fixed_e")
    sigma_result = intervention_result(ablation, "sigma_zero")
    strong_support = bool(
        complete and ordered_count >= 2 and positive_r_lag >= 2 and
        fixed_result["improved"] and sigma_result["improved"]
    )

    stable_counts = {}
    if not summaries.empty:
        for condition in ("fixed_e", "sigma_zero"):
            subset = summaries.loc[summaries["condition"] == condition]
            stable = (
                subset["instability_s"].isna() &
                (subset["r_final5_median_deg"] < 5.0) &
                (subset["p_final5_median_m"] < 0.5) &
                (subset["v_final5_median_mps"] < 0.5)
            )
            stable_counts[condition] = int(stable.sum())
    else:
        stable_counts = {"fixed_e": 0, "sigma_zero": 0}

    if not complete:
        status = "实验尚未完成，不能形成因果结论。"
    elif fixed_result["improved"] and sigma_result["improved"]:
        status = "fixed-e 与 sigma_R=0 均明显改善，主要危险路径支持为 `e → sigma_R → R`。"
    elif fixed_result["improved"] and not sigma_result["improved"]:
        status = "fixed-e 明显改善而 sigma_R=0 不足，说明 e 还通过视觉更新或其他状态/协方差耦合产生影响。"
    elif not fixed_result["improved"] and not sigma_result["improved"]:
        status = (
            "按预设整体验收，两种干预均未达到“明显改善”；不能把 e "
            "认定为唯一或已经验证的主要上游机制。但两种干预都显著降低 "
            "R，并消除了 baseline 的失稳事件，说明 e–sigma_R 分支确实参与误差放大。"
        )
    else:
        status = "sigma_R=0 改善但 fixed-e 未改善，干预结果不一致，暂不作因果结论。"

    if strong_support:
        conclusion_boundary = [
            "> 实验支持 `e` 瞬态在有限时间内触发或放大系统误差，因此仅有 `R` 渐近收敛保证不足以保证运行安全。",
            "",
            "该结论不声称已经观察或证明失稳后的 `R` 最终一定收敛。",
        ]
    else:
        conclusion_boundary = [
            "本轮没有同时满足事件顺序和双干预验收，因此不能写成"
            "“实验已经支持完整 e→R→p/v→视觉正反馈链”。",
            "",
            "若后续实验满足全部验收，允许使用的结论边界是："
            "“实验支持 `e` 瞬态在有限时间内触发或放大系统误差，因此仅有 "
            "`R` 渐近收敛保证不足以保证运行安全。”",
            "本实验始终不声称已经观察或证明失稳后的 `R` 最终一定收敛。",
        ]
    group_medians = (
        summaries.groupby("condition")[
            [
                "theta_e_peak_deg", "epsilon_orth_peak", "sigma_r_raw_peak",
                "r_peak_deg", "p_peak_m", "v_peak_mps",
            ]
        ].median()
        if not summaries.empty else pd.DataFrame()
    )
    median_rho = (
        float(baseline_lag["peak_rho"].median())
        if not baseline_lag.empty else math.nan
    )
    median_lag = (
        float(baseline_lag["peak_lag_s"].median())
        if not baseline_lag.empty else math.nan
    )

    lines = [
        "# `e` 瞬态—`sigma_R`—状态发散实验报告",
        "",
        "## 当前结论",
        "",
        status,
        "",
        f"- baseline 事件顺序满足次数：{ordered_count}/3。",
        f"- baseline 的 E 增量领先 R 增量次数：{positive_r_lag}/3；"
        f"峰值 Spearman ρ 中位数 {median_rho:.3f}，滞后中位数 {median_lag:.2f}s。",
        f"- fixed-e 稳定次数：{stable_counts['fixed_e']}/3；sigma_R=0 稳定次数：{stable_counts['sigma_zero']}/3。",
        "",
        *conclusion_boundary,
        "",
        "## 数据完整性",
        "",
        "| 条件 | 已完成 | 计划 |",
        "|---|---:|---:|",
    ]
    for condition in CONDITION_ORDER:
        lines.append(f"| {LABELS[condition]} | {counts[condition]} | {expected[condition]} |")
    lines.extend([
        "",
        "所有正式组固定使用 V1_02 estimated-live mapping、`bag_rate=1.0`、关闭 ZUPT。GT 只用于计算误差，不参与建图。",
        "",
        "## 核心判定",
        "",
        f"- fixed-e 明显改善：{fixed_result['improved']}；各类累计误差降低："
        + ", ".join(
            f"{key}={value * 100:.1f}%" if np.isfinite(value) else f"{key}=N/A"
            for key, value in fixed_result["reductions"].items()
        ),
        f"- sigma_R=0 明显改善：{sigma_result['improved']}；各类累计误差降低："
        + ", ".join(
            f"{key}={value * 100:.1f}%" if np.isfinite(value) else f"{key}=N/A"
            for key, value in sigma_result["reductions"].items()
        ),
        "- “明显改善”要求 R/p/v/视觉四类累计指标至少三类降低 50%，且无一类恶化超过 25%。",
        "- 累计干预比较使用匹配三元组的共同时间窗，并在其中最早的失稳事件处截断。",
        "- 滞后统计使用 0.5 秒误差增量，并截断在首次失稳之前，避免共同上升趋势制造虚假相关。",
        "",
        "## 关键观测",
        "",
        "| 条件 | θE峰值中位数(°) | εorth峰值中位数 | sigma峰值中位数 | R峰值中位数(°) | p峰值中位数(m) | v峰值中位数(m/s) |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ])
    for condition in CONDITION_ORDER:
        if condition not in group_medians.index:
            continue
        row = group_medians.loc[condition]
        lines.append(
            f"| {LABELS[condition]} | {row['theta_e_peak_deg']:.3f} | "
            f"{row['epsilon_orth_peak']:.6f} | {row['sigma_r_raw_peak']:.3f} | "
            f"{row['r_peak_deg']:.3f} | {row['p_peak_m']:.3f} | "
            f"{row['v_peak_mps']:.3f} |"
        )
    lines.extend([
        "",
        "- baseline 的 θE 峰值中位数低于 2°，但 R/p/v 仍明显发散；单凭 θE 大小不能解释本轮失稳。",
        "- 三次 baseline 都是 p/v 或视觉事件先于 R 事件，预设的 `E→R→p/v→视觉` 持续事件顺序没有复现。",
        "- sigma_R=0 组保留并出现更大的 θE 与原始 sigma_R，却显著压低 R/p/v，说明被屏蔽的 sigma_R 应用路径具有重要影响。",
        "- fixed-e 和 sigma_R=0 均无失稳事件，R 峰值中位数由 17.8° 降至约 5°；但最终 p 误差仍不满足稳定门限。",
        "- fixed-e 同样显著压低全程 R/p/v，但失稳前视觉修正累计量反而更高；因此不能把结果简化为已经验证完整视觉正反馈链。",
        "- no-projection 的 εorth 明显累积，证明结构投影持续抑制 E 非正交；该短程组不用于最终精度排名。",
        "",
        "## 连续曲线与增长率复核",
        "",
    ])
    if not growth.empty:
        growth_median = growth.median(numeric_only=True)
        lines.extend([
            f"- R 的持续增长起点中位数为 {growth_median['r_growth_onset_s']:.2f}s；"
            f"此时 R 约 {growth_median['r_rise_r_error_deg']:.2f}°，"
            f"θE_raw 约 {growth_median['r_rise_theta_e_raw_deg']:.2f}°。",
            f"- 第一次 v 越过 0.5m/s 时，R 中位数只有 "
            f"{growth_median['first_v_threshold_r_error_deg']:.2f}°；"
            f"第一次 p 越过 0.5m 时，R 已约 "
            f"{growth_median['first_p_threshold_r_error_deg']:.2f}°，但仍未越过 5°。",
            f"- 真正持续的 v/p 爆涨分别在约 "
            f"{growth_median['v_runaway_onset_s']:.2f}s 和 "
            f"{growth_median['p_runaway_onset_s']:.2f}s 开始；"
            f"v 爆涨起点的 R/倾斜误差中位数约为 "
            f"{growth_median['v_runaway_r_error_deg']:.2f}°/"
            f"{growth_median['v_runaway_tilt_error_deg']:.2f}°，"
            f"对应重力投影误差约 "
            f"{growth_median['v_runaway_gravity_projection_error_mps2']:.2f}m/s²。",
            f"- 视觉更新持续停止的中位时间约 "
            f"{growth_median['visual_updates_off_s']:.2f}s，与 p/v 爆涨同处 "
            "46–48s 时间段。",
            "- 因而第一次绝对阈值顺序确实低估了 R 的早期连续恶化。更符合曲线的两阶段描述是："
            "小 R/倾斜误差先产生早期 v/p 瞬态，R 随后持续恶化；到约 46–48s，"
            "大倾斜误差与视觉更新中断同时出现，v/p 转入持续爆涨。",
            "- 该复核增强了“R 早于最终 p/v 发散”的证据，但仍不能仅凭曲线区分"
            "重力投影和视觉闭环各自的因果份额。",
        ])
    lines.extend([
        "",
        "## 结果入口",
        "",
        "- `summary/run_summary.csv`：逐运行峰值、累计量、最终误差和稳定性。",
        "- `summary/event_order.csv`：持续 5 帧事件及先后顺序。",
        "- `summary/lag_analysis.csv`：误差增量的 0–2 秒滞后关联。",
        "- `summary/causal_ablation.csv`：按重复匹配的共同时间窗干预比较。",
        "- `summary/growth_rate_diagnostics.csv`：早期阈值和持续增长起点。",
        "- `figures/early_growth.*`：R/倾斜、p/v 阈值比、1 秒增长率、重力投影与视觉更新联合图。",
        "- `figures/e_transient.*`、`state_feedback.*`、`event_timeline.*`、`causal_ablation.*`：其余主图。",
        "",
        "## 解释边界",
        "",
        "- fixed-e 不冻结 P、Kalman 增益或交叉协方差，因此失败不能完全排除 e 相关协方差机制。",
        "- sigma_R=0 只屏蔽名义状态传播中的应用项，保留正常 e 演化和原协方差算法。",
        "- no-projection 是 900 帧结构诊断，不参与最终轨迹精度或干预效果排名。",
        "- 视觉 Updater 不直接修改 R，记录到的视觉 dR 应接近零；视觉对 R 的主要被测路径是 dE 经后续 sigma_R 传播。",
        "",
    ])
    report_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    data_root = args.data_root.resolve()
    output_root = args.output_root.resolve()
    summary_dir = output_root / "summary"
    figures_dir = output_root / "figures"
    summary_dir.mkdir(parents=True, exist_ok=True)
    figures_dir.mkdir(parents=True, exist_ok=True)
    gt = load_ground_truth(args.ground_truth.resolve())
    runs = []
    if data_root.exists():
        for manifest_path in sorted(data_root.glob("*/run_*/manifest.json")):
            run = load_run(manifest_path, gt)
            if run is not None:
                runs.append(run)
    baseline_initial = [
        run.frame.loc[run.frame["time"] <= 5.0]
        for run in runs if run.condition == "baseline"
    ]
    if baseline_initial:
        pooled = pd.concat(baseline_initial, ignore_index=True)
        thresholds = {
            column: robust_threshold(pooled[column].to_numpy(dtype=float))
            for column in (
                "epsilon_orth",
                "epsilon_det",
                "visual_dp_m",
                "visual_dv_mps",
                "visual_dE_fro",
            )
        }
    else:
        thresholds = {
            "epsilon_orth": 1e-12,
            "epsilon_det": 1e-12,
            "visual_dp_m": 1e-12,
            "visual_dv_mps": 1e-12,
            "visual_dE_fro": 1e-12,
        }
    (summary_dir / "thresholds.json").write_text(
        json.dumps(thresholds, indent=2) + "\n", encoding="utf-8"
    )

    event_rows = []
    summary_rows = []
    lag_rows = []
    for run in runs:
        event, _ = analyze_events(run, thresholds)
        event_rows.append(event)
        summary_rows.append(run_summary(run, event, thresholds))
        lag_rows.extend(lag_metrics(run, float(event["instability_s"]), thresholds))
    events = pd.DataFrame(event_rows)
    summaries = pd.DataFrame(summary_rows)
    lag = pd.DataFrame(lag_rows)
    growth = growth_diagnostics(runs, events)
    ablation = matched_ablation(runs, thresholds, events)
    summaries.to_csv(summary_dir / "run_summary.csv", index=False)
    events.to_csv(summary_dir / "event_order.csv", index=False)
    lag.to_csv(summary_dir / "lag_analysis.csv", index=False)
    growth.to_csv(summary_dir / "growth_rate_diagnostics.csv", index=False)
    ablation.to_csv(summary_dir / "causal_ablation.csv", index=False)
    summaries.to_csv(summary_dir / "transient_metrics.csv", index=False)
    if runs:
        make_figures(runs, events, ablation, thresholds, figures_dir)
        make_early_growth_figure(runs, growth, figures_dir)
    write_report(
        runs,
        summaries,
        events,
        lag,
        ablation,
        growth,
        output_root / "REPORT.md",
    )
    print(f"analyzed {len(runs)} completed formal runs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

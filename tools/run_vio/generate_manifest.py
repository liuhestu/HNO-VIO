#!/usr/bin/env python3
import argparse
import json
import random
from pathlib import Path


def clamp_int(value, low, high):
    return max(low, min(high, int(round(value))))


def candidate(case_id, rng):
    if case_id <= 50:
        stage = 1
        code_variant = "param_switch_search"
    elif case_id <= 100:
        stage = 2
        code_variant = "landmark_policy_search"
    else:
        stage = 3
        code_variant = "updater_interpretation_small_step"

    params = {
        "update_chi2_gate": rng.choice([8.0, 10.0, 12.0, 15.0, 18.0, 22.0, 26.0]),
        "update_max_delta_p": rng.choice([0.08, 0.12, 0.16, 0.20, 0.25, 0.30]),
        "update_max_delta_r": rng.choice([0.06, 0.10, 0.12, 0.15, 0.20]),
        "update_pixel_noise": rng.choice([1.0, 1.5, 2.0, 2.5, 3.0]),
        "update_enforce_structure": rng.choice([False, True]),
        "feature_tracker_num_pts": rng.choice([160, 200, 240, 280, 320]),
        "feature_tracker_fast_threshold": rng.choice([12, 15, 20, 25, 30]),
        "feature_tracker_grid_x": rng.choice([4, 5, 6]),
        "feature_tracker_grid_y": rng.choice([4, 5, 6]),
        "feature_tracker_min_px_dist": rng.choice([10, 12, 15, 18, 22]),
        "feature_max_stereo_depth": rng.choice([4.0, 5.0, 6.0, 8.0, 10.0]),
        "feature_stereo_reproj_thresh": rng.choice([0.010, 0.015, 0.020, 0.030]),
        "feature_reproj_thresh": rng.choice([0.05, 0.07, 0.08, 0.10, 0.12]),
        "feature_reproj_thresh_low": rng.choice([0.08, 0.10, 0.12, 0.15]),
        "feature_mature_thresh": rng.choice([2, 3, 4]),
        "feature_mature_thresh_low": rng.choice([1, 2, 3]),
        "feature_fail_limit": rng.choice([3, 5, 7]),
        "feature_fail_limit_low": rng.choice([5, 8, 10]),
        "feature_map_jump_thresh": rng.choice([0.25, 0.40, 0.50, 0.75, 1.00]),
        "feature_active_mature_thresh": rng.choice([2, 3, 4]),
        "feature_health_min_stable": rng.choice([15, 20, 25]),
        "feature_health_min_db": rng.choice([15, 20, 25]),
        "feature_health_hold_frames": rng.choice([3, 5]),
        "feature_health_start_frame": rng.choice([60, 90]),
        "update_min_observations": rng.choice([15, 20, 25]),
        "update_low_observation_hold_frames": rng.choice([3, 5]),
        "update_warn_delta_ratio": 0.8,
    }

    if case_id == 1:
        params.update({
            "update_chi2_gate": 15.0,
            "update_max_delta_p": 0.20,
            "update_max_delta_r": 0.15,
            "update_pixel_noise": 2.0,
            "update_enforce_structure": False,
            "feature_tracker_num_pts": 200,
            "feature_tracker_fast_threshold": 20,
            "feature_tracker_grid_x": 5,
            "feature_tracker_grid_y": 5,
            "feature_tracker_min_px_dist": 15,
            "feature_max_stereo_depth": 5.0,
            "feature_stereo_reproj_thresh": 0.015,
            "feature_reproj_thresh": 0.08,
            "feature_reproj_thresh_low": 0.10,
            "feature_mature_thresh": 3,
            "feature_mature_thresh_low": 2,
            "feature_fail_limit": 5,
            "feature_fail_limit_low": 8,
            "feature_map_jump_thresh": 0.50,
            "feature_active_mature_thresh": 3,
            "feature_health_min_stable": 20,
            "feature_health_min_db": 20,
            "feature_health_hold_frames": 3,
            "feature_health_start_frame": 60,
            "update_min_observations": 20,
            "update_low_observation_hold_frames": 3,
            "update_warn_delta_ratio": 0.8,
        })

    if stage >= 2:
        params["feature_max_stereo_depth"] = rng.choice([5.0, 8.0, 12.0, 15.0])
        params["feature_active_mature_thresh"] = rng.choice([1, 2, 3])
        params["feature_map_jump_thresh"] = rng.choice([0.40, 0.75, 1.25, 1.75])
        params["feature_health_hold_frames"] = rng.choice([3, 5, 8])

    if stage >= 3:
        params["update_enforce_structure"] = True
        params["update_chi2_gate"] = rng.choice([6.0, 8.0, 10.0, 12.0, 15.0])
        params["update_max_delta_p"] = rng.choice([0.04, 0.06, 0.08, 0.12, 0.16])
        params["update_max_delta_r"] = rng.choice([0.03, 0.05, 0.08, 0.10])

    params["feature_low_feature_pts"] = clamp_int(params["feature_tracker_num_pts"] * rng.uniform(0.30, 0.55), 40, 160)
    params["feature_low_feature_db"] = clamp_int(params["feature_tracker_num_pts"] * rng.uniform(0.20, 0.45), 30, 140)

    return {
        "case_id": f"case_{case_id:03d}",
        "stage": stage,
        "seed": rng.randrange(1, 2**31 - 1),
        "code_variant": code_variant,
        "params": params,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--count", type=int, default=150)
    parser.add_argument("--seed", type=int, default=20260624)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    rng = random.Random(args.seed)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as f:
        for idx in range(1, args.count + 1):
            entry = candidate(idx, rng)
            entry["dataset"] = args.dataset
            f.write(json.dumps(entry, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()

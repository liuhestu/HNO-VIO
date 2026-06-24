#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


BASE_CASE009 = {
    "feature_active_mature_thresh": 4,
    "feature_fail_limit": 5,
    "feature_fail_limit_low": 10,
    "feature_low_feature_db": 61,
    "feature_low_feature_pts": 62,
    "feature_map_jump_thresh": 0.4,
    "feature_mature_thresh": 4,
    "feature_mature_thresh_low": 2,
    "feature_max_stereo_depth": 6.0,
    "feature_reproj_thresh": 0.05,
    "feature_reproj_thresh_low": 0.08,
    "feature_stereo_reproj_thresh": 0.01,
    "feature_tracker_fast_threshold": 20,
    "feature_tracker_grid_x": 4,
    "feature_tracker_grid_y": 6,
    "feature_tracker_min_px_dist": 15,
    "feature_tracker_num_pts": 160,
    "update_enforce_structure": True,
    "update_pixel_noise": 1.5,
}


def candidate(idx, dataset):
    chi2 = [8.0, 10.0, 12.0, 15.0, 18.0]
    max_dp = [0.06, 0.08, 0.12, 0.16]
    max_dr = [0.05, 0.08, 0.10, 0.12]
    map_jump = [0.4, 0.5]
    active_mature = [3, 4]

    params = dict(BASE_CASE009)
    params.update({
        "update_chi2_gate": chi2[(idx - 1) % len(chi2)],
        "update_max_delta_p": max_dp[((idx - 1) // len(chi2)) % len(max_dp)],
        "update_max_delta_r": max_dr[((idx - 1) // (len(chi2) * len(max_dp))) % len(max_dr)],
        "feature_map_jump_thresh": map_jump[(idx - 1) % len(map_jump)],
        "feature_active_mature_thresh": active_mature[((idx - 1) // len(map_jump)) % len(active_mature)],
        "feature_health_min_stable": 20,
        "feature_health_min_db": 20,
        "feature_health_hold_frames": 3,
        "feature_health_start_frame": 60,
        "update_min_observations": 20,
        "update_low_observation_hold_frames": 3,
        "update_warn_delta_ratio": 0.8,
    })
    return {
        "case_id": f"case009_guarded_{idx:03d}",
        "stage": 1,
        "seed": 909000 + idx,
        "code_variant": "case009_guarded_local_search",
        "dataset": dataset,
        "params": params,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as f:
        for idx in range(1, args.count + 1):
            f.write(json.dumps(candidate(idx, args.dataset), sort_keys=True) + "\n")


if __name__ == "__main__":
    main()

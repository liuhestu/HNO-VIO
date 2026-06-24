#!/usr/bin/env python3
import argparse
import json
import shutil
from pathlib import Path


HNO_CANDIDATE_KEYS = {
    "feature_active_mature_thresh",
    "feature_fail_limit",
    "feature_fail_limit_low",
    "feature_health_hold_frames",
    "feature_health_min_db",
    "feature_health_min_stable",
    "feature_health_start_frame",
    "feature_low_feature_db",
    "feature_low_feature_pts",
    "feature_map_jump_thresh",
    "feature_mature_thresh",
    "feature_mature_thresh_low",
    "feature_min_stereo_depth",
    "feature_max_stereo_depth",
    "feature_reproj_thresh",
    "feature_reproj_thresh_low",
    "feature_stereo_reproj_thresh",
    "feature_tracker_fast_threshold",
    "feature_tracker_grid_x",
    "feature_tracker_grid_y",
    "feature_tracker_min_px_dist",
    "feature_tracker_num_pts",
    "update_chi2_gate",
    "update_enforce_structure",
    "update_focal_length",
    "update_low_observation_hold_frames",
    "update_max_delta_p",
    "update_max_delta_r",
    "update_min_observations",
    "update_pixel_noise",
    "update_warn_delta_ratio",
    "use_gt_init",
    "use_gt_mapping",
}


def yaml_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return f'"{value}"'
    return str(value)


def yaml_key(line):
    stripped = line.strip()
    if not stripped or stripped.startswith("#") or ":" not in stripped:
        return None
    return stripped.split(":", 1)[0].strip()


def strip_hno_candidate_keys(text):
    filtered = []
    removed = []
    for line in text.splitlines():
        key = yaml_key(line)
        if key in HNO_CANDIDATE_KEYS:
            removed.append(key)
            continue
        filtered.append(line)
    return "\n".join(filtered).rstrip(), removed


def assert_no_duplicate_hno_keys(path):
    seen = {}
    duplicates = {}
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        key = yaml_key(line)
        if key not in HNO_CANDIDATE_KEYS:
            continue
        if key in seen:
            duplicates.setdefault(key, [seen[key]]).append(line_no)
        else:
            seen[key] = line_no
    if duplicates:
        details = ", ".join(f"{key}@{lines}" for key, lines in sorted(duplicates.items()))
        raise RuntimeError(f"duplicate HNO candidate keys in {path}: {details}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-config", required=True)
    parser.add_argument("--candidate-json", required=True)
    parser.add_argument("--output-config", required=True)
    parser.add_argument("--output-params", required=True)
    args = parser.parse_args()

    candidate = json.loads(args.candidate_json)
    base_config = Path(args.base_config)
    output_config = Path(args.output_config)
    output_params = Path(args.output_params)
    output_config.parent.mkdir(parents=True, exist_ok=True)

    text, removed = strip_hno_candidate_keys(base_config.read_text(encoding="utf-8"))
    with output_config.open("w", encoding="utf-8") as f:
        f.write(text.rstrip())
        f.write("\n\n# Auto-converge candidate parameters.\n")
        f.write('use_gt_init: false\n')
        f.write('use_gt_mapping: false\n')
        for key, value in sorted(candidate["params"].items()):
            f.write(f"{key}: {yaml_value(value)}\n")

    assert_no_duplicate_hno_keys(output_config)
    output_params.write_text(json.dumps(candidate, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    for external in ("kalibr_imu_chain.yaml", "kalibr_imucam_chain.yaml"):
        src = base_config.parent / external
        if src.exists():
            shutil.copy2(src, output_config.parent / external)


if __name__ == "__main__":
    main()

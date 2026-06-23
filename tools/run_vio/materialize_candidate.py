#!/usr/bin/env python3
import argparse
import json
import shutil
from pathlib import Path


def yaml_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return f'"{value}"'
    return str(value)


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

    text = base_config.read_text(encoding="utf-8")
    with output_config.open("w", encoding="utf-8") as f:
        f.write(text.rstrip())
        f.write("\n\n# Auto-converge candidate parameters.\n")
        f.write('use_gt_init: false\n')
        f.write('use_gt_mapping: false\n')
        for key, value in sorted(candidate["params"].items()):
            f.write(f"{key}: {yaml_value(value)}\n")

    output_params.write_text(json.dumps(candidate, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    for external in ("kalibr_imu_chain.yaml", "kalibr_imucam_chain.yaml"):
        src = base_config.parent / external
        if src.exists():
            shutil.copy2(src, output_config.parent / external)


if __name__ == "__main__":
    main()

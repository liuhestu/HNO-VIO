#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def load_summaries(exp_dir):
    summaries = []
    for path in sorted(Path(exp_dir).glob("case*/summary.json")):
        summaries.append(json.loads(path.read_text(encoding="utf-8")))
    return summaries


def metric(summary, key, default=1e9):
    value = summary.get(key)
    return default if value is None else value


def score(summary):
    if not summary.get("usable"):
        return (1, metric(summary, "se3_ate_rmse"), abs(metric(summary, "path_length_ratio", 0.0) - 1.0))
    return (
        0,
        metric(summary, "se3_ate_rmse"),
        metric(summary, "rpe1_trans_rmse"),
        abs(metric(summary, "path_length_ratio", 0.0) - 1.0),
    )


def table_row(summary):
    keys = [
        "case_id", "failure_reason", "usable", "se3_ate_rmse", "se3_ate_median",
        "rpe1_trans_rmse", "rpe1_rot_rmse_deg", "path_length_ratio", "duration_sec",
        "update_accept_ratio", "tail_E_orth_frob", "mean_num_features", "active_landmarks_tail",
        "first_divergence_sec", "min_active_landmarks_tail10", "max_delta_p_seen", "max_delta_r_seen",
    ]
    values = []
    for key in keys:
        value = summary.get(key)
        if isinstance(value, float):
            value = f"{value:.4g}"
        values.append(str(value))
    return "| " + " | ".join(values) + " |"


def write_report(exp_dir, dataset, output, final=False):
    exp_dir = Path(exp_dir)
    summaries = load_summaries(exp_dir)
    usable = [s for s in summaries if s.get("usable")]
    strong = [s for s in summaries if s.get("strong_baseline")]
    best = sorted(summaries, key=score)[0] if summaries else None

    lines = [
        f"# {dataset} {'Final' if final else 'Round'} Report",
        "",
        f"- experiment_dir: `{exp_dir}`",
        f"- cases_completed: {len(summaries)}",
        f"- usable_cases: {len(usable)}",
        f"- strong_baseline_cases: {len(strong)}",
    ]
    if best:
        lines.extend([
            f"- best_case: `{best['case_id']}`",
            f"- best_failure_reason: {best.get('failure_reason')}",
            f"- best_usable: {best.get('usable')}",
            f"- best_strong_baseline: {best.get('strong_baseline')}",
        ])

    failures = {}
    for summary in summaries:
        reason = summary.get("failure_reason", "UNKNOWN")
        failures[reason] = failures.get(reason, 0) + 1
    if failures:
        lines.extend(["", "## Failure Counts", ""])
        for reason, count in sorted(failures.items(), key=lambda item: (-item[1], item[0])):
            lines.append(f"- {reason}: {count}")

    lines.extend([
        "",
        "## Cases",
        "",
        "| case_id | failure_reason | usable | ate_rmse | ate_median | rpe_t_rmse | rpe_r_deg | path_ratio | duration | accept | EOrth | mean_feat | active_tail | div_sec | min_tail10 | max_dP | max_dR |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ])
    for summary in sorted(summaries, key=lambda s: s.get("case_id", "")):
        lines.append(table_row(summary))

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")

    if best:
        best_path = exp_dir / "best_candidate.json"
        case_params = exp_dir / best["case_id"] / "params.json"
        payload = {
            "dataset": dataset,
            "experiment_dir": str(exp_dir),
            "case_dir": str(exp_dir / best["case_id"]),
            "summary": best,
            "candidate": json.loads(case_params.read_text(encoding="utf-8")) if case_params.exists() else {},
        }
        best_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--exp-dir", required=True)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--final", action="store_true")
    args = parser.parse_args()
    write_report(args.exp_dir, args.dataset, args.output, args.final)


if __name__ == "__main__":
    main()

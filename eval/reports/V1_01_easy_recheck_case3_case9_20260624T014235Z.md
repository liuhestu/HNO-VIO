# V1_01_easy case_003 / case_009 Recheck

- experiment_dir: `/home/sharpa/hno_vio_clean/src/hno_vio/eval/V1_01_easy_recheck_case3_case9_20260624T014235Z`
- runner: `tools/run_vio/run_case.sh` with `rviz=false`, matching the 50-run auto-converge case path
- use_gt_init: false
- use_gt_mapping: false

## Result

| case | original usable | recheck usable | recheck failure | ate_rmse | path_ratio | accept | active_tail |
|---|---:|---:|---|---:|---:|---:|---:|
| case_003 | True | False | FEATURE_STARVATION | 401.323 | 29.7597 | 0.762727 | 19 |
| case_009 | True | True | SUCCESS | 0.394582 | 1.09581 | 0.839126 | 32 |

## Duplicate Parameter Check

| case | key | values in materialized config |
|---|---|---|
| case_003 | `update_chi2_gate` | `['8.0', '8.0']` |
| case_003 | `feature_tracker_grid_y` | `['4', '4']` |
| case_003 | `feature_map_jump_thresh` | `['1.0', '1.0']` |
| case_003 | `update_max_delta_p` | `['0.08', '0.08']` |
| case_003 | `feature_reproj_thresh` | `['0.07', '0.07']` |
| case_009 | `update_chi2_gate` | `['8.0', '26.0']` |
| case_009 | `feature_tracker_grid_y` | `['4', '6']` |
| case_009 | `feature_map_jump_thresh` | `['1.0', '0.4']` |
| case_009 | `update_max_delta_p` | `['0.08', '0.3']` |
| case_009 | `feature_reproj_thresh` | `['0.07', '0.05']` |

## Conclusion

- `case_003_recheck` did not reproduce and failed with `FEATURE_STARVATION`.
- `case_009_recheck` reproduced a strong usable result close to the original case_009 metrics.
- The materialized configs now contain duplicate HNO parameter keys because default `estimator_config.yaml` already includes case_003 parameters and `run_case.sh` appends candidate parameters again.
- This confirms a configuration-chain hazard. The safest fix is to make candidate materialization remove existing HNO search keys before appending a candidate, or use a clean base config without embedded candidate defaults for auto-converge runs.

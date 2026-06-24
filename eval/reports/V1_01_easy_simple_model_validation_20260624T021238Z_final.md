# V1_01_easy Simple Model Validation Final Report

- validation_dir: `/home/sharpa/hno_vio_clean/src/hno_vio/eval/V1_01_easy_simple_model_validation_20260624T021238Z`
- model: clean base `estimator_config.yaml`; each case directory owns its generated `estimator_config.yaml`
- headless validation: 3 repeats per case
- RViz replay: 1 repeat per case
- use_gt_init: false
- use_gt_mapping: false

## Headless 3/3 Results

| case | usable_repeats | strong_repeats | promotion |
|---|---:|---:|---|
| case_003 | 0/3 | 0/3 | rejected |
| case_009 | 0/3 | 0/3 | rejected |

## Repeat Metrics

| case | repeat | failure_reason | usable | ate_rmse | ate_median | rpe_t | rpe_r_deg | path_ratio | accept | active_tail |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| case_003 | 1 | FEATURE_STARVATION | False | 428.114 | 243.58 | 1.06817 | 0.179059 | 30.6403 | 0.779058 | 11 |
| case_003 | 2 | FEATURE_STARVATION | False | 15.0044 | 4.03989 | 0.146523 | 0.235115 | 3.18946 | 0.800753 | 17 |
| case_003 | 3 | FEATURE_STARVATION | False | 25.953 | 6.99827 | 0.212232 | 0.218041 | 4.34009 | 0.812779 | 7 |
| case_009 | 1 | ATE_DIVERGED | False | 764.173 | 358.915 | 2.42819 | 0.220045 | 59.8255 | 0.929497 | 20 |
| case_009 | 2 | PATH_LENGTH_BAD | False | 1.14475 | 0.840394 | 0.015329 | 0.30428 | 1.25129 | 0.94386 | 20 |
| case_009 | 3 | FEATURE_STARVATION | False | 9.95178 | 2.2714 | 0.127481 | 0.243622 | 2.89383 | 0.933985 | 11 |

## RViz Replay Metrics

| case | failure_reason | usable | ate_rmse | path_ratio | accept | active_tail |
|---|---|---:|---:|---:|---:|---:|
| case_003 | FEATURE_STARVATION | False | 1606.35 | 102.976 | 0.278207 | 11 |
| case_009 | FEATURE_STARVATION | False | 4162.79 | 269.17 | 0.436515 | 1 |

## Candidate YAML Key Check

| case | key | values in repeat_1 config |
|---|---|---|
| case_003 | `update_chi2_gate` | `['8.0']` |
| case_003 | `feature_tracker_grid_y` | `['4']` |
| case_003 | `feature_map_jump_thresh` | `['1.0']` |
| case_003 | `update_max_delta_p` | `['0.08']` |
| case_003 | `feature_reproj_thresh` | `['0.07']` |
| case_009 | `update_chi2_gate` | `['26.0']` |
| case_009 | `feature_tracker_grid_y` | `['6']` |
| case_009 | `feature_map_jump_thresh` | `['0.4']` |
| case_009 | `update_max_delta_p` | `['0.3']` |
| case_009 | `feature_reproj_thresh` | `['0.05']` |

## Conclusion

- The simple model has been restored: materialized candidate YAMLs contain one value per HNO candidate key.
- Neither case_003 nor case_009 passed 3/3 validation in this run.
- case_009 had one near-miss repeat but failed 0/3 by the existing usable thresholds, so it was not promoted to `eval/best_V1_01_easy.json`.

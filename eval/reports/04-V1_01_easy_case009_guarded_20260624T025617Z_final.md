# V1_01_easy Final Report

- experiment_dir: `src/hno_vio/eval/V1_01_easy_case009_guarded_20260624T025617Z`
- cases_completed: 23
- usable_cases: 4
- strong_baseline_cases: 1
- best_case: `case009_guarded_020`
- best_failure_reason: SUCCESS
- best_usable: True
- best_strong_baseline: True

## Failure Counts

- FEATURE_STARVATION: 17
- SUCCESS: 4
- ATE_DIVERGED: 1
- PATH_LENGTH_BAD: 1

## Cases

| case_id | failure_reason | usable | ate_rmse | ate_median | rpe_t_rmse | rpe_r_deg | path_ratio | duration | accept | EOrth | mean_feat | active_tail | div_sec | min_tail10 | max_dP | max_dR |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| case009_baseline_repeat_1 | FEATURE_STARVATION | False | 7.456 | 2.374 | 0.07433 | 0.2518 | 2.405 | 142.9 | 0.9166 | 0 | 117.1 | 11 | 105.8 | 11 | 0.148 | 0.007 |
| case009_baseline_repeat_2 | SUCCESS | True | 0.5597 | 0.4278 | 0.009888 | 0.2627 | 1.123 | 142.9 | 0.9501 | 0 | 117.4 | 25 | 106.2 | 25 | 0.016 | 0.004 |
| case009_baseline_repeat_3 | FEATURE_STARVATION | False | 460.6 | 203.6 | 1.658 | 0.2734 | 37.99 | 142.9 | 0.9353 | 0 | 118 | 11 | 81.25 | 11 | 0.134 | 0.009 |
| case009_guarded_001 | SUCCESS | True | 0.8011 | 0.2879 | 0.01137 | 0.2429 | 1.194 | 142.9 | 0.8844 | 0 | 117.4 | 22 | 115.1 | 22 | 0.046 | 0.009 |
| case009_guarded_002 | FEATURE_STARVATION | False | 226.8 | 108.2 | 0.6875 | 0.2059 | 17.84 | 142.9 | 0.8613 | 0 | 119.2 | 12 | 81.7 | 12 | 0.365 | 0.009 |
| case009_guarded_003 | FEATURE_STARVATION | False | 8.684 | 2.246 | 0.1082 | 0.2765 | 2.49 | 142.9 | 0.879 | 0 | 117.1 | 10 | 105 | 10 | 0.338 | 0.005 |
| case009_guarded_004 | FEATURE_STARVATION | False | 0.6054 | 0.2908 | 0.01804 | 0.2587 | 1.248 | 142.9 | 0.9263 | 1e-06 | 119.7 | 12 | 113.5 | 12 | 0.013 | 0.004 |
| case009_guarded_005 | FEATURE_STARVATION | False | 22.25 | 5.421 | 0.2358 | 0.2578 | 4.411 | 142.9 | 0.922 | 0 | 116.8 | 7 | 104.6 | 7 | 0.035 | 0.006 |
| case009_guarded_006 | FEATURE_STARVATION | False | 57.95 | 17.69 | 0.3825 | 0.2794 | 7.505 | 142.9 | 0.7824 | 0 | 117.6 | 14 | 99.85 | 14 | 0.377 | 0.01 |
| case009_guarded_007 | ATE_DIVERGED | False | 12.32 | 3.153 | 0.139 | 0.2405 | 3.024 | 142.9 | 0.8498 | 4e-06 | 117.6 | 30 | 64.8 | 12 | 0.053 | 0.004 |
| case009_guarded_008 | FEATURE_STARVATION | False | 50.3 | 14.45 | 0.3669 | 0.2464 | 6.992 | 142.9 | 0.8894 | 0 | 118.4 | 14 | 101.5 | 14 | 0.348 | 0.006 |
| case009_guarded_009 | FEATURE_STARVATION | False | 7.391 | 1.975 | 0.09281 | 0.2562 | 2.302 | 142.9 | 0.9131 | 0 | 119.6 | 11 | 105.9 | 7 | 0.326 | 0.01 |
| case009_guarded_010 | FEATURE_STARVATION | False | 370.3 | 159.1 | 1.375 | 0.2508 | 31.43 | 142.9 | 0.9092 | 0 | 118.4 | 15 | 84.1 | 15 | 0.054 | 0.004 |
| case009_guarded_011 | FEATURE_STARVATION | False | 2267 | 1748 | 4.126 | 0.2019 | 138.6 | 142.9 | 0.6057 | 0 | 117.5 | 13 | 34.6 | 13 | 0.055 | 0.003 |
| case009_guarded_012 | FEATURE_STARVATION | False | 15.12 | 4.271 | 0.1235 | 0.2594 | 2.987 | 142.9 | 0.8305 | 0 | 117.5 | 15 | 104.3 | 12 | 0.558 | 0.018 |
| case009_guarded_013 | FEATURE_STARVATION | False | 191.5 | 79.52 | 0.7499 | 0.2565 | 17.1 | 142.9 | 0.8603 | 0 | 117.7 | 10 | 85.55 | 10 | 0.248 | 0.006 |
| case009_guarded_014 | SUCCESS | True | 0.6945 | 0.4093 | 0.01138 | 0.2959 | 1.137 | 142.9 | 0.8999 | 0 | 118.9 | 21 | 103.2 | 21 | 0.01 | 0.004 |
| case009_guarded_015 | PATH_LENGTH_BAD | False | 1.28 | 0.8335 | 0.01259 | 0.2714 | 1.2 | 142.9 | 0.9149 | 0 | 116.3 | 28 | None | 28 | 0.016 | 0.005 |
| case009_guarded_016 | FEATURE_STARVATION | False | 1319 | 874 | 2.748 | 0.1896 | 85.52 | 142.9 | 0.6619 | 0 | 120.6 | 10 | 53.5 | 10 | 0.501 | 0.009 |
| case009_guarded_017 | FEATURE_STARVATION | False | 84.46 | 28.88 | 0.4671 | 0.2792 | 9.474 | 142.9 | 0.8225 | 0 | 118.7 | 9 | 91.4 | 9 | 0.081 | 0.003 |
| case009_guarded_018 | FEATURE_STARVATION | False | 41.95 | 11.61 | 0.3344 | 0.2574 | 6.314 | 142.9 | 0.8661 | 1e-06 | 118.3 | 15 | 100.8 | 15 | 0.116 | 0.007 |
| case009_guarded_019 | FEATURE_STARVATION | False | 107.1 | 35.72 | 0.6186 | 0.3023 | 11.95 | 142.9 | 0.8754 | 0 | 116 | 14 | 92.75 | 13 | 0.087 | 0.006 |
| case009_guarded_020 | SUCCESS | True | 0.4436 | 0.3833 | 0.009789 | 0.3269 | 1.089 | 142.9 | 0.9209 | 0 | 117.6 | 35 | 102.3 | 35 | 0.017 | 0.006 |

## Guarded Validation
- first_usable_case: `case009_guarded_001`
- first_usable_repeats: 0/3
- first_usable_validation_dir: `src/hno_vio/eval/V1_01_easy_case009_guarded_20260624T025617Z/validation_case009_guarded_001`
- best_case: `case009_guarded_020`
- best_case_usable_repeats: 0/3
- best_case_validation_dir: `src/hno_vio/eval/V1_01_easy_case009_guarded_20260624T025617Z/validation_case009_guarded_020`

## Conclusion
前50轮次测试结果表明，继续单纯收紧 update gate/低特征保护已经不能解决稳定性问题。case009 附近仍然是“单次可用、重复不稳定”。下一步应该进入 51-100 轮结构调整，重点放在 landmark/map policy，而不是继续围绕 gate 小调参。当前新日志已经能支撑后续结构搜索定位首次失控点。

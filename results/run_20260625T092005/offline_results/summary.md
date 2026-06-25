# RTAB-Map Offline Evaluation

Raw odometry: `/home/sharpa/hno_vio_clean/src/hno_vio/results/run_20260625T092005/vio_results/odom_raw.txt`.
Optimized odometry: `/home/sharpa/hno_vio_clean/src/hno_vio/results/run_20260625T092005/offline_results/odom_optimized.txt`.
Ground truth: `/home/sharpa/hno_vio_clean/src/hno_vio/ground_truth/euroc_mav/V1_01_easy.txt`.

| metric | raw | optimized |
| --- | ---: | ---: |
| ATE RMSE [m] | 0.646145 | 0.402959 |
| ATE mean [m] | 0.575495 | 0.250151 |
| ATE median [m] | 0.535357 | 0.112172 |
| RPE trans RMSE @20 frames (~1s) [m] | 0.134655 | 0.711987 |
| RPE rot RMSE @20 frames (~1s) [deg] | 1.568192 | 12.009329 |
| path length [m] | 61.488284 | 53.199574 |
| optimized/raw path length ratio | 1.000000 | 0.865199 |
| pose count | 2290 | 108 |
| duration [s] | 142.850000 | 142.850000 |
| final raw-vs-optimized position delta [m] | 0.024829 | 0.024829 |
| raw-vs-optimized mean position delta [m] | 3.313446749 | 3.313446749 |
| raw-vs-optimized max position delta [m] | 5.698921907 | 5.698921907 |
| optimized graph pose count | 108 | 108 |

Notes:
- Evaluation uses `evo_ape/evo_rpe --align --correct_scale` for shape comparison.
- `odom_optimized.txt` is exported from `/rtabmap/mapData.graph.poses`.
- The primary RTAB-Map odometry input is TF `odom -> base_link`; `/hno_vio/odom` is recorded for export/debugging.
- RPY plot and CSV apply +-180 degree unwrap correction before visualization.

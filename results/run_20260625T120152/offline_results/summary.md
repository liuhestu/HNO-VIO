# RTAB-Map Offline Evaluation

Raw odometry: `/home/sharpa/hno_vio_clean/src/hno_vio/results/run_20260625T120152/vio_results/odom_raw.txt`.
Optimized odometry: `/home/sharpa/hno_vio_clean/src/hno_vio/results/run_20260625T120152/offline_results/odom_optimized.txt`.
Ground truth: `/home/sharpa/hno_vio_clean/src/hno_vio/ground_truth/euroc_mav/V1_01_easy.txt`.

| metric | raw | optimized |
| --- | ---: | ---: |
| ATE RMSE [m] | 0.392425 | 0.236789 |
| ATE mean [m] | 0.340964 | 0.131592 |
| ATE median [m] | 0.268993 | 0.076402 |
| RPE trans RMSE @20 frames (~1s) [m] | 0.040445 | 0.619762 |
| RPE rot RMSE @20 frames (~1s) [deg] | 0.706970 | 9.634599 |
| path length [m] | 60.894471 | 52.021598 |
| optimized/raw path length ratio | 1.000000 | 0.854291 |
| pose count | 2293 | 106 |
| duration [s] | 142.850000 | 142.850000 |
| final raw-vs-optimized position delta [m] | 0.001661 | 0.001661 |
| raw-vs-optimized mean position delta [m] | 2.359265415 | 2.359265415 |
| raw-vs-optimized max position delta [m] | 4.506796658 | 4.506796658 |
| optimized graph pose count | 106 | 106 |

Notes:
- Evaluation uses `evo_ape/evo_rpe --align --correct_scale` for shape comparison.
- `odom_optimized.txt` is exported from `/rtabmap/mapData.graph.poses`.
- The primary RTAB-Map odometry input is TF `odom -> base_link`; `/hno_vio/odom` is recorded for export/debugging.
- RPY plot and CSV unwrap each curve and place optimized/GT on the nearest 360-degree branch of raw before visualization.

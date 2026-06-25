# HNO-VIO Saved Odom + RTAB-Map Offline Pipeline

Run RTAB-Map offline from one HNO-VIO run saved under `results/run_*/`.

Input layout:

```text
results/run_YYYYmmddTHHMMSS/
  run_context.json
  vio_results/
    odom_raw.csv
    odom_raw.txt
```

`run_context.json` is the source of dataset context, including `euroc_mav0`,
`ground_truth_tum`, `odom_frame`, and `base_frame`.

Run:

```bash
cd /home/sharpa/hno_vio_clean
src/hno_vio/tools/run_rtabmap/scripts/run_rtabmap.sh \
  src/hno_vio/results/run_20260625T092005/vio_results/odom_raw.csv
```

The script also accepts the `vio_results/` directory:

```bash
src/hno_vio/tools/run_rtabmap/scripts/run_rtabmap.sh \
  src/hno_vio/results/run_20260625T092005/vio_results
```

Output layout:

```text
results/run_YYYYmmddTHHMMSS/offline_results/
  rtabmap_input.bag/
  rtabmap_output.bag/
  rtabmap.db

  odom_optimized.txt

  evo_ape_raw.txt
  evo_rpe_raw.txt
  evo_ape_optimized.txt
  evo_rpe_optimized.txt

  evo_ape_optimized.pdf
  evo_rpe_trans_optimized.pdf
  evo_rpe_rot_optimized.pdf
  evo_traj_gt_raw_optimized.pdf

  summary.md
```

`odom_optimized.txt` is exported from `/rtabmap/mapData.graph.poses`. If the graph
contains fewer than 20 poses the run fails; 20-49 poses is treated as a warning.

## RTAB-Map Density Tuning

The optimized trajectory is only as dense as the RTAB-Map graph. The default
pipeline uses a denser graph than RTAB-Map's 1 Hz default to reduce visibly
piecewise-linear optimized curves:

```text
RTABMAP_DETECTION_RATE=5
RTABMAP_LINEAR_UPDATE=0.03
RTABMAP_ANGULAR_UPDATE=0.03
RTABMAP_CREATE_INTERMEDIATE_NODES=true
```

Override them per run when needed:

```bash
RTABMAP_DETECTION_RATE=10 RTABMAP_LINEAR_UPDATE=0.02 \
src/hno_vio/tools/run_rtabmap/scripts/run_rtabmap.sh \
  src/hno_vio/results/run_YYYYmmddTHHMMSS/vio_results/odom_raw.csv
```

Check `offline_results/logs/export_report.txt` after a run. If
`graph_final_mean_hz` is still low or `graph_final_max_step_m` is large, raise
`RTABMAP_DETECTION_RATE` or lower the update thresholds.

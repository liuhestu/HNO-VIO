# HNO-VIO RTAB-Map Backend

This directory contains the ROS2 RTAB-Map backend tools for one HNO-VIO run.

`hno_vio.launch.py run_preprocess:=true` must generate the input bag first:

```text
results/run_YYYYmmddTHHMMSS/
  run_context.json
  vio_results/
    odom_raw.csv
    odom_raw.txt
    rtabmap_input_db3/
      metadata.yaml
      *.db3
```

Run the backend:

```bash
cd /home/sharpa/hno_vio_clean
src/hno_vio/tools/run_rtabmap/hno_rtabmap.sh \
  src/hno_vio/results/run_YYYYmmddTHHMMSS/vio_results/rtabmap_input_db3
```

This is a one-command pipeline. It starts stereo synchronization and
RTAB-Map, records the output bag, plays the complete input bag, exports the
optimized graph trajectory, and runs evo. No keyboard interaction is needed.
For EuRoC `V1_01_easy`, leave the command running for several minutes until
it prints `completed:`.

Output:

```text
results/run_YYYYmmddTHHMMSS/
  offline_results/
    rtabmap_output.bag/
    rtabmap.db
    odom_optimized.txt
    logs/
  evo_results/
    ape_raw.zip
    ape_optimized.zip
    ate_plot.pdf
    ate_stats.txt
    traj_trajectories.png
    traj_xyz.png
    traj_rpy.png
    traj_speeds.png
```

`odom_optimized.txt` is exported from `/rtabmap/mapData.graph.poses`.
`rtabmap.db` is kept for database viewer and graph debugging.

Re-run only the EVO analysis for an existing run:

```bash
cd /home/sharpa/hno_vio_clean/src/hno_vio
python3 tools/run_rtabmap/eval_and_analysis.py \
  results/run_YYYYmmddTHHMMSS
```

The analysis recreates `evo_results/` and uses the exact ground-truth path
recorded in `run_context.json`.

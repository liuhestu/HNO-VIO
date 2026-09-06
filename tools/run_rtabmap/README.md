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
cd /home/he/hno_vio_ws
src/hno_vio/tools/run_rtabmap/hno_rtabmap.sh \
  src/hno_vio/results/run_YYYYmmddTHHMMSS
```

This is a one-command pipeline. It starts stereo synchronization and
RTAB-Map, records the output bag, plays the complete input bag, exports the
optimized graph trajectory, and runs evo. No keyboard interaction is needed.
The script automatically uses EVO from `/home/he/.venvs/hnovio`; set
`HNO_VIO_VENV` if that environment is stored elsewhere. ROS Python tools
continue to use `/usr/bin/python3`.
For EuRoC `V1_01_easy`, leave the command running for several minutes until
it prints `completed:`.

The backend runs in an isolated ROS domain and permits only one script
instance. Stereo images are not duplicated. The output bag contains the
lightweight map path and exactly one final MapData fetched through GetMap2
without image or vocabulary payloads, so it can be replayed in RViz without
restoring continuous high-memory MapData recording.

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

`odom_optimized.txt` is exported from the final optimized
`GetMap2.data.graph.poses`.
`rtabmap.db` is kept for database viewer and graph debugging.

Convert the database and trajectories to a Rerun recording:

```bash
cd /home/he/hno_vio_ws/src/hno_vio
tools/visual_rerun/visual_rerun.sh \
  results/run_YYYYmmddTHHMMSS
```

This writes `visual_results/rtabmap.rrd` under the same run directory and
opens it in the Rerun Viewer. See `tools/visual_rerun/README.md` for headless
and point-cloud resolution options.

Re-run only the EVO analysis for an existing run:

```bash
cd /home/he/hno_vio_ws/src/hno_vio
hnovio
python3 tools/run_rtabmap/eval_and_analysis.py \
  results/run_YYYYmmddTHHMMSS
```

The analysis recreates `evo_results/` and uses the exact ground-truth path
recorded in `run_context.json`.

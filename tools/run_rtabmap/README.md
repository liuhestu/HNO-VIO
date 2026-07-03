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

Output:

```text
results/run_YYYYmmddTHHMMSS/offline_results/
  rtabmap_output.bag/
  rtabmap.db
  odom_optimized.txt
  summary.md
  logs/
```

`odom_optimized.txt` is exported from `/rtabmap/mapData.graph.poses`.
`rtabmap.db` is kept for database viewer and graph debugging.

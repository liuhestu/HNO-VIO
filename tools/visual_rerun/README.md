# RTAB-Map Rerun Visualization

Convert one completed HNO-VIO RTAB-Map run into a self-contained Rerun
recording containing the assembled color point cloud, aligned ground-truth,
raw and optimized trajectories, and timestamped pose markers.

The run must already contain:

```text
results/run_YYYYmmddTHHMMSS/
  run_context.json
  vio_results/odom_raw.txt
  offline_results/rtabmap.db
```

Generate and open the recording:

```bash
cd /home/he/hno_vio_ws/src/hno_vio
tools/visual_rerun/visual_rerun.sh results/run_YYYYmmddTHHMMSS
```

The default output is:

```text
results/run_YYYYmmddTHHMMSS/visual_results/rtabmap.rrd
```

Use `--no-open` for headless conversion, `--voxel METERS` to change the
default 5 cm cloud resolution, `--point-radius METERS` to change the rendered
point size, or `--output PATH` to override only the RRD location. The default
point radius is 2.5 cm, which keeps the scene compact while making surfaces
easier to see. `conversion.log` always remains under the run's `visual_results/`
directory. Set `HNO_VIO_VENV` if the Python environment is not located at
`/home/he/.venvs/hnovio`.

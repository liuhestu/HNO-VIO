# RTAB-Map Rerun Visualization

Convert one completed HNO-VIO RTAB-Map run into a self-contained Rerun
recording containing the assembled color point cloud, aligned ground-truth,
raw and optimized trajectories, synchronized rectified camera images, a
progressively growing optimized path and map, and roll/pitch/yaw plots. Because
the assembled PLY does not retain first-observation timestamps, map points are
assigned to their nearest optimized keyframe for playback.

The run must already contain:

```text
results/run_YYYYmmddTHHMMSS/
  run_context.json
  vio_results/odom_raw.txt
  vio_results/rtabmap_input_db3/
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

The default dashboard emphasizes the growing optimized path while also showing
thin raw and ground-truth paths for comparison. Orientation angles are
unwrapped so that crossing +/-180 degrees does not add plot discontinuities.

Use `--no-open` for headless conversion, `--voxel METERS` to change the
default 3 cm cloud resolution, `--point-radius METERS` to change the default
6 mm point radius, or `--output PATH` to override only the RRD location. Camera
images are sampled at up to 10 Hz and JPEG-compressed by default. Point-cloud
updates are grouped at 2 Hz to keep playback responsive. Use `--image-rate HZ`,
`--map-rate HZ`, `--jpeg-quality 1..100`, or `--no-images` to adjust them.
`conversion.log` always remains under the run's `visual_results/` directory.
Set `HNO_VIO_VENV` if the Python environment is not located at
`/home/he/.venvs/hnovio`.

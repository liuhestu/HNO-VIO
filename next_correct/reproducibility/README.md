# Reproducibility records

Each local run under `../data/` contains:

- `manifest.json`: condition, launch command, source state, binary SHA-256,
  start/end time, exit status, row count, and output checksums.
- `launch.log`: complete ROS launch output.
- `vio_results/odom_raw.csv` and `.txt`: unchanged trajectory export.
- `vio_results/e_diagnostics.csv`: one row per committed camera state.
- `run_context.json`: ROS-side paths and experiment switches.

The matrix runner refuses to overwrite an existing run directory. Use
`--resume` to skip completed runs; move failed directories aside before a
deliberate retry so the original failure remains auditable.

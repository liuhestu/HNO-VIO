# run_vio

This directory is now a minimal ROS2 smoke entry point for HNO-VIO.

The old ROS1 auto-converge scripts were removed because the current
converged parameter set lives in `config/euroc_mav/estimator_config.yaml`
and normal runs should use:

```bash
ros2 launch hno_vio hno_vio.launch.py
```

Build smoke:

```bash
src/hno_vio/tools/run_vio/run_smoke.sh
```

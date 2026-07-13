# run_vio

This directory contains the automated ROS2 regression smoke entry point for
HNO-VIO.

The old ROS1 auto-converge scripts were removed because the current
converged parameter set lives in `config/euroc_mav/estimator_config.yaml`
and normal runs should use:

```bash
ros2 launch hno_vio hno_vio.launch.py
```

Full smoke (build, GT mapping regression, and up to three ordinary VIO runs):

```bash
src/hno_vio/tools/run_vio/run_smoke.sh
```

The GT mapping run must pass its accuracy and trajectory-completeness checks
before ordinary VIO starts. Ordinary VIO passes when one of at most three runs
is complete, non-divergent, and stays within the configured mean/RMSE limits;
remaining slots are recorded as `SKIPPED`.

# HNO-VIO Saved Odom + RTAB-Map Offline Pipeline

This tool replays one selected successful HNO-VIO odometry CSV with EuRoC stereo
images into RTAB-Map ROS2, exports raw and optimized trajectories, and evaluates
them against EuRoC ground truth.

Default selected case:

```text
/home/sharpa/hno_vio_clean/src/hno_vio/success_odom/case009_guarded_020
```

Run:

```bash
cd /home/sharpa/hno_vio_clean/src/hno_vio/tools/run_rtabmap/scripts
./run_all_case009.sh
```

Results are written to:

```text
/home/sharpa/hno_vio_clean/src/hno_vio/success_odom/case009_guarded_020/offline_results
```

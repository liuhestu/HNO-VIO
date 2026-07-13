# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 Humble `ament_cmake` package for HNO visual-inertial odometry. Core C++ code lives in `src/`, with public headers in `include/hno_vio/`. The reconstructed architecture is split into `frontend/` for tracking and landmark health, `observer/` for propagation and visual/ZUPT updates, `pipeline/` for sensor buffering and estimator orchestration, and `ros/` for node, publishing, GT, and export integration. `Diagnostics.cpp` owns periodic runtime logging; algorithm modules populate diagnostic snapshots instead of formatting recurring logs themselves.

Launch entry points are in `launch/`, and runtime calibration/estimator settings are split under `config/euroc_mav/` and `config/realsense/`. RTAB-Map and evaluation utilities are in `tools/run_rtabmap/`; the full VIO regression entry point is in `tools/run_vio/`. `thirdparty/openvins_core/` contains vendored OpenVINS-derived support code. Ground-truth trajectories are stored in `ground_truth/euroc_mav/`. Generated experiment output belongs under `results/` or `smoke_results/` and should not be treated as source.

## Build, Test, and Development Commands

From the workspace root, build only this package:

```bash
colcon build --packages-select hno_vio
```

From this package directory, run the full smoke regression:

```bash
tools/run_vio/run_smoke.sh
```

After building and sourcing the workspace, run the main launch file:

```bash
ros2 launch hno_vio hno_vio.launch.py
```

For a VIO-only run without RTAB-Map recording or RViz:

```bash
ros2 launch hno_vio hno_vio.launch.py run_preprocess:=false rviz:=false
```

For RTAB-Map postprocessing, first generate a `results/run_*/vio_results/rtabmap_input_db3` bag, then run:

```bash
tools/run_rtabmap/hno_rtabmap.sh results/run_YYYYmmddTHHMMSS/vio_results/rtabmap_input_db3
```

## Coding Style & Naming Conventions

Use C++17 for package code. Match the existing style: four-space indentation, PascalCase class names such as `VioPipeline` and `FeatureManager`, and descriptive snake_case ROS parameters and Python variables. Preserve the existing `ZUPT_print` capitalization because it is a public launch/ROS parameter. Keep ROS-facing names stable unless migration notes and launch/config updates are included. Prefer small, focused changes in `src/` and `include/`; avoid editing `thirdparty/` except for deliberate vendor fixes.

## Testing Guidelines

There is no dedicated unit-test suite in this package. Validate C++ changes with `colcon build --packages-select hno_vio`. Run `python3 -m py_compile launch/hno_vio.launch.py` for launch edits and `bash -n tools/run_vio/run_smoke.sh` for smoke-script edits. The full smoke first validates GT mapping, then permits up to three ordinary VIO attempts; do not weaken its accuracy or trajectory-completeness criteria to make a change pass. For launch, configuration, preprocessing, or evaluation changes, run the relevant `ros2 launch` path and, when applicable, rerun `tools/run_rtabmap/eval_and_analysis.py results/run_YYYYmmddTHHMMSS`. Record the dataset, bag path, and key output files in your notes.

## Commit & Pull Request Guidelines

Recent commits use short imperative summaries, often in Chinese, for example `整理评估脚本` or `优化rtabmap占用`. Keep the first line concise and focused on the behavioral change. Pull requests should describe the affected VIO/RTAB-Map path, list build or run commands executed, mention datasets used, and call out changes to calibration, launch arguments, or output formats.

## Configuration & Data Notes

Use estimator YAML files for numeric tuning and launch arguments for behavior switches. Parameter precedence is C++ fallback, then YAML numeric value, then an explicit ROS parameter override. In the Euroc estimator config, `relative_config_imu` is a compatibility entry and is not currently consumed by the HNO pipeline; IMU noise retains the existing C++ fallback and parser behavior unless a task explicitly changes that contract. Keep Realsense and Euroc calibration/config changes isolated to the correct dataset family.

Diagnostics are throttled to every 30 committed camera frames. Defaults are `essential_print:=true`, `frontend_print:=false`, `updater_print:=false`, `ZUPT_print:=false`, and `pipeline_print:=false`. One-time anomaly messages remain unconditional. Keep RViz launch-managed: `rviz:=true` starts it as a ROS launch `Node`, and `rviz:=false` does not start it. Do not detach RViz or other launch children unless explicitly requested.

Do not commit new run artifacts from `results/` or `smoke_results/` unless explicitly required. When adding launch parameters, update defaults in `launch/hno_vio.launch.py` and document any required bag topics or frame names.

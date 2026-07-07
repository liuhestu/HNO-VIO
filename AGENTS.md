# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 Humble `ament_cmake` package for HNO visual-inertial odometry. Core C++ code lives in `src/`, with public headers in `include/hno_vio/`. Launch entry points are in `launch/`, and runtime calibration/estimator settings are split under `config/euroc_mav/` and `config/realsense/`. RTAB-Map and evaluation utilities are in `tools/run_rtabmap/`; VIO smoke build helpers are in `tools/run_vio/`. `thirdparty/openvins_core/` contains vendored OpenVINS-derived support code. Ground-truth trajectories are stored in `ground_truth/euroc_mav/`. Generated experiment output belongs under `results/` and should not be treated as source.

## Build, Test, and Development Commands

From the workspace root, build only this package:

```bash
colcon build --packages-select hno_vio
```

From this package directory, run the provided smoke build:

```bash
tools/run_vio/run_smoke.sh
```

After building and sourcing the workspace, run the main launch file:

```bash
ros2 launch hno_vio hno_vio.launch.py
```

For RTAB-Map postprocessing, first generate a `results/run_*/vio_results/rtabmap_input_db3` bag, then run:

```bash
tools/run_rtabmap/hno_rtabmap.sh results/run_YYYYmmddTHHMMSS/vio_results/rtabmap_input_db3
```

## Coding Style & Naming Conventions

Use C++17 for package code. Match the existing style: four-space indentation, PascalCase class names such as `HNOManager`, and descriptive snake_case ROS parameters and Python variables. Keep ROS-facing names stable unless migration notes and launch/config updates are included. Prefer small, focused changes in `src/` and `include/`; avoid editing `thirdparty/` except for deliberate vendor fixes.

## Testing Guidelines

There is no dedicated unit-test suite in this package. Validate C++ changes with `colcon build --packages-select hno_vio` or `tools/run_vio/run_smoke.sh`. For launch, configuration, preprocessing, or evaluation changes, run the relevant `ros2 launch` path and, when applicable, rerun `tools/run_rtabmap/eval_and_analysis.py results/run_YYYYmmddTHHMMSS`. Record the dataset, bag path, and key output files in your notes.

## Commit & Pull Request Guidelines

Recent commits use short imperative summaries, often in Chinese, for example `整理评估脚本` or `优化rtabmap占用`. Keep the first line concise and focused on the behavioral change. Pull requests should describe the affected VIO/RTAB-Map path, list build or run commands executed, mention datasets used, and call out changes to calibration, launch arguments, or output formats.

## Configuration & Data Notes

Do not commit large new run artifacts from `results/` unless explicitly required. Keep calibration files paired with the correct dataset family. When adding launch parameters, update defaults in `launch/hno_vio.launch.py` and document any required bag topics or frame names.

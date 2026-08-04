# `e` transient experiment

`REPORT.md` is the primary result entry point. The experiment compares normal
`e`, oracle fixed-`e`, forced `sigma_R=0`, and a 900-frame no-projection
diagnostic on EuRoC V1_02.

## Build and validate instrumentation

From the workspace root:

```bash
colcon build --packages-select hno_vio
```

From this package directory:

```bash
next_correct/scripts/run_experiment.sh --mode validation
```

Validation runs exactly 100 committed frames for every condition. Inspect each
`manifest.json` and `vio_results/e_diagnostics.csv` before starting the formal
matrix.

## Run the formal matrix

```bash
next_correct/scripts/run_experiment.sh --mode formal
```

The runner uses the prescribed rotated order. The three no-projection runs stop
at exactly 900 committed frames; the other nine consume the complete bag. If a
completed matrix is interrupted only between runs, restart with:

```bash
next_correct/scripts/run_experiment.sh --mode formal --resume
```

## Analyze

```bash
next_correct/scripts/analyze_experiment.sh
```

The analysis reads EuRoC's provided velocity columns when available, performs a
single first-pose SE(3) alignment, detects sustained events, and computes lag
association from 0.5-second error increments rather than raw rising curves.
It regenerates `REPORT.md`, `summary/`, and `figures/`.

Raw runs are intentionally ignored by Git. Do not copy conclusions from earlier
parity, GT-mapping, convergence-basin, or tuning experiments into this report.

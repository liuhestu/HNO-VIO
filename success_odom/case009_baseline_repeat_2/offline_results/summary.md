# RTAB-Map Offline Evaluation

Selected run: `case009_guarded_020`.

| metric | raw | optimized |
| --- | ---: | ---: |
| ATE RMSE [m] | 0.554646 | 0.554646 |
| ATE mean [m] | 0.497843 | 0.497843 |
| ATE median [m] | 0.435464 | 0.435464 |
| RPE trans RMSE @20 frames (~1s) [m] | 0.056389 | 0.056389 |
| RPE rot RMSE @20 frames (~1s) [deg] | 0.802792 | 0.802791 |
| path length [m] | 61.986036 | 61.986029 |
| optimized/raw path length ratio | 1.000000 | 1.000000 |
| pose count | 2760 | 2760 |
| duration [s] | 142.850000 | 142.850000 |
| final raw-vs-optimized position delta [m] | 0.000000 | 0.000000 |
| raw-vs-optimized mean position delta [m] | 0.000000478 | 0.000000478 |
| raw-vs-optimized max position delta [m] | 0.000003701 | 0.000003701 |

Graph-final trajectory from `mapData.graph.poses`:

| metric | graph_final |
| --- | ---: |
| ATE RMSE [m] | 0.060055 |
| ATE mean [m] | 0.053425 |
| ATE median [m] | 0.043171 |
| pose count | 121 |

Notes:
- Evaluation uses `evo_ape/evo_rpe --align --correct_scale` for shape comparison.
- The trajectory PDF applies the same kind of Sim(3) alignment before plotting against GT.
- `rtabmap_optimized.tum` is exported through `map->odom`; `rtabmap_graph_final.tum` is exported directly from the optimized RTAB-Map graph.
- In this run, RTAB-Map did not publish a meaningful `map->odom` correction, but the final optimized graph trajectory is non-identity.
- The primary RTAB-Map odometry input is TF `odom -> base_link`; `/hno_vio/odom` is recorded for export/debugging.
- RPY plot and CSV apply +-180 degree unwrap correction before visualization.

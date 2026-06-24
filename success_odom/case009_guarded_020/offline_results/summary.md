# RTAB-Map Offline Evaluation

Selected run: `case009_guarded_020`.

| metric | raw | optimized |
| --- | ---: | ---: |
| ATE RMSE [m] | 0.424819 | 0.424819 |
| ATE mean [m] | 0.375884 | 0.375884 |
| ATE median [m] | 0.334094 | 0.334094 |
| RPE trans RMSE @20 frames (~1s) [m] | 0.069124 | 0.069124 |
| RPE rot RMSE @20 frames (~1s) [deg] | 0.914845 | 0.914847 |
| path length [m] | 61.847874 | 61.847874 |
| optimized/raw path length ratio | 1.000000 | 1.000000 |
| pose count | 2619 | 2619 |
| duration [s] | 142.850000 | 142.850000 |
| final raw-vs-optimized position delta [m] | 0.000000 | 0.000000 |
| raw-vs-optimized mean position delta [m] | 0.000000432 | 0.000000432 |
| raw-vs-optimized max position delta [m] | 0.000002926 | 0.000002926 |

Graph-final trajectory from `mapData.graph.poses`:

| metric | graph_final |
| --- | ---: |
| ATE RMSE [m] | 0.094988 |
| ATE mean [m] | 0.089102 |
| ATE median [m] | 0.093145 |
| pose count | 118 |

Notes:
- Evaluation uses `evo_ape/evo_rpe --align --correct_scale` for shape comparison.
- The trajectory PDF applies the same kind of Sim(3) alignment before plotting against GT.
- `rtabmap_optimized.tum` is exported through `map->odom`; `rtabmap_graph_final.tum` is exported directly from the optimized RTAB-Map graph.
- In this run, RTAB-Map did not publish a meaningful `map->odom` correction, but the final optimized graph trajectory is non-identity.
- The primary RTAB-Map odometry input is TF `odom -> base_link`; `/hno_vio/odom` is recorded for export/debugging.
- RPY plot and CSV apply +-180 degree unwrap correction before visualization.

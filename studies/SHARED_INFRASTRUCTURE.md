# Shared Infrastructure

The `lpb_ws/src` tree is the shared ROS workspace used by both the Preliminary Study and the Full Study. The study folders under `studies/` are documentation-oriented; they do not duplicate or move ROS packages, maps, worlds, local planner source, or local planner parameter files.

## Evaluated Local Planner Packages

All five evaluated local planner implementations required by the benchmark are repository-contained:

| Evaluated local planner | Package | Plugin |
| --- | --- | --- |
| DWA | `dwa_local_planner` | `dwa_local_planner/DWAPlannerROS` |
| Trajectory Rollout | `base_local_planner` | `base_local_planner/TrajectoryPlannerROS` |
| TEB | `teb_local_planner` | `teb_local_planner/TebLocalPlannerROS` |
| Neo Local Planner | `neo_local_planner` | `neo_local_planner/NeoLocalPlanner` |
| Hybrid Local Planner | `hybrid_local_planner` | `hybrid_local_planner/HybridPlannerROS` |

## Shared ROS Navigation Infrastructure

The active study chain runs through `move_base`, which loads a global planner and one selected local planner plugin for each matrix case.

Directly active navigation packages include:

- `move_base`: coordinates global planning, local planning, recovery behavior, and action execution.
- `costmap_2d`: provides global and local costmaps for obstacle-aware navigation.
- `global_planner`: provides the global planner used by the study launch files.
- `nav_core`: provides the ROS plugin interfaces used by the global planner and local planner plugins.
- `amcl`: provides localization for the TurtleBot3 navigation setup.
- `map_server`: loads the study map files used by the navigation stack.

Related ROS Navigation source packages are retained as part of the coherent repository-contained source snapshot and build context:

- `navfn`
- `base_local_planner`
- `carrot_planner`
- `clear_costmap_recovery`
- `dwa_local_planner`
- `fake_localization`
- `move_slow_and_clear`
- `navigation`
- `rotate_recovery`
- `voxel_grid`

## Robot And Simulation Infrastructure

The TurtleBot3 packages provide robot description, simulation assets, messages, Gazebo integration, and navigation launch support:

- `turtlebot3`
- `turtlebot3_msgs`
- `turtlebot3_simulations`

## Experiment-Specific Packages

- `test`: experiment launch files, maps, worlds, dynamic obstacle nodes, metrics scripts, archived result folders, and matrix runners.
- `my_nav_cfg`: local planner parameter YAML files for the `original` and `same` configuration modes.

## Canonical Runtime Entry Points

- Preliminary Study: `lpb_ws/src/test/run_matrix_ps.sh`
- Full Study: `lpb_ws/src/test/run_matrix_fs.sh`

## Study Launch And Metrics Chains

- Preliminary Study: `all_Tests_ps.launch` -> `nav_measurements_ps.py`
- Full Study: `all_Tests_fs.launch` -> `nav_measurements_fs.py`

## Result Paths

- Preliminary Study raw results: `lpb_ws/src/test/docs_tables_media/preliminary_study/raw_results/`
- Full Study raw results: `lpb_ws/src/test/docs_tables_media/full_study/raw_results/`
- Full Study processed results: `lpb_ws/src/test/docs_tables_media/full_study/editted_results/`

## Generated Paths

Generated paths must not be committed:

- `lpb_ws/build/`
- `lpb_ws/devel/`
- Python `__pycache__/`
- `lpb_ws/src/test/worlds/build/`

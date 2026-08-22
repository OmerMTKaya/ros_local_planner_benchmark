# Preliminary Study

The Preliminary Study is the baseline comparison in this repository. It evaluates five ROS-based local planners under a restricted scope before the larger Full Study.

## Runtime Chain

```text
lpb_ws/src/test/run_matrix_ps.sh
  -> lpb_ws/src/test/launch/all_Tests_ps.launch
  -> lpb_ws/src/test/scripts/nav_measurements_ps.py
```

Here `ps` means Preliminary Study.

## Local Planners

- DWA: `dwa_local_planner/DWAPlannerROS`
- TEB: `teb_local_planner/TebLocalPlannerROS`
- Trajectory Rollout: `base_local_planner/TrajectoryPlannerROS`
- Neo Local Planner: `neo_local_planner/NeoLocalPlanner`
- Hybrid Local Planner: `hybrid_local_planner/HybridPlannerROS`

## Experimental Design

- Configuration: `original`
- Environments: empty and static
- Runtime world identifiers: `bosOrt` and `duraganOrt3`
- Pose transitions: `5`
- Repetitions: `2`
- Total trials: `1 configuration x 2 environments x 5 local planners x 5 transitions x 2 repetitions = 100 trials`

## Recorded Outputs

Raw CSV files use this column contract:

```text
Trial
From (x,y)
To (x,y)
Navigation Time (s)
Average Speed (m/s)
Collision
Timeout
Success
```

Archived Preliminary Study raw outputs are stored at:

```text
lpb_ws/src/test/docs_tables_media/preliminary_study/raw_results/
```

## Collision And Timeout

The Preliminary Study records one unified `Collision` outcome. Collision detection is implemented in the Preliminary Study metrics script with a LiDAR-based mechanism using `/scan`.

Timeouts are path-length based using `0.22 m/s` as the TurtleBot3 Burger nominal translational velocity:

- empty environment: `3 x nominal traversal duration`
- static environment: `5 x nominal traversal duration`

## Execution

After installing dependencies and building the workspace:

```bash
cd lpb_ws
source devel/setup.bash
./src/test/run_matrix_ps.sh
```

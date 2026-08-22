# Full Study

The Full Study is the larger extension of the Preliminary Study. It evaluates the same five ROS-based local planners across two parameter-configuration modes and four environment categories.

## Runtime Chain

```text
lpb_ws/src/test/run_matrix_fs.sh
  -> lpb_ws/src/test/launch/all_Tests_fs.launch
  -> lpb_ws/src/test/scripts/nav_measurements_fs.py
```

Here `fs` means Full Study.

## Local Planners

- DWA: `dwa_local_planner/DWAPlannerROS`
- TEB: `teb_local_planner/TebLocalPlannerROS`
- Trajectory Rollout: `base_local_planner/TrajectoryPlannerROS`
- Neo Local Planner: `neo_local_planner/NeoLocalPlanner`
- Hybrid Local Planner: `hybrid_local_planner/HybridPlannerROS`

## Experimental Design

- Configurations: `original`, `same`
- Environments: empty, static, dynamic, mixed
- Runtime world identifiers: `bosOrt`, `duraganOrt`, `hareketliOrt`, `karmaOrt`
- Pose transitions: `10`
- Repetitions: `10`
- Total trials: `2 configurations x 4 environments x 5 local planners x 10 transitions x 10 repetitions = 4000 trials`

The `original` configuration preserves local-planner-specific parameter configurations used by the study. The `same` configuration standardizes shared motion limits and tolerances while preserving local-planner-specific internal parameters.

## Evaluation Indicators

The Full Study performance-reliability framework records:

- Average Navigation Time
- Average Deviation Time
- Average Executed Path Length
- Overall Average Speed
- Static Collision Rate
- Dynamic Collision Rate
- Local Planner Failure Rate
- Timeout Rate
- Successful Attempt Rate

Planned path length is recorded as an auxiliary nominal route descriptor where applicable.

## Termination And Reset Behavior

Each trial terminates as success, static collision, dynamic collision, timeout, or local planner failure. The runtime includes controlled reset/teleport behavior after terminal failure conditions so subsequent trials start from the intended pose.

## Results

Archived Full Study outputs are stored at:

```text
lpb_ws/src/test/docs_tables_media/full_study/raw_results/
lpb_ws/src/test/docs_tables_media/full_study/editted_results/
```

The historical spelling `editted_results` is preserved.

## Execution

After installing dependencies and building the workspace:

```bash
cd lpb_ws
source devel/setup.bash
./src/test/run_matrix_fs.sh
```

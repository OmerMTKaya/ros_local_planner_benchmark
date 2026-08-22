#!/usr/bin/env bash
set -euo pipefail

# ROS environment check.
if ! command -v rospack >/dev/null 2>&1; then
  echo "ERROR: rospack not found. Run in a ROS-sourced shell (source devel/setup.bash)." >&2
  exit 1
fi

# Matrix settings.
CONFIGS=(same original)
WORLDS=(bosOrt duraganOrt hareketliOrt karmaOrt)
PLANNERS=(dwa teb trajectory hybrid neo)
REPEATS=10
TIME_LIMIT="120m"

# Clean up stale Gazebo and RViz processes.
cleanup_stale() {
  pkill -f gzserver 2>/dev/null || true
  pkill -f gzclient 2>/dev/null || true
  pkill -f rviz     2>/dev/null || true
}
trap cleanup_stale EXIT

# Matrix loop.
for cfg in "${CONFIGS[@]}"; do
  for world in "${WORLDS[@]}"; do
    for planner in "${PLANNERS[@]}"; do
      echo ">>> RUNNING: config=$cfg | world=$world | planner=$planner"
      cleanup_stale

      set +e
      timeout --signal=SIGINT --kill-after=30s "$TIME_LIMIT" \
        roslaunch test all_Tests_fs.launch \
          config:=$cfg world:=$world planner:=$planner repeats:=$REPEATS \
          open_rviz:=false gazebo_gui:=false
      status=$?
      set -e

      if [[ $status -eq 124 ]]; then
        echo "TIMEOUT: $cfg $world $planner"
        exit 124
      elif [[ $status -ne 0 ]]; then
        echo "roslaunch failed: $cfg $world $planner (exit $status)"
        exit "$status"
      fi

      cleanup_stale
      sleep 1
    done
  done
done

echo "ALL TESTS COMPLETED SUCCESSFULLY"

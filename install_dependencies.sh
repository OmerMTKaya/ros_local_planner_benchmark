#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_SRC="$REPO_ROOT/lpb_ws/src"
ROS_SETUP="/opt/ros/noetic/setup.bash"

log() {
  printf '[deps] %s\n' "$*"
}

if [[ ! -d "$WORKSPACE_SRC" ]]; then
  printf 'ERROR: expected workspace source directory at %s\n' "$WORKSPACE_SRC" >&2
  exit 1
fi

if [[ ! -r /etc/os-release ]]; then
  printf 'ERROR: cannot determine operating system from /etc/os-release\n' >&2
  exit 1
fi

. /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "20.04" ]]; then
  printf 'ERROR: this repository is validated for Ubuntu 20.04 with ROS Noetic; detected %s %s\n' "${ID:-unknown}" "${VERSION_ID:-unknown}" >&2
  exit 1
fi

if [[ ! -f "$ROS_SETUP" ]]; then
  cat >&2 <<'EOF'
ERROR: ROS Noetic was not found at /opt/ros/noetic.
Install ROS Noetic first, then rerun this script.
EOF
  exit 1
fi

if ! command -v rosdep >/dev/null 2>&1; then
  log "Installing rosdep helper package"
  sudo apt-get update
  sudo apt-get install -y python3-rosdep
fi

source "$ROS_SETUP"

# Explicit prerequisites observed during clean-machine validation on
# Ubuntu 20.04.6 + ROS Noetic + Gazebo 11.15.1. Some are declared in
# upstream package.xml files but were not installed by rosdep on the
# validation machine; SuiteSparse is required by TEB's CMake checks.
EXTRA_APT_PACKAGES=(
  ros-noetic-rosserial-python
  ros-noetic-tf2-sensor-msgs
  ros-noetic-move-base-msgs
  ros-noetic-costmap-converter
  ros-noetic-mbf-costmap-core
  ros-noetic-mbf-msgs
  ros-noetic-libg2o
  libsuitesparse-dev
)

log "Installing explicit reproducibility prerequisites"
sudo apt-get update
sudo apt-get install -y "${EXTRA_APT_PACKAGES[@]}"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  log "Initializing rosdep"
  sudo rosdep init
else
  log "rosdep is already initialized"
fi

log "Updating rosdep database"
rosdep update

log "Installing remaining workspace dependencies declared in package.xml files"
rosdep install --from-paths "$WORKSPACE_SRC" --ignore-src -r -y --rosdistro noetic \
  --skip-keys rosserial_python

if command -v gazebo >/dev/null 2>&1; then
  log "Detected $(gazebo --version | head -n 1)"
else
  log "WARNING: gazebo command was not found after dependency installation"
fi

log "Dependency installation complete"

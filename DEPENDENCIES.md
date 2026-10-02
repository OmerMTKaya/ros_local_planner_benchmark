# Dependency And Reproducibility Notes

This repository has been validated on Ubuntu 20.04.6 LTS with ROS Noetic and Gazebo 11.15.1. Other Ubuntu 20.04 patch levels with ROS Noetic/Gazebo 11 should be compatible, but exact patch-version equivalence has not been established.

No additional local planner or project source repository needs to be cloned after cloning this repository.

## Base System Prerequisites

- Ubuntu 20.04
- ROS Noetic installed at `/opt/ros/noetic`
- Gazebo 11
- `catkin_make`
- `rosdep`
- `git` (when cloning from GitHub)
- `sudo` privileges for apt/rosdep installation

## Repository-Contained Source

The source packages needed to reproduce the validated local planner studies are included directly under `lpb_ws/src/`:

- ROS Navigation 1.17.3 packages: `amcl`, `base_local_planner`, `carrot_planner`, `clear_costmap_recovery`, `costmap_2d`, `dwa_local_planner`, `fake_localization`, `global_planner`, `map_server`, `move_base`, `move_slow_and_clear`, `nav_core`, `navfn`, `navigation`, `rotate_recovery`, `voxel_grid`.
- TEB Local Planner: `teb_local_planner` 0.9.1.
- Neo Local Planner: `neo_local_planner`.
- Hybrid Local Planner: `hybrid_local_planner`.
- TurtleBot3 source packages: `turtlebot3`, `turtlebot3_msgs`, `turtlebot3_simulations`.
- Study packages: `test`, `my_nav_cfg`.

## System/Python Dependencies Installed Through Apt/Rosdep

Run:

```bash
./install_dependencies.sh
```

The installer checks for Ubuntu 20.04 and `/opt/ros/noetic/setup.bash`, installs `python3-rosdep` if needed, installs the explicit clean-machine prerequisites listed below, initializes/updates `rosdep`, and then resolves the remaining package.xml dependencies.

Explicit prerequisites installed by the script:

- `ros-noetic-rosserial-python`
- `ros-noetic-tf2-sensor-msgs`
- `ros-noetic-move-base-msgs`
- `ros-noetic-costmap-converter`
- `ros-noetic-mbf-costmap-core`
- `ros-noetic-mbf-msgs`
- `ros-noetic-libg2o`
- `libsuitesparse-dev`

The ROS packages above were required during clean-machine validation even though several are already declared by retained upstream package manifests. `libsuitesparse-dev` is required by the retained TEB CMake configuration.

The installer then runs:

```bash
rosdep install --from-paths lpb_ws/src --ignore-src -r -y --rosdistro noetic --skip-keys rosserial_python
```

The skip key is used because this rosdep database does not define `rosserial_python` for Ubuntu 20.04, although the corresponding apt package is available as `ros-noetic-rosserial-python`.

The active experiment chains use ROS/system packages including:

- Gazebo ROS integration: `gazebo_ros`, `gazebo_msgs`
- Visualization/model description tools: `rviz`, `xacro`, `robot_state_publisher`
- ROS Python/runtime packages: `rospy`, `tf`, `tf2_ros`
- Message packages: `actionlib_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_srvs`
- Python analysis stack: `numpy`, `pandas`, `matplotlib`, `seaborn`, `opencv-python` via Ubuntu package `python3-opencv`

On the validated machine, active Python imports resolved to:

- `numpy 1.24.4`
- `pandas 2.0.3`
- `matplotlib 3.7.5`
- `seaborn 0.13.2`
- `cv2 4.2.0`

The repository declares Ubuntu/rosdep package keys instead of freezing the full local Python environment. The benchmark launch chain defaults the TurtleBot3 model to `burger`; setting `TURTLEBOT3_MODEL` is not required for the documented runs.

## Build And Run

```bash
source /opt/ros/noetic/setup.bash
cd lpb_ws
catkin_make
source devel/setup.bash
./src/test/run_matrix_ps.sh
```

For the Full Study matrix:

```bash
./src/test/run_matrix_fs.sh
```

Do not source another historical ROS workspace before building or running this repository.

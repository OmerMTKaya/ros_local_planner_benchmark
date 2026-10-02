# Source Provenance

This repository includes exact ROS source package copies used to build the portable workspace. The included ROS Navigation and TEB source snapshots correspond to the validated source releases used by the benchmark and were verified before inclusion in the portable workspace.

## Installed Package Integrity

Before source inclusion, `dpkg -V` reported no package-managed file differences for the validated installed ROS Navigation and TEB packages.

## ROS Navigation

- Source version: `1.17.3`
- Validated binary family: `1.17.3-1focal.*`
- Source packages included under `lpb_ws/src/`: `amcl`, `base_local_planner`, `carrot_planner`, `clear_costmap_recovery`, `costmap_2d`, `dwa_local_planner`, `fake_localization`, `global_planner`, `map_server`, `move_base`, `move_slow_and_clear`, `nav_core`, `navfn`, `navigation`, `rotate_recovery`, `voxel_grid`.
- Upstream project: ROS Navigation stack, commonly distributed from `https://github.com/ros-planning/navigation`.
- License declarations preserved in package sources: mostly BSD; `amcl` declares LGPL, and the `navigation` metapackage declares BSD/LGPL/LGPL (amcl).
- Local package modifications detected in installed `/opt/ros/noetic` package-managed files: none from `dpkg -V`.

Validated binary versions:

```text
ros-noetic-amcl 1.17.3-1focal.20250521.001518
ros-noetic-base-local-planner 1.17.3-1focal.20250521.005206
ros-noetic-carrot-planner 1.17.3-1focal.20250521.010103
ros-noetic-clear-costmap-recovery 1.17.3-1focal.20250521.005210
ros-noetic-costmap-2d 1.17.3-1focal.20250521.001646
ros-noetic-dwa-local-planner 1.17.3-1focal.20250521.005823
ros-noetic-fake-localization 1.17.3-1focal.20250520.001758
ros-noetic-global-planner 1.17.3-1focal.20250521.005728
ros-noetic-map-server 1.17.3-1focal.20250519.233856
ros-noetic-move-base 1.17.3-1focal.20250521.010519
ros-noetic-move-slow-and-clear 1.17.3-1focal.20250521.005242
ros-noetic-nav-core 1.17.3-1focal.20250521.004844
ros-noetic-navfn 1.17.3-1focal.20250521.005304
ros-noetic-rotate-recovery 1.17.3-1focal.20250521.010109
ros-noetic-voxel-grid 1.17.3-1focal.20250519.231756
```

## TEB Local Planner

- Source version: `0.9.1`
- Validated binary: `0.9.1-1focal.20250521.005922`
- Source package included under `lpb_ws/src/`: `teb_local_planner`.
- Upstream project: `teb_local_planner`, commonly distributed from `https://github.com/rst-tu-dortmund/teb_local_planner`.
- License declaration preserved in package source: BSD.
- Local package modifications detected in installed `/opt/ros/noetic` package-managed files: none from `dpkg -V`.

## Neo Local Planner

- Repository-contained package: `neo_local_planner`.
- ROS plugin used by the studies: `neo_local_planner/NeoLocalPlanner`.
- Source project identified in the research: `https://github.com/neobotix/neo_local_planner`.
- Package metadata version retained locally: `1.0.1`.
- The repository-contained Neo Local Planner source is retained from the Neobotix source used during development. The package's original authorship, license metadata, and license notices are preserved with the source.
- Retained package authorship metadata includes Max Wittal / Neobotix.
- Exact revision not recoverable from retained local metadata.

## Hybrid Local Planner

- Repository-contained package: `hybrid_local_planner`.
- ROS plugin used by the studies: `hybrid_local_planner/HybridPlannerROS`.
- Source project identified in the research: `https://github.com/fahimfss/Hybrid_Local_Planner`.
- Publication reference identified in the research: Fahim Shahriar, *Hybrid Local Planner: An Implementation of ROS Local Planner Using the Dijkstra's Algorithm and the Hybrid A Star Algorithm*, Version v1.0.0, DOI `10.5281/zenodo.15225866`.
- Package metadata version retained locally: `0.0.1`.
- The repository-contained Hybrid Local Planner source retains the original authorship and license notices distributed with the retained source copy.
- Retained package authorship metadata includes Fahim Shahriar.
- Exact revision not recoverable from retained local metadata.

## TurtleBot3 Source Packages

- Repository-contained packages: `turtlebot3`, `turtlebot3_msgs`, `turtlebot3_simulations`.
- Upstream project family: ROBOTIS TurtleBot3, commonly distributed from `https://github.com/ROBOTIS-GIT/turtlebot3`, `https://github.com/ROBOTIS-GIT/turtlebot3_msgs`, and `https://github.com/ROBOTIS-GIT/turtlebot3_simulations`.
- Research baseline for default DWA and Trajectory Rollout configurations: TurtleBot3 ROS Noetic repository configuration used in the study.
- Package metadata versions retained locally: `turtlebot3` 1.2.6, `turtlebot3_msgs` 1.0.1, `turtlebot3_simulations` 1.3.2.
- Retained license declarations: Apache 2.0 and/or BSD depending on package.
- Exact revision not recoverable from retained local metadata.
- Portability patch: the documented benchmark launch chain uses `$(optenv TURTLEBOT3_MODEL burger)` in `turtlebot3_navigation/launch/turtlebot3_navigation.launch`, `turtlebot3_navigation/launch/move_base.launch`, and `turtlebot3_bringup/launch/turtlebot3_remote.launch` instead of requiring `TURTLEBOT3_MODEL` to be defined. This does not alter the benchmark model or planner behavior; it preserves environment-variable override support while making Burger the fallback used by the documented benchmark.

## Source Artifact SHA256

```text
02e475846d47382496b1f107158910b645298ebf29d4694dfce283d580d3f9b8  ros-noetic-teb-local-planner_0.9.1-1focal.debian.tar.xz
07ebc6ee9b65e89792252d6f83807c4d319bdba12116f3214a193c7f06b614d2  ros-noetic-move-base_1.17.3-1focal.dsc
09c0ee016b66275b9dd0115bc0814fa16b33aa0850f6b79767b34d00806f8942  ros-noetic-clear-costmap-recovery_1.17.3-1focal.dsc
110097861b202901c03582078048602d5138f9a0c43643cfa44b3165980882b2  ros-noetic-nav-core_1.17.3.orig.tar.gz
12b807523c4f24e2ee49caeed92b57e3d70c9134189ec927029c426c4de2fd3b  ros-noetic-teb-local-planner_0.9.1.orig.tar.gz
12e6d7b9d4d1101a5704418377b9cc79e413008e89976e2ae64a47b3c0f4d574  ros-noetic-map-server_1.17.3-1focal.dsc
1353a1371ffb1704651d7abe533877fb06908a50b2909bd3a7f4ada6451c6da1  ros-noetic-navfn_1.17.3-1focal.debian.tar.xz
18ab6b6d82dc29da78b6bbf6bef0ab0d42cb33347a73bd21e697c9097cb8c13a  ros-noetic-voxel-grid_1.17.3.orig.tar.gz
268bd3a575e6abfd68310f6ddbc1c9fd4ac7a3b96126a976e00ae1df23db4271  ros-noetic-fake-localization_1.17.3.orig.tar.gz
26f6bda463fe9ad60e727e96bd6cfc7fddb08d1e958c1a01b82e5554784dc94d  ros-noetic-costmap-2d_1.17.3.orig.tar.gz
295f7ddf85782ed6e2852d8ca487732e5eb88cea3f870b2ed265a71055010199  ros-noetic-carrot-planner_1.17.3.orig.tar.gz
2a67ae1ffed7b729f8a73829e80d7b199caeabecbad3d0ea1828fecfc65893c6  ros-noetic-fake-localization_1.17.3-1focal.dsc
2ca03142c4691a23ac8740a87d88e8fbd4ebba6b255d9f932600bb90d50b36e9  ros-noetic-move-slow-and-clear_1.17.3-1focal.dsc
367786421e5579ab28d34be8816156176af9c45687b221d9e066a08a12725eff  ros-noetic-rotate-recovery_1.17.3-1focal.dsc
374275606bcb797fe09f51760f51be56bcd7011d09b1160200e048733147d4bb  ros-noetic-carrot-planner_1.17.3-1focal.debian.tar.xz
37d3cc360d56da8756252b84e1cf99654f6bb7a3d26be9f39c0295b3e55b9609  ros-noetic-move-base_1.17.3-1focal.debian.tar.xz
3da7ade956e6a9cbe9d9f39e867e01e8ea24addab0cd5141de668c21820a7f7c  ros-noetic-navfn_1.17.3-1focal.dsc
3f000e85e7335a4b288a62ead95abf14c134ea0ec1aa3b3efa262ae50700b837  ros-noetic-teb-local-planner_0.9.1-1focal.dsc
4156a6b9ce9f25db07347e75a3bd25c354bed379dc4bc380a175710b7321a6da  ros-noetic-base-local-planner_1.17.3-1focal.dsc
42232a3647060a5cab2231aa80b2a35bc360a0417182e342aa3982f2f796572d  ros-noetic-rotate-recovery_1.17.3.orig.tar.gz
42d71c623e6bcb968dceb07b6e659e7473d89ca4a0a5f48a165d7879f28f5072  ros-noetic-amcl_1.17.3.orig.tar.gz
43c7e371b6d53e1a06e4aa7c616e4428f71368b7753110c8eafed2c5b3800cf2  ros-noetic-dwa-local-planner_1.17.3-1focal.dsc
456905f9ff0033ff227a17d955b2a5e17b38f668d7965545e86b1d65e154a8a5  ros-noetic-global-planner_1.17.3-1focal.dsc
4bf889a522a2795b1493f48b67c7993db2dafad89debacf2cce5f9ccb1ba20b8  ros-noetic-costmap-2d_1.17.3-1focal.dsc
4e3c02981c0830bb70c5dfaffd8c27d18444477ab403d859c58c7942da7a3fd1  ros-noetic-amcl_1.17.3-1focal.dsc
52c478fdf5e24eb323940f4e8e7673bacc08e35660ed55cecc591169203d309a  ros-noetic-global-planner_1.17.3-1focal.debian.tar.xz
5398dafe86165c9160050a78b4769323312f8b16f8f98c48a0a004124d07bf82  ros-noetic-move-slow-and-clear_1.17.3.orig.tar.gz
5d09c47df064ee9ae2eba3d22410695978fa329812b037b0dbbded1a5a6965e0  ros-noetic-dwa-local-planner_1.17.3-1focal.debian.tar.xz
5fe34dd7476ca9955f6bb2f286076afda7a4106cf44b1eff345be78d67f785b8  ros-noetic-carrot-planner_1.17.3-1focal.dsc
62b7ab0c5040282a70e3f245a1a8468e740066967d4ca5c4066049ae1170f934  ros-noetic-navigation_1.17.3-1focal.debian.tar.xz
6827908035ea8c7a5a928671a63505e31890d6b41bd5edddeb393e61eba11d4d  ros-noetic-map-server_1.17.3-1focal.debian.tar.xz
71cc95dd279ae1517db303d717c63ae4f9942b98dca8e627c4b888f2fdc66b60  ros-noetic-nav-core_1.17.3-1focal.debian.tar.xz
745593aa0a72651a3dc94bca69abef7c19798c456de89decefe426c33b3b505d  ros-noetic-nav-core_1.17.3-1focal.dsc
753bdea7423e1350c9710569a14d4609690fed1745af4ec1560ce204e2043d79  ros-noetic-amcl_1.17.3-1focal.debian.tar.xz
7828531ece071223c9443628a549f2aa1685928c7a39de090d51aaa2979b2161  ros-noetic-voxel-grid_1.17.3-1focal.debian.tar.xz
83ef052d374b11b1118811fceb1184d07f9a5ae80850fc9bc31f0ecf85663bf7  ros-noetic-rotate-recovery_1.17.3-1focal.debian.tar.xz
8c7f869be04aadd3b620399be287e90fbd8f008bc5b19e7ed50b9846e6a41aaa  ros-noetic-clear-costmap-recovery_1.17.3-1focal.debian.tar.xz
8c9323b10c945aad311462ea69cd4f31eba540e17d19d75e1b9a8288bb741b65  ros-noetic-navigation_1.17.3-1focal.dsc
8f595e2e6d6cfb89a56647c04b0c77c6082e2bdd215f50fece44e553a42eb5c4  ros-noetic-costmap-2d_1.17.3-1focal.debian.tar.xz
a066b40d6145de84e7a29b7fa00d77084575732d0ad435a8ebc729973eb08506  ros-noetic-navfn_1.17.3.orig.tar.gz
aea3e6c5a720d80c33ec397a0fa0bf8cd065e71cbd986393f89fe309febcc926  ros-noetic-global-planner_1.17.3.orig.tar.gz
afdb9758be117e9eaf513d60bfa4f7800aad685b82e0a1f9e144c9a8b8e435d3  ros-noetic-navigation_1.17.3.orig.tar.gz
b4f5bfcc311888d33258fac05e542b72193cf185d0737dbe8ba81145533b53f0  ros-noetic-voxel-grid_1.17.3-1focal.dsc
c063b4c4297890c08b81667955ac8339655f64a9aed3de33470a838f880e83e5  ros-noetic-move-base_1.17.3.orig.tar.gz
cbc5aebed705c6a31294908e7c23b1f7a40415878b6a4becd0f60d6a2451d335  ros-noetic-dwa-local-planner_1.17.3.orig.tar.gz
e055caf9b0a2bd665b1fc795485034d9d3f85da6cf6f011309bc6d35c54a01c4  ros-noetic-base-local-planner_1.17.3-1focal.debian.tar.xz
e9ea10594dc9f0ef68619614306bbd2b2847aa72fd0ff8fc476c02fb81dc60ec  ros-noetic-move-slow-and-clear_1.17.3-1focal.debian.tar.xz
f2df50aa9a26b05464f433a03c917273051a3430f96ff6a46361a77a64d867ac  ros-noetic-clear-costmap-recovery_1.17.3.orig.tar.gz
f3a0cb47e3d92155d1c77d878149c514345206b4be73a085dd184feb19fc3bd5  ros-noetic-base-local-planner_1.17.3.orig.tar.gz
f82ab553972b4f836b8fc3735f6ff26fdf1d18ceb3e7255ce6de56a840eadd15  ros-noetic-fake-localization_1.17.3-1focal.debian.tar.xz
fc23b895f33a304d99d5b767a9a1e8255a35ee22462515aa655256734e1de61a  ros-noetic-map-server_1.17.3.orig.tar.gz
```

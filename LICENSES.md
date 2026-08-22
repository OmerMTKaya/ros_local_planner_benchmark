# License Notice

This is a mixed-license repository. The root `LICENSE` file is a scope notice, not a claim that one license applies uniformly to every file.

## Research-Authored Material

Original research-authored materials are licensed under GPL-3.0-only where the repository owner has the right to license them. This scope includes:

- Root documentation: `README.md`, `DEPENDENCIES.md`, `SOURCE_PROVENANCE.md`, `CITATIONS.bib`, `CITATION.cff`, `LICENSE`, `LICENSES.md`.
- Study documentation under `studies/`.
- Research experiment package material under `lpb_ws/src/test/`, excluding upstream or third-party material if any is separately identified inside that package.
- Research configuration package material under `lpb_ws/src/my_nav_cfg/`.

The full GPLv3 text is provided in `LICENSES/GPL-3.0-only.txt`.

## Upstream Source Packages

Repository-contained upstream components retain their own package-level license metadata, copyright notices, and retained `LICENSE`/`COPYING` files. Do not treat the research GPL scope as relicensing these components.

- ROS Navigation Stack packages: `amcl`, `base_local_planner`, `carrot_planner`, `clear_costmap_recovery`, `costmap_2d`, `dwa_local_planner`, `fake_localization`, `global_planner`, `map_server`, `move_base`, `move_slow_and_clear`, `nav_core`, `navfn`, `navigation`, `rotate_recovery`, `voxel_grid`.
  Retained package metadata and source notices identify BSD and LGPL-family licensing depending on package.
- TEB Local Planner: `teb_local_planner`.
  Retained package metadata and `LICENSE` file identify BSD licensing.
- Neo Local Planner: `neo_local_planner`.
  The retained source copy preserves its own package-level authorship, license metadata, and license notices. The exact historical upstream revision is not recoverable from retained local metadata.
- Hybrid Local Planner: `hybrid_local_planner`.
  The retained source copy preserves its own package-level authorship, license metadata, and license notices. The exact historical upstream revision is not recoverable from retained local metadata.
- TurtleBot3 source packages: `turtlebot3`, `turtlebot3_msgs`, `turtlebot3_simulations`.
  Retained package metadata and `LICENSE` files identify Apache 2.0 and/or BSD licensing depending on package.

## Citation

Academic citation is requested as scholarly attribution, not as an additional GPL legal condition. See `CITATIONS.bib` and `CITATION.cff`.

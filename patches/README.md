# External dependency patches

The large upstream repositories are intentionally not stored in this repository.
Their exact base revisions are pinned in `config/dependencies.repos`.

- `px4-ca-lsc-transition-allocation.patch` contains the CA-LSC transition
  allocation, uORB/DDS, SITL startup, and lockstep changes for PX4-Autopilot
  base commit `4817c0618a1286846116e90c6eb8919efaa013cf`.
- `px4-msgs-ca-lsc-topics.patch` exposes the corresponding custom
  `NormalizedUnsignedSetpoint` topic aliases in `px4_msgs` base commit
  `a1045ec4feb6d709bdecaf3895f1d5b43a5dabb8`.

Run `scripts/bootstrap_dependencies.sh` from anywhere after cloning the main
repository. The script imports the pinned repositories, applies each patch once,
and initializes PX4 submodules.

To verify an already prepared checkout manually:

```bash
git -C PX4-Autopilot apply --reverse --check \
  ../patches/px4-ca-lsc-transition-allocation.patch
git -C ros2_ws/src/px4_msgs apply --reverse --check \
  ../../../patches/px4-msgs-ca-lsc-topics.patch
```

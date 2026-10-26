# Mapping integration handoff — R16-173

## Status

**Integration-test candidate, not final mapping release.**

- ROS 2 Jazzy / Gazebo / slam_toolbox / Nav2
- Mapping functional tests: 141 passed, 3 deselected
- Critical Python lint: passed
- Mapping-only colcon build: passed
- Navigation PR #4: independently maintained, unchanged
- Safety sources: integrity verified

## Implemented behavior

- Frontier detection and clearance-qualified viewpoint selection
- Nav2 ComputePathToPose validation
- Verified-path risk screening
- Failure memory and candidate rotation
- Navigation with live safety checks
- Recovery from confirmed transient safety cancellations
- Recovery waiting for confirmed obstacle-hold clearance
- Raw OccupancyGrid map saving and strict completion verification

## Known limitations

The previous Gazebo acceptance run, before the R16-173
obstacle-hold recovery change, was not a final success:

- Nav2 goals sent: 18
- Arrived: 7
- Safety cancellations: 11
  - TF_NOT_STABLE: 10
  - OBSTACLE_HOLD_NOT_CLEAR: 1
- Raw map saved: yes
- Map complete: no
- Free/unknown boundary cells: 67

The R16-173 obstacle-hold recovery change has passed offline
tests and build but has NOT been revalidated in Gazebo.

Intermittent TF instability remains under investigation.
SLAM configuration in the main workspace and PR4 review
workspace was confirmed equivalent for inspected parameters.

## Safety and integration requirements

- Do not disable Collision Monitor or obstacle hold.
- Do not bypass TF freshness checks.
- Preserve the Mapping occupied-cell path clearance of 0.48 m.
- Use real Nav2 planner responses; do not synthesize paths.
- Do not treat a saved map as a completed verified map.
- Confirm compatible navigation, Gazebo and robot interfaces
  before starting real autonomous motion.

## Sharing scope

This branch is for cross-team integration testing and review.
Final mapping acceptance remains pending.

---
phase: 08-demo-visualization
plan: 02
subsystem: visualization
tags: [ros2, rviz, path, launch, visualization]

# Dependency graph
requires:
  - phase: 07-03
    provides: NavigationNode with /navigation/odometry topic and TF broadcast
provides:
  - PathPublisherNode for trajectory accumulation and visualization
  - RViz configuration with dual path display
  - Launch file with scenario selection and RViz integration
affects: [08-demo-visualization]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - Path publisher pattern (odometry to path accumulation)
    - OpaqueFunction for scenario-based launch configuration
    - RViz TopDownOrtho view for 2D trajectory visualization

key-files:
  created:
    - src/usbl_navigation/include/usbl_navigation/path_publisher_node.hpp
    - src/usbl_navigation/src/nodes/path_publisher_node.cpp
    - src/usbl_navigation/src/nodes/path_publisher_main.cpp
    - src/usbl_navigation/config/rviz_config.rviz
  modified:
    - src/usbl_navigation/CMakeLists.txt
    - src/usbl_navigation/launch/simulation.launch.py

key-decisions:
  - "2Hz path publish rate for RViz efficiency (not every odometry update)"
  - "1000 point max path length to prevent memory growth"
  - "TopDownOrtho camera for 2D lawnmower pattern visualization"
  - "OpaqueFunction for scenario-based parameter overrides"

patterns-established:
  - "Path publisher pattern: subscribe Odometry, accumulate poses, publish Path"
  - "Scenario-based launch: OpaqueFunction with choices argument"

# Metrics
duration: 6min
completed: 2026-01-20
---

# Phase 08 Plan 02: RViz Configuration and Launch File Summary

**PathPublisherNode for trajectory visualization with RViz config showing truth (blue) vs estimate (orange) paths, and launch file with scenario selection**

## Performance

- **Duration:** 6 min
- **Started:** 2026-01-20T23:02:00Z
- **Completed:** 2026-01-20T23:08:00Z
- **Tasks:** 3
- **Files modified:** 6

## Accomplishments

- Created PathPublisherNode that accumulates odometry poses into nav_msgs/Path for RViz
- Created RViz configuration with Grid, TF, dual Path displays (truth blue, estimate orange)
- Updated launch file with scenario argument (nominal, canyon_dropout, high_outlier)
- Added rviz argument to optionally launch RViz with config
- Added two path_publisher_node instances for truth and estimate trajectories

## Task Commits

Each task was committed atomically:

1. **Task 1: Create path publisher nodes for visualization** - `1b25a16` (feat)
2. **Task 2: Create RViz configuration file** - `9e58318` (feat)
3. **Task 3: Update launch file and CMakeLists** - `34b6550` (feat)

## Files Created/Modified

- `src/usbl_navigation/include/usbl_navigation/path_publisher_node.hpp` - PathPublisherNode class declaration
- `src/usbl_navigation/src/nodes/path_publisher_node.cpp` - Path accumulation and publishing logic
- `src/usbl_navigation/src/nodes/path_publisher_main.cpp` - Node entry point
- `src/usbl_navigation/config/rviz_config.rviz` - RViz configuration with dual paths, TF, grid
- `src/usbl_navigation/CMakeLists.txt` - Added path_publisher_node executable and install
- `src/usbl_navigation/launch/simulation.launch.py` - Scenario selection, rviz flag, path publishers

## Decisions Made

1. **2Hz path publish rate** - More efficient than publishing on every odometry update (50Hz)
2. **1000 point max path length** - Prevents unbounded memory growth during long runs
3. **TopDownOrtho camera** - Best view for 2D lawnmower pattern visualization
4. **OpaqueFunction launch pattern** - Enables scenario-based parameter overrides with choices validation
5. **Z offset for estimate path** - Prevents z-fighting when truth and estimate overlap

## Deviations from Plan

None - plan executed exactly as written.

## Issues Encountered

None.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- Path publishers ready for trajectory visualization
- RViz config installed and launchable with `rviz:=true`
- Scenario selection enables nominal, canyon_dropout, and high_outlier testing
- Ready for Plan 08-03: Scenario testing and metrics analysis

---
*Phase: 08-demo-visualization*
*Completed: 2026-01-20*

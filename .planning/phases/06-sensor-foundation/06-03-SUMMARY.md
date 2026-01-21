---
phase: 06-sensor-foundation
plan: 03
subsystem: navigation
tags: [ros2, eigen, usbl, delayed-state, outlier-injection, sensor-simulation]

# Dependency graph
requires:
  - phase: 06-01
    provides: usbl_navigation package structure, truth generator node
provides:
  - USBL simulator with delayed timestamps at 0.2Hz
  - Range-dependent noise model (2% of range, minimum 0.3m)
  - 10% dropout probability with validity status
  - 5% outlier injection with 5m random offset
  - Position buffer for historical state lookup
affects: [07-navigation-filter, 08-demo-visualization]

# Tech tracking
tech-stack:
  added: []
  patterns: [delayed timestamp USBL simulation, circular buffer for time-based lookup, range-dependent noise]

key-files:
  created:
    - src/usbl_navigation/include/usbl_navigation/usbl_simulator_node.hpp
    - src/usbl_navigation/src/nodes/usbl_simulator_node.cpp
    - src/usbl_navigation/src/nodes/usbl_simulator_main.cpp
  modified:
    - src/usbl_navigation/CMakeLists.txt
    - src/usbl_navigation/launch/simulation.launch.py

key-decisions:
  - "USBL publishes with measurement_time (current - delay), not current time"
  - "Range-dependent noise: max(2% * range, 0.3m) per axis"
  - "Position buffer stores 100 entries for interpolated historical lookup"
  - "Outliers inject 5m offset in random direction"

patterns-established:
  - "Delayed timestamp pattern critical for testing delayed-state EKF"
  - "Linear interpolation between buffered positions for delay lookup"
  - "Validity topic published alongside measurement for dropout indication"

# Metrics
duration: 6min
completed: 2026-01-21
---

# Phase 6 Plan 03: USBL Simulator Summary

**USBL simulator with delayed timestamps (0.2Hz), range-dependent noise, 10% dropouts, and 5% outlier injection for testing delayed-state EKF**

## Performance

- **Duration:** 6 min
- **Started:** 2026-01-21T01:55:00Z
- **Completed:** 2026-01-21T01:55:43Z
- **Tasks:** 3
- **Files modified:** 5

## Accomplishments

- USBL simulator publishing at 0.2Hz (5 second period) with DELAYED timestamps
- Range-dependent Gaussian noise: max(2% * range, 0.3m) on each axis
- 10% dropout probability with /usbl/valid=false indication
- 5% outlier injection with 5m random direction offset
- Position buffer for interpolated historical position lookup
- All 4 sensor nodes (truth, IMU, DVL, USBL) launch together from single launch file

## Task Commits

Each task was committed atomically:

1. **Task 1: Create usbl_simulator_node header** - `68017c5` (feat)
2. **Task 2: Implement usbl_simulator_node with delayed timestamps** - `d78ee40` (feat)
3. **Task 3: Update CMakeLists.txt and launch file** - `3da7cc7` (feat)

**Plan metadata:** (this commit)

## Files Created/Modified

- `src/usbl_navigation/include/usbl_navigation/usbl_simulator_node.hpp` - USBL simulator class with buffer and noise parameters
- `src/usbl_navigation/src/nodes/usbl_simulator_node.cpp` - USBL simulation with delays, noise, dropouts, outliers
- `src/usbl_navigation/src/nodes/usbl_simulator_main.cpp` - USBL node entry point
- `src/usbl_navigation/CMakeLists.txt` - Added usbl_simulator_node executable
- `src/usbl_navigation/launch/simulation.launch.py` - Added usbl_simulator_node to launch

## Decisions Made

- **Delayed timestamp pattern**: USBL publishes with `measurement_time = now() - delay`, not current time
- **Range-dependent noise**: noise_std = max(range_noise_percent * range, min_noise) = max(2% * range, 0.3m)
- **Position buffer**: 100-entry circular buffer with linear interpolation for historical lookup
- **Outlier direction**: Random unit vector scaled by outlier_offset (5m)
- **Statistics tracking**: Total measurements, dropout count, outlier count for logging

## Deviations from Plan

None - plan executed exactly as written.

## Issues Encountered

None - all tasks completed successfully.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- Phase 6 complete - all sensor simulators operational
- Topics published:
  - /truth/odometry (100Hz), /truth/path (1Hz)
  - /imu/data (100Hz)
  - /dvl/twist (5Hz), /dvl/bottom_lock (5Hz)
  - /usbl/position (0.2Hz with DELAYED timestamps), /usbl/valid (0.2Hz)
- Ready for Phase 7 (Navigation Filter with delayed-state EKF)
- USBL delayed timestamps will test the state buffer and repropagation logic

---
*Phase: 06-sensor-foundation*
*Completed: 2026-01-21*

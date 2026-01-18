---
phase: 03-control
plan: 02
subsystem: control
tags: [pid, ros2, cpp, auv-control, thruster, depth, heading, velocity]

# Dependency graph
requires:
  - phase: 03-01
    provides: "Generic PID class and control node skeleton with EKF subscriber"
provides:
  - Three functional PID controllers (depth, heading, velocity)
  - Configurable PID gains via ROS2 parameters
  - Angle wrapping for heading discontinuity handling
  - Thruster command publishing to all axes
affects: [03-03, 04-planning, 05-demo]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "dt-based PID computation with invalid timestep rejection"
    - "Angle wrapping for heading error in [-pi, pi]"
    - "Control enable flags for axis isolation"

key-files:
  created: []
  modified:
    - src/auv_control/include/auv_control/control_node.hpp
    - src/auv_control/src/control_node.cpp

key-decisions:
  - "Depth error = target - current (positive when shallower than target)"
  - "Heading error uses wrapAngle to avoid 270-degree spinning"
  - "Yaw bow and stern get same command (stern inverted in XML)"
  - "1Hz state logging for debugging"

patterns-established:
  - "Control loop dt rejection for invalid timesteps (<=0 or >1s)"
  - "Per-axis enable flags for selective control"

# Metrics
duration: 2min
completed: 2026-01-18
---

# Phase 03 Plan 02: PID Controllers Summary

**Three PID controllers (depth, heading, velocity) computing errors and publishing thruster commands with angle wrapping and configurable gains**

## Performance

- **Duration:** 2 min
- **Started:** 2026-01-18T02:41:42Z
- **Completed:** 2026-01-18T02:43:38Z
- **Tasks:** 2
- **Files modified:** 2

## Accomplishments

- Implemented depth controller with heave thruster output (±50N limit)
- Implemented heading controller with angle wrapping and yaw thruster output (±30N limit)
- Implemented velocity controller with surge thruster output (±100N limit)
- Added configurable PID gains via ROS2 parameters for runtime tuning

## Task Commits

Each task was committed atomically:

1. **Task 1: Add PID controllers and parameters to control node** - `c04154b` (feat)
2. **Task 2: Implement controlLoop with all three controllers** - `65b8525` (feat)

## Files Created/Modified

- `src/auv_control/include/auv_control/control_node.hpp` - Added PID controller pointers, enable flags, last_control_time_, wrapAngle declaration
- `src/auv_control/src/control_node.cpp` - PID parameter declaration/instantiation, full controlLoop implementation, wrapAngle helper

## Decisions Made

1. **Error sign convention** - Depth error = target - current, so positive error means AUV is shallower than target, requiring positive heave force
2. **Angle wrapping** - wrapAngle() normalizes heading error to [-pi, pi] to prevent 270-degree spins
3. **Yaw thruster pairing** - Both yaw_bow and yaw_stern receive identical commands since stern is inverted in auv.xml
4. **dt validation** - Skip control loop if dt <= 0 or dt > 1.0 to avoid PID spikes

## Deviations from Plan

None - plan executed exactly as written.

## Issues Encountered

None.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- All three PID controllers are functional
- Ready for Plan 03-03: Launch file and full control system verification
- Controllers will be tested with actual EKF input in Docker environment

---
*Phase: 03-control*
*Completed: 2026-01-18*
